"""
===================================================
  eLSI Sprint 1 - Task 2B : PID Line Following + Pick & Place (dual line)
===================================================

Participant template (PID variant).

TASK 2B
  Follow the track (white line on black AND black line on white) through the
  checkpoints, pick the red and blue boxes near the circle, drop each in its
  matching colour drop zone, then finish at the white box.
  Boxes are handled ONE AT A TIME: pick one, deliver it, come back for the other.

HOW TO RUN
  1. Open the Task 2B scene in CoppeliaSim.
  2. Start the bridge:   python3 bridge_v1_2b.py --eval
  3. Run this file:      python3 task2b_pid_template.py

WHAT YOU IMPLEMENT
  control_loop()  - PID controller that returns (left_speed, right_speed).
  detect_color()  - identify the box colour from the RGB sensor.
  should_pick()   - decide when to pick a box (only when one is right next to you).
  should_drop()   - decide when to drop the carried box (at its matching zone).

Everything else (connecting, receiving sensors, sending motor/pick/drop
commands) is handled by CoppeliaClient. Don't edit outside the marked TODO
sections. You may add helper functions.

SENSOR PROTOCOL (from bridge_v1_2b.py):
  Line sensors:  'left_corner','left','middle','right','right_corner' — [0,1].
                 NOTE: this track has BOTH white-line-on-black and
                 black-line-on-white sections, so "on the line" is not always
                 "high" — design your error term to handle both.
  Proximity:     'proximity' — metres to nearest object; 1.0 = nothing in range.
  Color sensor:  'color_r','color_g','color_b' — [0,1].

Team ID: [ XXX ]
"""

import time

from connector_2b import CoppeliaClient

# The five line sensors, ordered left -> right across the robot ([0.0, 1.0]).
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']


# =============================================================================
#  TODO (participants): implement the four functions below.
#  You may add helper functions anywhere in this section.
# =============================================================================

# Only the TODO section is implemented.  The main loop below is unchanged.
# A CSV named task2b_debug.csv is created in the same folder on every run.

import csv

# -----------------------------------------------------------------------------
# STABLE ADAPTIVE TURBO PROFILE — 1 minute 35 second target
# -----------------------------------------------------------------------------
#
# CSV 33 total wall runtime:
#     968.5 s = 16 min 8.5 s
#
# A plain 10x multiplier would theoretically take about 96.9 s, not 60 s.
# This adaptive profile targets approximately 93–100 s from the CSV distribution:
#
#   centred narrow line : 13.5x
#   moderate curve      : 12x
#   stronger curve      :  8x
#   highlighted node    : 5.5x
#   special manoeuvre   :  6x
#   blue/red circle     :  6x
#   temporarily lost    :  4x
#
# Motor commands are accelerated, but controller timers remain at normal rate.
# This is required when CoppeliaSim playback is slowed manually: the robot gets
# enough physical movement over each node before its turn phase can advance.
#
# This is a stability-first 1 minute 35 second target profile. The original working file
# remains separate and unchanged.
TURBO_ENABLED = True

# Keep False while running CoppeliaSim in manually slowed playback mode.
# Wheel speeds remain turbo-scaled, but junction/drop/circle timers do not.
TURBO_SCALE_ROUTE_TIMERS = False

TURBO_CENTER_FACTOR = 13.5
TURBO_MODERATE_FACTOR = 12.0
TURBO_CURVE_FACTOR = 8.0
TURBO_NODE_FACTOR = 5.5
TURBO_MANOEUVRE_FACTOR = 6.0
TURBO_CIRCLE_FACTOR = 6.0
TURBO_LOST_FACTOR = 4.0

# Tiny highlighted node on the BLUE main path. This local factor affects only
# the few frames spent crossing that node, so the overall 95-second target is
# practically unchanged.
TURBO_BLUE_SKIP_NODE_FACTOR = 4.5

# Third white junction -> black return path.
# Slightly slower than the generic 6x manoeuvre speed for reliable line lock.
TURBO_BLUE_RETURN_LEFT_FACTOR = 4.8

# Post-BLUE-drop RIGHT node and its short PID handoff.
# This avoids the old 6x continuous turn while preserving overall fast speed.
TURBO_BLUE_RETURN_RIGHT_FACTOR = 4.8

TURBO_CENTER_ERROR = 0.10
TURBO_MODERATE_ERROR = 0.28

# Prevent a one-frame PID spike from sending an extreme wheel command.
# Normal centred cruise is approximately 20.5, so 22 keeps full cruise speed.
TURBO_MAX_ABS_MOTOR = 30.0

_turbo_real_start = time.perf_counter()
_turbo_real_last = _turbo_real_start
_turbo_virtual_now_value = _turbo_real_start
_turbo_factor = 1.0


def _turbo_now():
    """Logical route time accelerated in proportion to the motor multiplier."""
    return _turbo_virtual_now_value


def _advance_turbo_clock(factor):
    """Advance controller time without breaking slowed CoppeliaSim playback.

    Motor commands still use ``factor``. Route timers normally advance at 1x,
    so the robot physically covers the node before a timed turn progresses.
    """
    global _turbo_real_last, _turbo_virtual_now_value

    real_now = time.perf_counter()
    real_dt = real_now - _turbo_real_last
    _turbo_real_last = real_now

    # Avoid a large logical-time jump after a blocking PICK/DROP reply.
    real_dt = max(0.0, min(0.25, real_dt))

    timer_factor = factor if TURBO_SCALE_ROUTE_TIMERS else 1.0
    _turbo_virtual_now_value += real_dt * timer_factor


def _choose_turbo_factor(sensors):
    """Select speed from current route phase and current line geometry."""
    if not TURBO_ENABLED:
        return 1.0

    if _route_state == FINISHED:
        return 1.0

    phase = (_last_manoeuvre_phase or "").lower()

    # The ignored BLUE main-path node needs a controlled equal-wheel crossing.
    if _manoeuvre_kind == "blue_main_skip_straight":
        return TURBO_BLUE_SKIP_NODE_FACTOR

    if (
        _manoeuvre_kind == "blue_return_left_black_lock"
        or _blue_return_left_handoff_frames > 0
    ):
        return TURBO_BLUE_RETURN_LEFT_FACTOR

    if (
        _manoeuvre_kind == "blue_return_right_lock"
        or _blue_return_right_handoff_frames > 0
    ):
        return TURBO_BLUE_RETURN_RIGHT_FACTOR

    # Dedicated node/branch manoeuvres and their capture windows must remain
    # slower than long cruise sections.
    if (
        _manoeuvre_kind is not None
        or "handoff" in phase
        or "capture" in phase
        or "search" in phase
        or "acquire" in phase
    ):
        return TURBO_MANOEUVRE_FACTOR

    # Keep box pickup laps controlled; high straight-line speed is unnecessary
    # inside the small circle.
    if _route_state in (BLUE_CIRCLE, RED_CIRCLE):
        return TURBO_CIRCLE_FACTOR

    raw = _raw_values(sensors)
    active = _active_values(raw)
    position, active_sum = _line_position(active)
    strong_count = sum(
        value >= NODE_ACTIVE_THRESHOLD for value in active
    )

    # Wide black/white highlighted nodes require reliable one-frame detection.
    if strong_count >= 3 or active_sum >= 2.60:
        return TURBO_NODE_FACTOR

    if position is None:
        return TURBO_LOST_FACTOR

    error_magnitude = abs(position)

    if error_magnitude <= TURBO_CENTER_ERROR:
        return TURBO_CENTER_FACTOR
    if error_magnitude <= TURBO_MODERATE_ERROR:
        return TURBO_MODERATE_FACTOR
    return TURBO_CURVE_FACTOR



# -----------------------------------------------------------------------------
# PID and route configuration
# -----------------------------------------------------------------------------
WEIGHTS = (-2.0, -1.0, 0.0, 1.0, 2.0)

KP = 1.40
KI = 0.00
KD = 0.54

BASE_SPEED = 2.05
MIN_BASE_SPEED = 1.20
MAX_MOTOR_SPEED = 2.80

# Sensor thresholds. These are deliberately grouped here so that the values can
# be tuned easily after examining task2b_debug.csv.
NODE_ACTIVE_THRESHOLD = 0.58
NODE_REQUIRED_SENSORS = 4
PROXIMITY_PICK_THRESHOLD = 0.30
LINE_SWITCH_FRAMES = 3

# The robot initially stands on a wide black start marker. That marker looks
# like a junction to the five sensors, so route-node detection remains disabled
# until the robot first reaches an ordinary narrow section of line.
START_CLEAR_REQUIRED_FRAMES = 1
START_GATE_MIN_SECONDS = 0.45

# First real junction correction (measured from task2b_debug CSV):
# Do NOT turn when only the inner three sensors first touch the wide dot.  That
# happens about seven control frames (~0.39 s) too early and places the pivot
# before the connector mouth.  Arm the turn only after all five sensors have
# seen the highlighted dot for two consecutive frames.
FIRST_NODE_ACTIVE_THRESHOLD = 4.55
FIRST_NODE_REQUIRED_SENSORS = 5

# Second highlighted black spot: entrance of the circle.
# Do not begin the blue-circle turn when the robot only touches one edge of
# this spot.  CSV runs 10/11/12 show the unreliable trigger near active_sum
# 3.9 with only four strong sensors.  The reliable run reached the centre of
# the spot with all five sensors active and active_sum close to 4.75.
CIRCLE_ENTRY_ACTIVE_THRESHOLD = 4.45
CIRCLE_ENTRY_REQUIRED_SENSORS = 5
CIRCLE_ENTRY_REQUIRED_FRAMES = 1
CIRCLE_ENTRY_APPROACH_ACTIVE_SUM = 3.65

# CSV 13:
#   frame 1788 -> four sensors first touched the second black spot
#   frame 1790 -> the reliable right-turn manoeuvre started
# At ~20 Hz, forcing the centred-node condition after 0.04 s plus the existing
# two-frame debounce reproduces that successful trigger even when the fifth
# sensor misses the exact centre by a few millimetres.
CIRCLE_ENTRY_FORCE_AFTER_SECONDS = 0.04

# First junction turn phases:
# 1) move a little beyond the centre of the black dot,
# 2) take a forward right-hand arc so the robot enters the connector naturally,
# 3) use a short pivot only as a final line-search fallback.
FIRST_TURN_ADVANCE_SECONDS = 0.30
FIRST_TURN_ADVANCE_SPEED = 1.30
FIRST_TURN_CURVE_MIN_SECONDS = 0.42
FIRST_TURN_CURVE_MAX_SECONDS = 1.05
FIRST_TURN_SEARCH_SECONDS = 0.40
FIRST_TURN_OUTER_SPEED = 2.15
FIRST_TURN_INNER_SPEED = 0.28
FIRST_TURN_SEARCH_SPEED = 1.55

# Circle-entry correction for the BLUE visit.
# After the connector, the robot reaches the SECOND highlighted black spot.
# It must take the robot's RIGHT-side circle branch toward the blue box.
# In this simulator that arc is produced by left wheel fast, right wheel slow.
# The turn is completed only after the sensor pattern proves that the robot
# has acquired that right-side circle line, not merely the old connector line.
BLUE_ENTRY_ADVANCE_SECONDS = 0.14
BLUE_ENTRY_ADVANCE_SPEED = 1.35

# Replay the successful CSV-13 motor sequence:
# frame 1793 through frame 1816 used (2.18, 0.02), approximately 1.24 s.
# The turn then hands control to a gentle PID ramp; it does not start another
# junction manoeuvre or sharply turn back toward the connector.
BLUE_ENTRY_FIXED_RIGHT_TURN_SECONDS = 1.24
BLUE_ENTRY_OUTER_SPEED = 2.18
BLUE_ENTRY_INNER_SPEED = 0.02
BLUE_ENTRY_HANDOFF_SECONDS = 0.45
BLUE_ENTRY_HANDOFF_BASE_SPEED = 1.55
BLUE_ENTRY_HANDOFF_MAX_CORRECTION = 0.35

# BLUE circle exit: THIRD highlighted black node.
# After completing the full circle and collecting the blue box, the robot must
# turn RIGHT from this node onto the connector that returns to the main path.
BLUE_EXIT_ADVANCE_SECONDS = 0.24
BLUE_EXIT_ADVANCE_SPEED = 1.30
BLUE_EXIT_CURVE_MIN_SECONDS = 0.50
BLUE_EXIT_CURVE_MAX_SECONDS = 1.28
BLUE_EXIT_SEARCH_SECONDS = 0.45
BLUE_EXIT_OUTER_SPEED = 2.18
BLUE_EXIT_INNER_SPEED = 0.16
BLUE_EXIT_SEARCH_SPEED = 1.45
BLUE_EXIT_CENTER_REQUIRED_FRAMES = 1
BLUE_EXIT_HANDOFF_SECONDS = 1.10
BLUE_EXIT_HANDOFF_BASE_SPEED = 1.55
BLUE_EXIT_HANDOFF_MAX_CORRECTION = 0.30

# FOURTH highlighted black spot: connector -> RIGHT onto main path.
MAIN_ENTRY_ADVANCE_SECONDS = 0.22
MAIN_ENTRY_ADVANCE_SPEED = 1.30
MAIN_ENTRY_HARD_RIGHT_SPEED = 2.12
MAIN_ENTRY_HARD_INNER_SPEED = 0.12
MAIN_ENTRY_MEDIUM_RIGHT_SPEED = 1.90
MAIN_ENTRY_MEDIUM_INNER_SPEED = 0.42
MAIN_ENTRY_SOFT_RIGHT_SPEED = 1.70
MAIN_ENTRY_SOFT_INNER_SPEED = 0.92
MAIN_ENTRY_WHITE_SEEN_THRESHOLD = 0.58
MAIN_ENTRY_BLACK_FLOOR_THRESHOLD = 0.32
MAIN_ENTRY_CENTER_REQUIRED_FRAMES = 1
MAIN_ENTRY_MAX_SECONDS = 2.40
MAIN_ENTRY_PID_HANDOFF_SECONDS = 0.60
MAIN_ENTRY_PID_HANDOFF_BASE = 1.42
MAIN_ENTRY_PID_HANDOFF_MAX_CORRECTION = 0.42

# BLUE connector -> main-path turn: physical sensor/frame gates.
# These replace the time-only start for this one junction.
BLUE_MAIN_DEEP_NODE_RAW_MAX = 0.16
BLUE_MAIN_DEEP_NODE_ACTIVE_MIN = 4.55
BLUE_MAIN_DEEP_NODE_REQUIRED_FRAMES = 5

# The incoming connector line has become the outgoing main-path line when:
#   left          = black
#   middle/right/right_corner = white floor
# left_corner may still be partially grey at high speed, so do not require it
# to be fully white or fully black.
BLUE_MAIN_LOCK_LEFT_CORNER_MAX = 0.55
BLUE_MAIN_LOCK_LEFT_MAX = 0.24
BLUE_MAIN_LOCK_MIDDLE_MIN = 0.50
BLUE_MAIN_LOCK_RIGHT_MIN = 0.68
BLUE_MAIN_LOCK_REQUIRED_FRAMES = 2

# BLUE main-path node that must be ignored.
# This manoeuvre is armed ONLY by the normal route node_event.
BLUE_SKIP_DEEP_ACTIVE_SUM = 2.60
BLUE_SKIP_DEEP_REQUIRED_SENSORS = 4

BLUE_SKIP_EXIT_MIDDLE_RAW_MAX = 0.18
BLUE_SKIP_EXIT_INNER_RAW_MIN = 0.62
BLUE_SKIP_EXIT_OUTER_RAW_MIN = 0.68
BLUE_SKIP_EXIT_ACTIVE_SUM_MIN = 0.72
BLUE_SKIP_EXIT_ACTIVE_SUM_MAX = 1.55
BLUE_SKIP_EXIT_REQUIRED_FRAMES = 3

BLUE_SKIP_STRAIGHT_SPEED = 1.52

# FIRST WHITE-LINE INTERSECTION
# Black background + white line. The required branch is only about 30-40 degrees
# to the LEFT, so this must remain a smooth forward arc, never a 90-degree pivot.
WHITE_LEFT_ADVANCE_SECONDS = 0.32
WHITE_LEFT_ADVANCE_SPEED = 1.18

WHITE_LEFT_SEARCH_LEFT_SPEED = 1.02
WHITE_LEFT_SEARCH_RIGHT_SPEED = 1.68

WHITE_LEFT_ACQUIRE_LEFT_SPEED = 1.14
WHITE_LEFT_ACQUIRE_RIGHT_SPEED = 1.60

WHITE_LEFT_CENTER_LEFT_SPEED = 1.30
WHITE_LEFT_CENTER_RIGHT_SPEED = 1.48

WHITE_LEFT_LINE_THRESHOLD = 0.48
WHITE_LEFT_DARK_THRESHOLD = 0.38
WHITE_LEFT_MIN_TURN_SECONDS = 0.16
WHITE_LEFT_MAX_SECONDS = 1.70
WHITE_LEFT_CENTER_REQUIRED_FRAMES = 1

# BLUE DROP ZONE — adaptive marker for the current high-speed run.
#
# CSV 43 counted the physical pre-rectangle marker as NODE 4 because two
# earlier short nodes were skipped at high speed. Use route progress plus the
# all-white sensor shape instead of requiring an exact node number.
BLUE_DROP_MARKER_NODE = 6  # legacy fallback only
BLUE_DROP_MARKER_MIN_STATE_FRAME = 900

# CSV 44 last marker has four fully bright sensors; the outer left corner is
# only partially over the white dot. Therefore use 4-of-5 bright sensors.
BLUE_DROP_MARKER_BRIGHT_THRESHOLD = 0.64
BLUE_DROP_MARKER_REQUIRED_BRIGHT_SENSORS = 4
BLUE_DROP_MARKER_INNER_MIN = 0.70

# Deterministic relative-frame drop:
# CSV 44 marker frame 3891 + 38 frames = frame 3929.
BLUE_DROP_DELAY_FRAMES_AFTER_MARKER = 38
BLUE_DROP_FRAME_WINDOW_END = 70

# Retained for compatibility with older logging/tests. The exact-frame route
# no longer requires these values to permit the BLUE drop.
BLUE_DROP_LINE_MIDDLE_MIN = 0.70
BLUE_DROP_SIDE_MAX = 0.30
BLUE_DROP_STABLE_FRAMES = 1

# CSV 25/26: node 5 is the sharp turn immediately before drop-marker node 6.
# Let the robot move only a few extra milliseconds before the unchanged PID
# takes that corner.
BLUE_DROP_CORNER_NODE = 5
BLUE_DROP_CORNER_ADVANCE_SECONDS = 0.18

# CSV 24: the correct left branch was already acquired after about 1.15 s.
# Release the special white-node manoeuvre at that point so the normal surface
# transition logic can switch to BLACK-line PID.
RETURN_WHITE_LEFT_RELEASE_SECONDS = 1.15

# ---------------------------------------------------------------------------
# BLUE DROP AREA EXIT: one required RIGHT, then normal PID.
# ---------------------------------------------------------------------------
# CSV 48 and CSV 49 show the same reliable turn-complete pattern:
# four bright sensors and the right-corner sensor back on dark background.
BLUE_RETURN_RIGHT_COVER_FRAMES = 3
BLUE_RETURN_RIGHT_COVER_SPEED = 1.48

BLUE_RETURN_RIGHT_ARC_LEFT_SPEED = 1.92
BLUE_RETURN_RIGHT_ARC_RIGHT_SPEED = 0.24
BLUE_RETURN_RIGHT_SOFT_LEFT_SPEED = 1.62
BLUE_RETURN_RIGHT_SOFT_RIGHT_SPEED = 0.72

BLUE_RETURN_RIGHT_EXIT_BRIGHT_MIN = 0.62
BLUE_RETURN_RIGHT_EXIT_RIGHT_CORNER_MAX = 0.30
BLUE_RETURN_RIGHT_EXIT_REQUIRED_FRAMES = 2

# Frame safety: never continue an open-loop turn indefinitely.
BLUE_RETURN_RIGHT_MAX_TURN_FRAMES = 22

# After the node edge clears, follow the actual continuous path using a capped
# PID ramp. This is line-following, not another route turn.
BLUE_RETURN_RIGHT_HANDOFF_FRAMES = 18
BLUE_RETURN_RIGHT_HANDOFF_BASE_SPEED = 1.42
BLUE_RETURN_RIGHT_HANDOFF_GAIN = 0.72
BLUE_RETURN_RIGHT_HANDOFF_MAX_CORRECTION = 0.38

# ---------------------------------------------------------------------------
# THIRD WHITE JUNCTION -> LEFT WHITE BRANCH, THEN PID
# ---------------------------------------------------------------------------
BLUE_RETURN_LEFT_COVER_FRAMES = 6
BLUE_RETURN_LEFT_COVER_SPEED = 1.16

# Reuse the smooth successful low-speed LEFT arc instead of the previous hard
# open-loop turn. Both wheels always remain forward.
BLUE_RETURN_LEFT_ARC_LEFT_SPEED = 1.02
BLUE_RETURN_LEFT_ARC_RIGHT_SPEED = 1.68
BLUE_RETURN_LEFT_SOFT_LEFT_SPEED = 1.14
BLUE_RETURN_LEFT_SOFT_RIGHT_SPEED = 1.54

# Successful turn-complete signature:
# left_corner begins clearing onto dark floor while left/middle/right/
# right_corner still see the wide selected WHITE branch.
BLUE_RETURN_LEFT_EXIT_CORNER_MAX = 0.52
BLUE_RETURN_LEFT_EXIT_INNER_MIN = 0.62
BLUE_RETURN_LEFT_EXIT_REQUIRED_FRAMES = 2

# Prevent any endless open-loop junction turn.
BLUE_RETURN_LEFT_MAX_TURN_FRAMES = 26

# Capped PID follows the curved WHITE branch after branch selection.
BLUE_RETURN_LEFT_HANDOFF_FRAMES = 60
BLUE_RETURN_LEFT_HANDOFF_BASE_SPEED = 1.42
BLUE_RETURN_LEFT_HANDOFF_GAIN = 0.78
BLUE_RETURN_LEFT_HANDOFF_MAX_CORRECTION = 0.48

# ---------------------------------------------------------------------------
# RED DELIVERY ON BLACK BACKGROUND / WHITE LINE
# ---------------------------------------------------------------------------

# Mirror of the proven gentle LEFT white-line manoeuvre.
# Both motors always remain forward. The robot first covers the wide white
# node, then follows right_corner -> right -> middle.
WHITE_RIGHT_ADVANCE_SECONDS = 0.34
WHITE_RIGHT_ADVANCE_SPEED = 1.18

WHITE_RIGHT_SEARCH_LEFT_SPEED = 1.68
WHITE_RIGHT_SEARCH_RIGHT_SPEED = 1.02

WHITE_RIGHT_ACQUIRE_LEFT_SPEED = 1.60
WHITE_RIGHT_ACQUIRE_RIGHT_SPEED = 1.14

WHITE_RIGHT_CENTER_LEFT_SPEED = 1.48
WHITE_RIGHT_CENTER_RIGHT_SPEED = 1.30

WHITE_RIGHT_LINE_THRESHOLD = WHITE_LEFT_LINE_THRESHOLD
WHITE_RIGHT_DARK_THRESHOLD = WHITE_LEFT_DARK_THRESHOLD
WHITE_RIGHT_MIN_TURN_SECONDS = 0.16
WHITE_RIGHT_MAX_SECONDS = 1.80
WHITE_RIGHT_CENTER_REQUIRED_FRAMES = 1

# After selecting the RIGHT branch:
#   node 2 -> continue straight with PID;
#   node 3 -> last white node before the red rectangle.
RED_DROP_MARKER_NODE = 3
RED_DROP_DELAY_AFTER_NODE = 0.65
RED_DROP_LINE_MIDDLE_MIN = 0.65
RED_DROP_SIDE_MAX = 0.35
RED_DROP_STABLE_FRAMES = 1

# Non-critical small improvement requested for the RED connector-to-main node.
# The proven BLUE manoeuvre is untouched; RED alone moves 0.05 s farther over
# its black node before beginning the existing RIGHT acquisition.
RED_MAIN_EXTRA_ADVANCE_SECONDS = 0.05

# ---------------------------------------------------------------------------
# FINAL WHITE NODE -> RIGHT TOWARD END SQUARE
# ---------------------------------------------------------------------------
# CSV 28:
# - all five sensors cover the wide white node first;
# - during the correct right turn, the finish line crosses LEFT + MIDDLE;
# - the old logic kept turning and then lost the line.
# Final node needs only a short forward movement because the incoming path is
# already curved toward the required right branch.
FINISH_RIGHT_ADVANCE_SECONDS = 0.12
FINISH_RIGHT_ADVANCE_SPEED = 1.12

# Initial forward RIGHT arc while crossing the wide node.
FINISH_RIGHT_ARC_LEFT_SPEED = 1.55
FINISH_RIGHT_ARC_RIGHT_SPEED = 1.05

# Tight forward-only RIGHT search after the wide node has cleared.
# Both motors remain forward; there is no in-place reverse pivot.
FINISH_RIGHT_SEARCH_LEFT_SPEED = 1.38
FINISH_RIGHT_SEARCH_RIGHT_SPEED = 0.18

# Sensor-guided branch acquisition:
# right_corner -> right -> middle.
FINISH_RIGHT_CORNER_LEFT_SPEED = 1.30
FINISH_RIGHT_CORNER_RIGHT_SPEED = 0.32
FINISH_RIGHT_INNER_LEFT_SPEED = 1.20
FINISH_RIGHT_INNER_RIGHT_SPEED = 0.55
FINISH_RIGHT_CENTER_LEFT_SPEED = 1.08
FINISH_RIGHT_CENTER_RIGHT_SPEED = 0.92

FINISH_RIGHT_NODE_CLEAR_RAW_SUM = 0.95
FINISH_RIGHT_GAP_MAX_SENSOR = 0.32
FINISH_RIGHT_LINE_THRESHOLD = 0.48
FINISH_RIGHT_CENTER_REQUIRED_FRAMES = 1
FINISH_RIGHT_MAX_SEARCH_SECONDS = 4.00

# Straight/right-only handoff. It cannot undo the final right turn with a
# sudden left command.
FINISH_RIGHT_HANDOFF_SECONDS = 0.65
FINISH_RIGHT_HANDOFF_BASE_SPEED = 1.28
FINISH_RIGHT_HANDOFF_MAX_RIGHT_CORRECTION = 0.20
FINISH_RIGHT_HANDOFF_CENTER_FRAMES = 1

# ---------------------------------------------------------------------------
# POST-BLUE-DROP RETURN ROUTE
# ---------------------------------------------------------------------------

# Black-line LEFT turn at the second black node when returning to the circle.
# The robot first advances over the dot and then takes a forward left arc while
# following the outgoing connector line.
RETURN_BLACK_LEFT_ADVANCE_SECONDS = 0.28
RETURN_BLACK_LEFT_ADVANCE_SPEED = 1.18
RETURN_BLACK_LEFT_SEARCH_LEFT_SPEED = 0.90
RETURN_BLACK_LEFT_SEARCH_RIGHT_SPEED = 1.82
RETURN_BLACK_LEFT_ACQUIRE_LEFT_SPEED = 1.08
RETURN_BLACK_LEFT_ACQUIRE_RIGHT_SPEED = 1.65
RETURN_BLACK_LEFT_CENTER_LEFT_SPEED = 1.28
RETURN_BLACK_LEFT_CENTER_RIGHT_SPEED = 1.50
RETURN_BLACK_LEFT_MIN_TURN_SECONDS = 0.18
RETURN_BLACK_LEFT_MAX_SECONDS = 1.85
RETURN_BLACK_LEFT_CENTER_REQUIRED_FRAMES = 1

# RED circle entry is the exact mirror of the successful BLUE CSV-13 entry.
# BLUE: left fast, right slow.
# RED:  left slow, right fast.
RED_ENTRY_ADVANCE_SECONDS = BLUE_ENTRY_ADVANCE_SECONDS
RED_ENTRY_ADVANCE_SPEED = BLUE_ENTRY_ADVANCE_SPEED
RED_ENTRY_FIXED_LEFT_TURN_SECONDS = BLUE_ENTRY_FIXED_RIGHT_TURN_SECONDS
RED_ENTRY_LEFT_SPEED = BLUE_ENTRY_INNER_SPEED
RED_ENTRY_RIGHT_SPEED = BLUE_ENTRY_OUTER_SPEED
RED_ENTRY_HANDOFF_SECONDS = BLUE_ENTRY_HANDOFF_SECONDS
RED_ENTRY_HANDOFF_BASE_SPEED = BLUE_ENTRY_HANDOFF_BASE_SPEED
RED_ENTRY_HANDOFF_MAX_CORRECTION = BLUE_ENTRY_HANDOFF_MAX_CORRECTION

# RED circle exit: mirror of the working BLUE circle-exit manoeuvre.
RED_EXIT_ADVANCE_SECONDS = BLUE_EXIT_ADVANCE_SECONDS
RED_EXIT_ADVANCE_SPEED = BLUE_EXIT_ADVANCE_SPEED
RED_EXIT_CURVE_MIN_SECONDS = BLUE_EXIT_CURVE_MIN_SECONDS
RED_EXIT_CURVE_MAX_SECONDS = BLUE_EXIT_CURVE_MAX_SECONDS
RED_EXIT_SEARCH_SECONDS = BLUE_EXIT_SEARCH_SECONDS
RED_EXIT_LEFT_SPEED = BLUE_EXIT_INNER_SPEED
RED_EXIT_RIGHT_SPEED = BLUE_EXIT_OUTER_SPEED
RED_EXIT_SEARCH_SPEED = BLUE_EXIT_SEARCH_SPEED
RED_EXIT_CENTER_REQUIRED_FRAMES = BLUE_EXIT_CENTER_REQUIRED_FRAMES
RED_EXIT_HANDOFF_SECONDS = BLUE_EXIT_HANDOFF_SECONDS
RED_EXIT_HANDOFF_BASE_SPEED = BLUE_EXIT_HANDOFF_BASE_SPEED
RED_EXIT_HANDOFF_MAX_CORRECTION = BLUE_EXIT_HANDOFF_MAX_CORRECTION


# -----------------------------------------------------------------------------
# Route states
# -----------------------------------------------------------------------------
START_TO_CIRCLE = "START_TO_CIRCLE"
BLUE_CONNECTOR = "BLUE_CONNECTOR"
BLUE_CIRCLE = "BLUE_CIRCLE"
BLUE_EXIT_TO_MAIN = "BLUE_EXIT_TO_MAIN"
BLUE_TO_WHITE = "BLUE_TO_WHITE"
BLUE_WHITE_TO_DROP = "BLUE_WHITE_TO_DROP"
BLUE_AFTER_DROP = "BLUE_AFTER_DROP"
BLUE_RETURN_WHITE = "BLUE_RETURN_WHITE"
BLUE_TO_BLACK = "BLUE_TO_BLACK"
RETURN_TO_CIRCLE = "RETURN_TO_CIRCLE"
RED_CONNECTOR = "RED_CONNECTOR"
RED_CIRCLE = "RED_CIRCLE"
RED_EXIT_TO_MAIN = "RED_EXIT_TO_MAIN"
RED_TO_WHITE = "RED_TO_WHITE"
RED_WHITE_TO_DROP = "RED_WHITE_TO_DROP"
RED_WAIT_AT_WHITE_JUNCTION = "RED_WAIT_AT_WHITE_JUNCTION"
RED_AFTER_DROP = "RED_AFTER_DROP"
FINISH_ROUTE = "FINISH_ROUTE"
FINISHED = "FINISHED"


_route_state = START_TO_CIRCLE
_line_mode = "black"       # "black" = black line on white background
                            # "white" = white line on black background
_state_enter_time = _turbo_now()
_state_node_count = 0

_previous_error = 0.0
_integral = 0.0
_last_pid_time = _turbo_now()
_last_valid_error = 0.0
_lost_frames = 0

_node_frames = 0
_node_latched = False
_node_cooldown_until = 0.0
_line_switch_counter = 0
_finish_counter = 0

# Start-marker rejection gate. The first wide black patch is NOT route node 1.
_start_node_armed = False
_start_clear_frames = 0
_last_strong_count = 0

_manoeuvre_kind = None
_manoeuvre_start = 0.0
_manoeuvre_duration = 0.0
_last_manoeuvre_phase = ""
_last_manoeuvre_elapsed = 0.0
_manoeuvre_branch_seen = False
_manoeuvre_node_cleared = False
_manoeuvre_center_frames = 0

_circle_start_time = 0.0
_blue_picked = False
_red_picked = False
_blue_delivered = False
_red_delivered = False

_last_carrying = None
_pick_cooldown_until = 0.0
_drop_cooldown_until = 0.0
_pick_colour_frames = 0
_drop_colour_frames = 0

_frame_number = 0
_start_time = _turbo_now()
_event_note = "program_start"
_pick_requested = False
_drop_requested = False
_last_detected_colour = None
_last_position = 0.0
_last_error = 0.0
_last_pid = 0.0
_last_node_now = False
_last_node_event = False

# True while the sensors have started touching the second black spot but have
# not yet reached its centre.  control_loop() then drives slowly straight so
# PID cannot pull the robot past the correct turning point.
_circle_entry_approach = False
_circle_entry_approach_started = 0.0

# After the fixed successful turn, use a short gentle PID handoff. This avoids
# the large derivative kick that CSV 13 showed immediately after frame 1817.
_blue_entry_handoff_until = 0.0

# Gentle PID handoff after the THIRD-node right turn onto the connector.
_blue_exit_handoff_until = 0.0
_blue_exit_handoff_center_frames = 0

_main_entry_handoff_until = 0.0
_main_entry_handoff_center_frames = 0

# Short capped-PID continuation after the required post-BLUE-drop RIGHT turn.
_blue_return_right_handoff_frames = 0

# Capped WHITE-line PID after selecting LEFT at the third return junction.
_blue_return_left_handoff_frames = 0

# Real ignored node on BLUE main path.
_blue_main_skip_completed = False

# Relative frame counters for speed-independent CSV comparison.
_state_frame_count = 0
_manoeuvre_frame_count = 0

# Armed at the late all-white node immediately before the blue rectangle.
_blue_drop_marker_time = 0.0
_blue_drop_marker_state_frame = -1
_blue_drop_armed = False
_blue_drop_corner_adjusted = False

# RED-route line-capture handoffs.
_red_entry_handoff_until = 0.0
_red_exit_handoff_until = 0.0
_red_exit_handoff_center_frames = 0
_red_main_handoff_until = 0.0
_red_main_handoff_center_frames = 0

# Armed at RED white node 3, immediately before the red rectangle.
_red_drop_marker_time = 0.0
_red_drop_armed = False

_finish_right_handoff_until = 0.0
_finish_right_handoff_center_frames = 0


# -----------------------------------------------------------------------------
# CSV logger
# -----------------------------------------------------------------------------
_CSV_FIELDS = [
    "frame", "elapsed_s", "state", "line_mode", "event",
    "left_corner", "left", "middle", "right", "right_corner",
    "raw_mean", "active_sum", "line_position", "error", "pid",
    "left_motor", "right_motor", "node_now", "node_event",
    "strong_count", "start_node_armed", "start_clear_frames",
    "state_node_count", "state_frame",
    "blue_drop_armed", "blue_drop_frames_from_marker",
    "blue_drop_due_now", "expected_next_action",
    "manoeuvre", "manoeuvre_phase",
    "manoeuvre_frame", "manoeuvre_elapsed",
    "sensor_mask", "base_left_motor", "base_right_motor",
    "turbo_factor", "virtual_elapsed_s", "proximity",
    "color_r", "color_g", "color_b", "detected_color",
    "carrying_inferred", "pick_requested", "drop_requested",
]

try:
    _csv_file = open("task2b_debug.csv", "w", newline="", encoding="utf-8")
    _csv_writer = csv.DictWriter(_csv_file, fieldnames=_CSV_FIELDS)
    _csv_writer.writeheader()
    _csv_file.flush()
except OSError as exc:
    print(f"[CSV] Could not create task2b_debug.csv: {exc}")
    _csv_file = None
    _csv_writer = None


def _add_event(message):
    """Add a short event message to the current CSV row and terminal."""
    global _event_note
    if not message:
        return
    if _event_note:
        _event_note += " | " + message
    else:
        _event_note = message
    print(f"[ROUTE] {message}")


def _set_state(new_state, reason=""):
    """Change route state and reset counters that belong to the old state."""
    global _route_state, _state_enter_time, _state_node_count
    global _line_switch_counter, _drop_colour_frames
    global _state_frame_count

    old_state = _route_state
    _route_state = new_state
    _state_enter_time = _turbo_now()
    _state_node_count = 0
    _state_frame_count = 0
    _line_switch_counter = 0
    _drop_colour_frames = 0
    _add_event(f"STATE {old_state} -> {new_state}: {reason}")


def _start_manoeuvre(kind, duration=None):
    """Start an open-loop junction manoeuvre.

    ``right_90`` is reserved for the first circle-connector junction.  It moves
    slightly forward over the highlighted dot, then takes a forward right arc
    and finishes when the outgoing connector line is centred under the sensors.
    """
    global _manoeuvre_kind, _manoeuvre_start, _manoeuvre_duration
    global _node_cooldown_until
    global _last_manoeuvre_phase, _last_manoeuvre_elapsed
    global _manoeuvre_branch_seen
    global _manoeuvre_node_cleared, _manoeuvre_center_frames
    global _manoeuvre_frame_count

    default_duration = {
        "left": 0.62,
        "right": 0.62,
        "left_soft": 0.48,
        "right_soft": 0.48,
        "straight": 0.46,
        "right_90": (
            FIRST_TURN_ADVANCE_SECONDS
            + FIRST_TURN_CURVE_MAX_SECONDS
            + FIRST_TURN_SEARCH_SECONDS
        ),
        "blue_red_side_acquire": (
            BLUE_ENTRY_ADVANCE_SECONDS
            + BLUE_ENTRY_FIXED_RIGHT_TURN_SECONDS
            + BLUE_ENTRY_HANDOFF_SECONDS
        ),
        "blue_exit_right_acquire": (
            BLUE_EXIT_ADVANCE_SECONDS
            + BLUE_EXIT_CURVE_MAX_SECONDS
            + BLUE_EXIT_SEARCH_SECONDS
        ),
        "blue_main_right_follow": (
            MAIN_ENTRY_ADVANCE_SECONDS + MAIN_ENTRY_MAX_SECONDS
        ),
        "blue_main_skip_straight": 2.20,  # debug fallback; route does not start it
        "blue_return_right_lock": 4.00,
        "blue_return_left_black_lock": 6.00,
        "white_left_gentle": (
            WHITE_LEFT_ADVANCE_SECONDS + WHITE_LEFT_MAX_SECONDS
        ),
        "white_right_gentle": (
            WHITE_RIGHT_ADVANCE_SECONDS + WHITE_RIGHT_MAX_SECONDS
        ),
        "finish_right_line_lock": (
            FINISH_RIGHT_ADVANCE_SECONDS
            + FINISH_RIGHT_MAX_SEARCH_SECONDS
        ),
        "return_black_left_connector": (
            RETURN_BLACK_LEFT_ADVANCE_SECONDS + RETURN_BLACK_LEFT_MAX_SECONDS
        ),
        "red_circle_left_replay": (
            RED_ENTRY_ADVANCE_SECONDS
            + RED_ENTRY_FIXED_LEFT_TURN_SECONDS
            + RED_ENTRY_HANDOFF_SECONDS
        ),
        "red_exit_left_acquire": (
            RED_EXIT_ADVANCE_SECONDS
            + RED_EXIT_CURVE_MAX_SECONDS
            + RED_EXIT_SEARCH_SECONDS
        ),
        "red_main_right_follow": (
            MAIN_ENTRY_ADVANCE_SECONDS
            + RED_MAIN_EXTRA_ADVANCE_SECONDS
            + MAIN_ENTRY_MAX_SECONDS
        ),
    }

    _manoeuvre_kind = kind
    _manoeuvre_start = _turbo_now()
    _manoeuvre_duration = (
        float(duration) if duration is not None
        else default_duration.get(kind, 0.50)
    )
    _last_manoeuvre_phase = "started"
    _last_manoeuvre_elapsed = 0.0
    _manoeuvre_branch_seen = False
    _manoeuvre_node_cleared = False
    _manoeuvre_center_frames = 0
    _manoeuvre_frame_count = 0
    _node_cooldown_until = _manoeuvre_start + _manoeuvre_duration + 0.55

    if kind == "right_90":
        _add_event(
            "MANOEUVRE right_90: cover dot for full timed distance, "
            "then follow a right arc into connector"
        )
    elif kind == "blue_red_side_acquire":
        _add_event(
            "MANOEUVRE blue_red_side_acquire: take RIGHT-side circle branch "
            "toward BLUE box and lock its line"
        )
    elif kind == "blue_exit_right_acquire":
        _add_event(
            "MANOEUVRE blue_exit_right_acquire: THIRD black node; "
            "turn RIGHT onto connector back to main path"
        )
    elif kind == "blue_main_right_follow":
        _add_event(
            "MANOEUVRE blue_main_right_follow: FOURTH black node; "
            "wait for deep-node frames, then turn RIGHT and frame-lock "
            "the black main-path line"
        )
    elif kind == "blue_main_skip_straight":
        _add_event(
            "MANOEUVRE blue_main_skip_straight: debug-only fallback; "
            "normal route does not start this manoeuvre"
        )
    elif kind == "blue_return_right_lock":
        _add_event(
            "MANOEUVRE blue_return_right_lock: cover drop-area node, "
            "take required RIGHT, then release immediately to line PID"
        )
    elif kind == "blue_return_left_black_lock":
        _add_event(
            "MANOEUVRE blue_return_left_black_lock: cover third white "
            "junction, select LEFT white branch, then follow it with PID"
        )
    elif kind == "white_left_gentle":
        _add_event(
            "MANOEUVRE white_left_gentle: move into white node, "
            "then follow the 30-40 degree LEFT branch"
        )
    elif kind == "white_right_gentle":
        _add_event(
            "MANOEUVRE white_right_gentle: cover white node, "
            "then follow the RIGHT branch with sensors"
        )
    elif kind == "finish_right_line_lock":
        _add_event(
            "MANOEUVRE finish_right_line_lock: cover final white node, "
            "turn RIGHT, and lock finish line on LEFT+MIDDLE sensors"
        )
    elif kind == "return_black_left_connector":
        _add_event(
            "MANOEUVRE return_black_left_connector: second black node; "
            "advance, turn LEFT, and lock the connector line"
        )
    elif kind == "red_circle_left_replay":
        _add_event(
            "MANOEUVRE red_circle_left_replay: circle entry; "
            "take LEFT branch toward RED box"
        )
    elif kind == "red_exit_left_acquire":
        _add_event(
            "MANOEUVRE red_exit_left_acquire: red lap complete; "
            "turn LEFT onto connector"
        )
    elif kind == "red_main_right_follow":
        _add_event(
            "MANOEUVRE red_main_right_follow: connector complete; "
            "turn RIGHT and lock black main-path line"
        )
    else:
        _add_event(f"MANOEUVRE {kind} for {_manoeuvre_duration:.2f}s")


def _finish_manoeuvre(message):
    """Clear the active manoeuvre and record why it ended."""
    global _manoeuvre_kind, _last_manoeuvre_phase

    _add_event(message)
    _manoeuvre_kind = None
    _last_manoeuvre_phase = "complete"


def _manoeuvre_command(now, active, raw=None):
    """Return motor speeds while a junction manoeuvre is active."""
    global _manoeuvre_kind
    global _last_manoeuvre_phase, _last_manoeuvre_elapsed
    global _manoeuvre_branch_seen
    global _manoeuvre_node_cleared, _manoeuvre_center_frames
    global _line_mode
    global _main_entry_handoff_until, _main_entry_handoff_center_frames
    global _red_entry_handoff_until
    global _red_exit_handoff_until, _red_exit_handoff_center_frames
    global _red_main_handoff_until, _red_main_handoff_center_frames
    global _manoeuvre_frame_count

    if _manoeuvre_kind is None:
        _last_manoeuvre_phase = ""
        _last_manoeuvre_elapsed = 0.0
        _manoeuvre_frame_count = 0
        return None

    _manoeuvre_frame_count += 1
    elapsed = now - _manoeuvre_start
    _last_manoeuvre_elapsed = elapsed

    # ------------------------------------------------------------------
    # First 90-degree junction: advance first, then curve onto connector.
    # ------------------------------------------------------------------
    if _manoeuvre_kind == "right_90":
        # The axle was previously rotating while still slightly before the
        # connector mouth. Move a short distance beyond the centre of the dot
        # before steering. This is forward movement, not a one-second stop.
        if elapsed < FIRST_TURN_ADVANCE_SECONDS:
            _last_manoeuvre_phase = "advance_over_dot"
            return FIRST_TURN_ADVANCE_SPEED, FIRST_TURN_ADVANCE_SPEED

        turn_elapsed = elapsed - FIRST_TURN_ADVANCE_SECONDS
        strong_count = sum(
            value >= NODE_ACTIVE_THRESHOLD for value in active
        )
        active_total = sum(active)

        # During a correct right turn the branch normally appears first under
        # one of the right-hand sensors. Remember that observation so the old
        # main line cannot end the turn prematurely.
        if active[3] >= 0.48 or active[4] >= 0.48:
            _manoeuvre_branch_seen = True

        connector_centred = (
            _manoeuvre_branch_seen
            and active[2] >= 0.52
            and strong_count <= 3
            and 0.28 <= active_total <= 2.55
        )

        if (
            turn_elapsed >= FIRST_TURN_CURVE_MIN_SECONDS
            and connector_centred
        ):
            _finish_manoeuvre(
                "MANOEUVRE right_90 complete: connector centred after right arc"
            )
            return None

        # Main steering phase: both wheels remain forward, but the left wheel
        # is faster. This produces the requested smooth right-hand arc while
        # the robot continues moving onto the connecting line.
        if turn_elapsed < FIRST_TURN_CURVE_MAX_SECONDS:
            _last_manoeuvre_phase = "curve_right_into_connector"
            return FIRST_TURN_OUTER_SPEED, FIRST_TURN_INNER_SPEED

        # If the connector has not centred yet, rotate slowly for only a short
        # search window. This is a fallback, not the main 90-degree turn.
        search_elapsed = turn_elapsed - FIRST_TURN_CURVE_MAX_SECONDS
        if search_elapsed < FIRST_TURN_SEARCH_SECONDS:
            _last_manoeuvre_phase = "fine_search_for_connector"
            return FIRST_TURN_SEARCH_SPEED, -FIRST_TURN_SEARCH_SPEED

        _finish_manoeuvre(
            "MANOEUVRE right_90 complete: connector search time reached"
        )
        return None

    # ------------------------------------------------------------------
    # BLUE circle entry: turn toward the RED side of the circle.
    # Keep steering until the wide black node is cleared and the outgoing
    # circle line is actually under the middle sensor.
    # ------------------------------------------------------------------
    if _manoeuvre_kind == "blue_red_side_acquire":
        global _blue_entry_handoff_until
        global _previous_error, _integral, _last_pid_time

        # CSV 13 first moved slightly over the centre of the second black spot.
        if elapsed < BLUE_ENTRY_ADVANCE_SECONDS:
            _last_manoeuvre_phase = "blue_enter_circle_node"
            return BLUE_ENTRY_ADVANCE_SPEED, BLUE_ENTRY_ADVANCE_SPEED

        turn_elapsed = elapsed - BLUE_ENTRY_ADVANCE_SECONDS

        # Reproduce the exact successful turn instead of waiting for a fragile
        # one-frame sensor signature.  Because the node trigger is now repeatable,
        # the same motor command and duration place the robot on the blue branch.
        if turn_elapsed < BLUE_ENTRY_FIXED_RIGHT_TURN_SECONDS:
            _last_manoeuvre_phase = "blue_replay_csv13_right_turn"
            return BLUE_ENTRY_OUTER_SPEED, BLUE_ENTRY_INNER_SPEED

        # The desired branch has been reached. Do not perform another turn.
        # Reset PID history and use a short, gentle handoff so the robot simply
        # continues following the circle line without a sharp reverse correction.
        _blue_entry_handoff_until = now + BLUE_ENTRY_HANDOFF_SECONDS
        _previous_error = 0.0
        _integral = 0.0
        _last_pid_time = now
        _finish_manoeuvre(
            "MANOEUVRE blue_red_side_acquire complete: "
            "CSV13 right turn replayed; continue circle line"
        )
        return None

    # ------------------------------------------------------------------
    # BLUE circle exit: THIRD black node -> RIGHT onto the connector.
    # ------------------------------------------------------------------
    if _manoeuvre_kind == "blue_exit_right_acquire":
        global _blue_exit_handoff_until, _blue_exit_handoff_center_frames

        # Move slightly over the centre of the highlighted dot so the axle is
        # level with the mouth of the connector before steering.
        if elapsed < BLUE_EXIT_ADVANCE_SECONDS:
            _last_manoeuvre_phase = "blue_exit_advance_over_third_dot"
            return BLUE_EXIT_ADVANCE_SPEED, BLUE_EXIT_ADVANCE_SPEED

        turn_elapsed = elapsed - BLUE_EXIT_ADVANCE_SECONDS
        strong_count = sum(
            value >= NODE_ACTIVE_THRESHOLD for value in active
        )
        active_total = sum(active)

        # The wide node initially covers four/five sensors. It is cleared once
        # no more than three sensors strongly see black.
        if strong_count <= 3 and active_total <= 3.30:
            _manoeuvre_node_cleared = True

        # CSV 16 shows the connector entering under the LEFT sensors and then
        # crossing the MIDDLE sensor during this right arc:
        #   frame 2528: left_corner/left strong, middle ~= 0.47 active
        #   frame 2529: left_corner/left strong, middle ~= 0.37 active
        # The previous code incorrectly waited for a RIGHT-sensor signature,
        # ignored this valid line crossing, and kept turning until the line was
        # completely lost.
        if (
            _manoeuvre_node_cleared
            and (active[0] >= 0.55 or active[1] >= 0.55)
            and active[3] <= 0.18
            and active[4] <= 0.18
        ):
            _manoeuvre_branch_seen = True

        connector_centred = (
            _manoeuvre_node_cleared
            and _manoeuvre_branch_seen
            and active[2] >= 0.34
            and active[0] >= 0.55
            and active[1] >= 0.40
            and active[3] <= 0.18
            and active[4] <= 0.18
            and strong_count <= 3
            and 1.55 <= active_total <= 2.80
        )

        if (
            turn_elapsed >= BLUE_EXIT_CURVE_MIN_SECONDS
            and connector_centred
        ):
            _manoeuvre_center_frames += 1
        else:
            _manoeuvre_center_frames = 0

        if _manoeuvre_center_frames >= BLUE_EXIT_CENTER_REQUIRED_FRAMES:
            _blue_exit_handoff_until = now + BLUE_EXIT_HANDOFF_SECONDS
            _blue_exit_handoff_center_frames = 0
            _previous_error = 0.0
            _integral = 0.0
            _last_pid_time = now
            _finish_manoeuvre(
                "MANOEUVRE blue_exit_right_acquire complete: "
                "CSV16 connector crossed middle sensor; continue to main path"
            )
            return None

        # Main right arc: left wheel fast, right wheel slow.
        if turn_elapsed < BLUE_EXIT_CURVE_MAX_SECONDS:
            _last_manoeuvre_phase = "blue_exit_curve_right_to_connector"
            return BLUE_EXIT_OUTER_SPEED, BLUE_EXIT_INNER_SPEED

        # Short right-pivot fallback only when the connector has not centred yet.
        search_elapsed = turn_elapsed - BLUE_EXIT_CURVE_MAX_SECONDS
        if search_elapsed < BLUE_EXIT_SEARCH_SECONDS:
            _last_manoeuvre_phase = "blue_exit_search_connector"
            return BLUE_EXIT_SEARCH_SPEED, -BLUE_EXIT_SEARCH_SPEED

        # Safety finish: hand control to a gentle line-following ramp rather than
        # continuing to rotate indefinitely.
        _blue_exit_handoff_until = now + BLUE_EXIT_HANDOFF_SECONDS
        _blue_exit_handoff_center_frames = 0
        _previous_error = 0.0
        _integral = 0.0
        _last_pid_time = now
        _finish_manoeuvre(
            "MANOEUVRE blue_exit_right_acquire ended by safety timeout; "
            "continue connector search with PID"
        )
        return None

    # ------------------------------------------------------------------
    # FOURTH black node: circle connector -> RIGHT onto the main path.
    #
    # Follow the real WHITE-line progression instead of using a fixed turn:
    # right_corner -> right -> middle. No left command is allowed.
    # ------------------------------------------------------------------
    if _manoeuvre_kind == "blue_main_right_follow":
        if raw is None:
            raw = [
                1.0 - (max(0.0, value) ** 0.5)
                for value in active
            ]

        active_sum_now = sum(active)

        # --------------------------------------------------------------
        # PHASE 1 — move physically deep over the fourth black node.
        #
        # Do not use elapsed time to decide when to turn. In slowed
        # CoppeliaSim playback, many control packets may arrive before the
        # robot has travelled far enough. A turn is allowed only after all
        # five sensors are deeply over the black node for five consecutive
        # frames.
        # --------------------------------------------------------------
        if not _manoeuvre_node_cleared:
            deep_over_node = (
                max(raw) <= BLUE_MAIN_DEEP_NODE_RAW_MAX
                and active_sum_now >= BLUE_MAIN_DEEP_NODE_ACTIVE_MIN
            )

            if deep_over_node:
                _manoeuvre_center_frames += 1
            else:
                _manoeuvre_center_frames = 0

            if (
                _manoeuvre_center_frames
                < BLUE_MAIN_DEEP_NODE_REQUIRED_FRAMES
            ):
                _last_manoeuvre_phase = (
                    "main_entry_sensor_depth_over_fourth_dot"
                )
                return (
                    MAIN_ENTRY_ADVANCE_SPEED,
                    MAIN_ENTRY_ADVANCE_SPEED,
                )

            # The sensor bar is now physically deep over the node.
            # Reset the counter before using it for line-lock frames.
            _manoeuvre_node_cleared = True
            _manoeuvre_center_frames = 0
            _last_manoeuvre_phase = (
                "main_entry_deep_node_confirmed_start_right"
            )

        # --------------------------------------------------------------
        # PHASE 2 — sensor-guided RIGHT turn onto the black main path.
        # --------------------------------------------------------------
        white_right_corner = (
            raw[4] >= BLUE_MAIN_LOCK_RIGHT_MIN
        )
        white_right = raw[3] >= BLUE_MAIN_LOCK_RIGHT_MIN
        white_middle = raw[2] >= BLUE_MAIN_LOCK_MIDDLE_MIN

        black_left = raw[1] <= BLUE_MAIN_LOCK_LEFT_MAX
        left_corner_safe = (
            raw[0] <= BLUE_MAIN_LOCK_LEFT_CORNER_MAX
        )

        if white_right_corner:
            _manoeuvre_branch_seen = True

        # CSV 35 frames 1503–1504 already contain the correct outgoing line:
        # left is black, while middle/right/right_corner are white. The old
        # gate rejected it only because left_corner was grey (~0.37), then the
        # robot kept turning and missed the line.
        black_main_line_acquired = (
            _manoeuvre_node_cleared
            and _manoeuvre_branch_seen
            and left_corner_safe
            and black_left
            and white_middle
            and white_right
            and white_right_corner
        )

        if black_main_line_acquired:
            _manoeuvre_center_frames += 1
        else:
            _manoeuvre_center_frames = 0

        if (
            _manoeuvre_center_frames
            >= BLUE_MAIN_LOCK_REQUIRED_FRAMES
        ):
            _line_mode = "black"
            _set_state(
                BLUE_TO_WHITE,
                "fourth black node completed; "
                "sensor-frame main-path lock acquired",
            )
            _main_entry_handoff_until = (
                now + MAIN_ENTRY_PID_HANDOFF_SECONDS
            )
            _main_entry_handoff_center_frames = 0

            position_now, _ = _line_position(active)
            error_now = (
                -position_now if position_now is not None else 0.0
            )
            _previous_error = error_now
            _integral = 0.0
            _last_pid_time = now

            _finish_manoeuvre(
                "MANOEUVRE blue_main_right_follow complete: "
                "main path locked for two sensor frames"
            )
            return None

        # The outgoing branch must enter from right_corner -> right -> middle.
        if not _manoeuvre_branch_seen:
            _last_manoeuvre_phase = (
                "main_entry_hard_right_search_after_deep_node"
            )
            return (
                MAIN_ENTRY_HARD_RIGHT_SPEED,
                MAIN_ENTRY_HARD_INNER_SPEED,
            )

        if white_right_corner and not white_right:
            _last_manoeuvre_phase = (
                "main_entry_follow_right_corner"
            )
            return (
                MAIN_ENTRY_HARD_RIGHT_SPEED,
                MAIN_ENTRY_HARD_INNER_SPEED,
            )

        if white_right and not white_middle:
            _last_manoeuvre_phase = (
                "main_entry_follow_right_sensor"
            )
            return (
                MAIN_ENTRY_MEDIUM_RIGHT_SPEED,
                MAIN_ENTRY_MEDIUM_INNER_SPEED,
            )

        # Once middle reaches the white floor, soften the turn and wait for
        # two complete line-lock frames. Never continue the hard arc through
        # an already acquired line.
        if white_middle:
            _last_manoeuvre_phase = (
                "main_entry_soft_right_until_frame_lock"
            )
            return 1.55, 1.08

        # No time-based timeout here. Keep a controlled forward right arc until
        # the physical sensor sequence appears.
        _last_manoeuvre_phase = (
            "main_entry_sensor_guided_safe_right_search"
        )
        return 1.55, 0.62

    # ------------------------------------------------------------------
    # BLUE MAIN-PATH NODE: IGNORE AND CONTINUE STRAIGHT.
    #
    # IMPORTANT: this block cannot start from a curve/widening pattern.
    # It starts only after _detect_node() produces NODE 1 in BLUE_TO_WHITE.
    # ------------------------------------------------------------------
    if _manoeuvre_kind == "blue_main_skip_straight":
        global _blue_main_skip_completed

        if raw is None:
            raw = [
                1.0 - (max(0.0, value) ** 0.5)
                for value in active
            ]

        active_sum_now = sum(active)
        strong_count_now = sum(
            value >= NODE_ACTIVE_THRESHOLD for value in active
        )

        deep_black_node = (
            active_sum_now >= BLUE_SKIP_DEEP_ACTIVE_SUM
            and strong_count_now >= BLUE_SKIP_DEEP_REQUIRED_SENSORS
        )
        if deep_black_node:
            _manoeuvre_node_cleared = True

        # Release only after the deep node has been seen and the normal narrow
        # black line returns under the centre sensor.
        narrow_centred_black_line = (
            _manoeuvre_node_cleared
            and raw[2] <= BLUE_SKIP_EXIT_MIDDLE_RAW_MAX
            and raw[1] >= BLUE_SKIP_EXIT_INNER_RAW_MIN
            and raw[3] >= BLUE_SKIP_EXIT_INNER_RAW_MIN
            and raw[0] >= BLUE_SKIP_EXIT_OUTER_RAW_MIN
            and raw[4] >= BLUE_SKIP_EXIT_OUTER_RAW_MIN
            and BLUE_SKIP_EXIT_ACTIVE_SUM_MIN
                <= active_sum_now
                <= BLUE_SKIP_EXIT_ACTIVE_SUM_MAX
        )

        if narrow_centred_black_line:
            _manoeuvre_center_frames += 1
        else:
            _manoeuvre_center_frames = 0

        if (
            _manoeuvre_center_frames
            >= BLUE_SKIP_EXIT_REQUIRED_FRAMES
        ):
            _blue_main_skip_completed = True

            position_now, _ = _line_position(active)
            error_now = (
                -position_now if position_now is not None else 0.0
            )
            _previous_error = error_now
            _integral = 0.0
            _last_pid_time = now

            _finish_manoeuvre(
                "MANOEUVRE blue_main_skip_straight complete: "
                "real node crossed; centred black line reacquired"
            )
            return None

        if not _manoeuvre_node_cleared:
            _last_manoeuvre_phase = (
                "blue_skip_real_node_deep_black"
            )
        else:
            _last_manoeuvre_phase = (
                "blue_skip_wait_narrow_centre_line"
            )

        # Equal wheel commands prevent branch selection only while physically
        # crossing the confirmed node.
        return BLUE_SKIP_STRAIGHT_SPEED, BLUE_SKIP_STRAIGHT_SPEED

    # ------------------------------------------------------------------
    # BLUE DROP AREA EXIT: RIGHT branch with sensor lock.
    #
    # CSV 45 and CSV 46 both show that the real narrow line reaches the middle
    # sensor well after the old 0.62 s manoeuvre had already ended.
    # ------------------------------------------------------------------
    if _manoeuvre_kind == "blue_return_right_lock":
        global _blue_return_right_handoff_frames

        if raw is None:
            raw = list(active)

        # Cover a few physical frames of the wide white node.
        if _manoeuvre_frame_count <= BLUE_RETURN_RIGHT_COVER_FRAMES:
            _last_manoeuvre_phase = (
                "blue_return_right_cover_node"
            )
            return (
                BLUE_RETURN_RIGHT_COVER_SPEED,
                BLUE_RETURN_RIGHT_COVER_SPEED,
            )

        # Both CSV 48 and successful CSV 49 show this exact turn-complete
        # signature: the first four sensors remain bright while right_corner
        # has cleared onto the dark background.
        node_right_edge_cleared = (
            raw[0] >= BLUE_RETURN_RIGHT_EXIT_BRIGHT_MIN
            and raw[1] >= BLUE_RETURN_RIGHT_EXIT_BRIGHT_MIN
            and raw[2] >= BLUE_RETURN_RIGHT_EXIT_BRIGHT_MIN
            and raw[3] >= BLUE_RETURN_RIGHT_EXIT_BRIGHT_MIN
            and raw[4] <= BLUE_RETURN_RIGHT_EXIT_RIGHT_CORNER_MAX
        )

        if node_right_edge_cleared:
            _manoeuvre_center_frames += 1
        else:
            _manoeuvre_center_frames = 0

        turn_complete = (
            _manoeuvre_center_frames
            >= BLUE_RETURN_RIGHT_EXIT_REQUIRED_FRAMES
        )
        frame_safety_reached = (
            _manoeuvre_frame_count
            >= BLUE_RETURN_RIGHT_MAX_TURN_FRAMES
        )

        if turn_complete or frame_safety_reached:
            # Do not search for another branch and do not keep turning.
            # Hand control to a short capped-PID line-following ramp.
            position_now, _ = _line_position(active)
            error_now = (
                -position_now if position_now is not None else 0.0
            )

            _previous_error = error_now
            _last_valid_error = error_now
            _integral = 0.0
            _last_pid_time = now
            _blue_return_right_handoff_frames = (
                BLUE_RETURN_RIGHT_HANDOFF_FRAMES
            )

            reason = (
                "four-bright plus dark-right-corner pattern"
                if turn_complete
                else "frame safety before over-turn"
            )
            _finish_manoeuvre(
                "MANOEUVRE blue_return_right_lock complete: "
                f"{reason}; continue route with PID"
            )
            return None

        # As the right corner begins to clear, soften the right arc. The robot
        # must not continue the previous hard turn through the continuous line.
        if raw[4] <= 0.58:
            _last_manoeuvre_phase = (
                "blue_return_right_soft_until_pid_release"
            )
            return (
                BLUE_RETURN_RIGHT_SOFT_LEFT_SPEED,
                BLUE_RETURN_RIGHT_SOFT_RIGHT_SPEED,
            )

        _last_manoeuvre_phase = (
            "blue_return_right_required_arc"
        )
        return (
            BLUE_RETURN_RIGHT_ARC_LEFT_SPEED,
            BLUE_RETURN_RIGHT_ARC_RIGHT_SPEED,
        )

    # ------------------------------------------------------------------
    # THIRD WHITE JUNCTION: STRONG LEFT AND LOCK BLACK RETURN LINE.
    #
    # This manoeuvre spans the white-to-black surface change and finishes only
    # after the black line is physically centred.
    # ------------------------------------------------------------------
    if _manoeuvre_kind == "blue_return_left_black_lock":
        global _blue_return_left_handoff_frames

        if raw is None:
            raw = list(active)

        # Cover a small physical part of the wide white node.
        if _manoeuvre_frame_count <= BLUE_RETURN_LEFT_COVER_FRAMES:
            _last_manoeuvre_phase = (
                "blue_return_left_cover_white_node"
            )
            return (
                BLUE_RETURN_LEFT_COVER_SPEED,
                BLUE_RETURN_LEFT_COVER_SPEED,
            )

        # CSV 50 reaches this shape around frame 4920. Successful CSV 49
        # reaches the same shape around frames 9354–9356:
        # darkening left_corner + four still-bright white-line sensors.
        selected_white_branch_seen = (
            raw[0] <= BLUE_RETURN_LEFT_EXIT_CORNER_MAX
            and raw[1] >= BLUE_RETURN_LEFT_EXIT_INNER_MIN
            and raw[2] >= BLUE_RETURN_LEFT_EXIT_INNER_MIN
            and raw[3] >= BLUE_RETURN_LEFT_EXIT_INNER_MIN
            and raw[4] >= BLUE_RETURN_LEFT_EXIT_INNER_MIN
        )

        if selected_white_branch_seen:
            _manoeuvre_center_frames += 1
        else:
            _manoeuvre_center_frames = 0

        branch_locked = (
            _manoeuvre_center_frames
            >= BLUE_RETURN_LEFT_EXIT_REQUIRED_FRAMES
        )
        frame_safety_reached = (
            _manoeuvre_frame_count
            >= BLUE_RETURN_LEFT_MAX_TURN_FRAMES
        )

        if branch_locked or frame_safety_reached:
            position_now, _ = _line_position(active)
            error_now = (
                -position_now if position_now is not None else 0.0
            )

            _previous_error = error_now
            _last_valid_error = error_now
            _integral = 0.0
            _last_pid_time = now
            _blue_return_left_handoff_frames = (
                BLUE_RETURN_LEFT_HANDOFF_FRAMES
            )

            reason = (
                "selected white branch detected"
                if branch_locked
                else "frame safety before over-turn"
            )
            _finish_manoeuvre(
                "MANOEUVRE blue_return_left_black_lock complete: "
                f"{reason}; follow curved white branch with PID"
            )
            return None

        if raw[0] <= 0.64:
            _last_manoeuvre_phase = (
                "blue_return_left_soft_branch_edge"
            )
            return (
                BLUE_RETURN_LEFT_SOFT_LEFT_SPEED,
                BLUE_RETURN_LEFT_SOFT_RIGHT_SPEED,
            )

        _last_manoeuvre_phase = (
            "blue_return_left_follow_white_branch"
        )
        return (
            BLUE_RETURN_LEFT_ARC_LEFT_SPEED,
            BLUE_RETURN_LEFT_ARC_RIGHT_SPEED,
        )

    # ------------------------------------------------------------------
    # FIRST WHITE-LINE INTERSECTION: choose the gentle LEFT branch.
    #
    # Both motors stay forward. This is not a pivot and not a 90-degree turn.
    # The robot first moves farther into the node, then tracks:
    # left_corner -> left -> middle.
    # ------------------------------------------------------------------
    if _manoeuvre_kind == "white_left_gentle":
        if raw is None:
            raw = list(active)

        # Move farther into the centre of the white node before steering.
        if elapsed < WHITE_LEFT_ADVANCE_SECONDS:
            _last_manoeuvre_phase = "white_left_advance_to_node_centre"
            return WHITE_LEFT_ADVANCE_SPEED, WHITE_LEFT_ADVANCE_SPEED

        turn_elapsed = elapsed - WHITE_LEFT_ADVANCE_SECONDS

        bright_count = sum(
            value >= WHITE_LEFT_LINE_THRESHOLD for value in raw
        )
        bright_total = sum(raw)

        white_left_corner = raw[0] >= WHITE_LEFT_LINE_THRESHOLD
        white_left = raw[1] >= WHITE_LEFT_LINE_THRESHOLD
        white_middle = raw[2] >= WHITE_LEFT_LINE_THRESHOLD
        dark_right = raw[3] <= WHITE_LEFT_DARK_THRESHOLD
        dark_right_corner = raw[4] <= WHITE_LEFT_DARK_THRESHOLD

        # The wide node initially covers four/five sensors.
        if bright_count <= 3 and bright_total <= 3.20:
            _manoeuvre_node_cleared = True

        # The required outgoing branch must appear on the LEFT first.
        if (
            _manoeuvre_node_cleared
            and (white_left_corner or white_left)
        ):
            _manoeuvre_branch_seen = True

        # Finish only after the chosen left branch reaches the middle sensor.
        left_branch_centred = (
            _manoeuvre_branch_seen
            and _manoeuvre_node_cleared
            and white_middle
            and (white_left_corner or white_left)
            and dark_right
            and dark_right_corner
            and 0.45 <= bright_total <= 2.75
        )

        if (
            turn_elapsed >= WHITE_LEFT_MIN_TURN_SECONDS
            and left_branch_centred
        ):
            _manoeuvre_center_frames += 1
        else:
            _manoeuvre_center_frames = 0

        if _manoeuvre_center_frames >= WHITE_LEFT_CENTER_REQUIRED_FRAMES:
            # Match PID history to the newly selected line to avoid an immediate
            # opposite-direction derivative kick.
            position_now, _ = _line_position(active)
            error_now = -position_now if position_now is not None else 0.0
            _previous_error = error_now
            _integral = 0.0
            _last_pid_time = now

            _finish_manoeuvre(
                "MANOEUVRE white_left_gentle complete: "
                "LEFT white branch centred; continue PID"
            )
            return None

        # Gentle LEFT arc: the right wheel is only moderately faster.
        if not _manoeuvre_branch_seen:
            _last_manoeuvre_phase = "white_left_slow_search"
            return (
                WHITE_LEFT_SEARCH_LEFT_SPEED,
                WHITE_LEFT_SEARCH_RIGHT_SPEED,
            )

        if white_left_corner and not white_left:
            _last_manoeuvre_phase = "white_left_follow_left_corner"
            return (
                WHITE_LEFT_SEARCH_LEFT_SPEED,
                WHITE_LEFT_SEARCH_RIGHT_SPEED,
            )

        if white_left and not white_middle:
            _last_manoeuvre_phase = "white_left_follow_left_sensor"
            return (
                WHITE_LEFT_ACQUIRE_LEFT_SPEED,
                WHITE_LEFT_ACQUIRE_RIGHT_SPEED,
            )

        if white_middle:
            _last_manoeuvre_phase = "white_left_soft_centre"
            return (
                WHITE_LEFT_CENTER_LEFT_SPEED,
                WHITE_LEFT_CENTER_RIGHT_SPEED,
            )

        if turn_elapsed < WHITE_LEFT_MAX_SECONDS:
            _last_manoeuvre_phase = "white_left_safe_forward_arc"
            return 1.16, 1.48

        # Do not rotate farther if acquisition takes longer. Move slowly forward
        # with only a tiny left bias and keep searching for the line.
        _last_manoeuvre_phase = "white_left_timeout_slow_follow"
        return 1.12, 1.26

    # ------------------------------------------------------------------
    # WHITE-LINE RIGHT BRANCH
    #
    # Used at:
    #   1. the first RED three-line junction toward the red rectangle;
    #   2. the final node after dropping RED, toward the finish square.
    #
    # Both wheels stay forward. The outgoing line must progress through:
    # right_corner -> right -> middle.
    # ------------------------------------------------------------------
    if _manoeuvre_kind == "white_right_gentle":
        if raw is None:
            raw = list(active)

        # Cover the centre of the wide white node before steering.
        if elapsed < WHITE_RIGHT_ADVANCE_SECONDS:
            _last_manoeuvre_phase = "white_right_advance_to_node_centre"
            return WHITE_RIGHT_ADVANCE_SPEED, WHITE_RIGHT_ADVANCE_SPEED

        turn_elapsed = elapsed - WHITE_RIGHT_ADVANCE_SECONDS

        bright_count = sum(
            value >= WHITE_RIGHT_LINE_THRESHOLD for value in raw
        )
        bright_total = sum(raw)

        white_middle = raw[2] >= WHITE_RIGHT_LINE_THRESHOLD
        white_right = raw[3] >= WHITE_RIGHT_LINE_THRESHOLD
        white_right_corner = raw[4] >= WHITE_RIGHT_LINE_THRESHOLD
        dark_left = raw[1] <= WHITE_RIGHT_DARK_THRESHOLD
        dark_left_corner = raw[0] <= WHITE_RIGHT_DARK_THRESHOLD

        # The wide node initially covers four/five sensors. Do not accept the
        # outgoing branch until the node begins clearing.
        if bright_count <= 3 and bright_total <= 3.20:
            _manoeuvre_node_cleared = True

        # The required branch must first remain visible on the RIGHT sensors.
        if (
            _manoeuvre_node_cleared
            and (white_right_corner or white_right)
        ):
            _manoeuvre_branch_seen = True

        # Finish only when the selected right branch reaches the middle sensor
        # and the left sensors are back over the black background.
        right_branch_centred = (
            _manoeuvre_branch_seen
            and _manoeuvre_node_cleared
            and white_middle
            and (white_right_corner or white_right)
            and dark_left
            and dark_left_corner
            and 0.45 <= bright_total <= 2.75
        )

        if (
            turn_elapsed >= WHITE_RIGHT_MIN_TURN_SECONDS
            and right_branch_centred
        ):
            _manoeuvre_center_frames += 1
        else:
            _manoeuvre_center_frames = 0

        if (
            _manoeuvre_center_frames
            >= WHITE_RIGHT_CENTER_REQUIRED_FRAMES
        ):
            position_now, _ = _line_position(active)
            error_now = -position_now if position_now is not None else 0.0
            _previous_error = error_now
            _integral = 0.0
            _last_pid_time = now

            _finish_manoeuvre(
                "MANOEUVRE white_right_gentle complete: "
                "RIGHT white branch centred; continue PID"
            )
            return None

        # Gentle RIGHT arc: left motor is moderately faster.
        if not _manoeuvre_branch_seen:
            _last_manoeuvre_phase = "white_right_slow_search"
            return (
                WHITE_RIGHT_SEARCH_LEFT_SPEED,
                WHITE_RIGHT_SEARCH_RIGHT_SPEED,
            )

        if white_right_corner and not white_right:
            _last_manoeuvre_phase = "white_right_follow_right_corner"
            return (
                WHITE_RIGHT_SEARCH_LEFT_SPEED,
                WHITE_RIGHT_SEARCH_RIGHT_SPEED,
            )

        if white_right and not white_middle:
            _last_manoeuvre_phase = "white_right_follow_right_sensor"
            return (
                WHITE_RIGHT_ACQUIRE_LEFT_SPEED,
                WHITE_RIGHT_ACQUIRE_RIGHT_SPEED,
            )

        if white_middle:
            _last_manoeuvre_phase = "white_right_soft_centre"
            return (
                WHITE_RIGHT_CENTER_LEFT_SPEED,
                WHITE_RIGHT_CENTER_RIGHT_SPEED,
            )

        if turn_elapsed < WHITE_RIGHT_MAX_SECONDS:
            _last_manoeuvre_phase = "white_right_safe_forward_arc"
            return 1.48, 1.16

        # Do not pivot or reverse if line acquisition is delayed.
        _last_manoeuvre_phase = "white_right_timeout_slow_follow"
        return 1.26, 1.12

    # ------------------------------------------------------------------
    # FINAL WHITE NODE: near-90-degree RIGHT turn toward the end square.
    # ------------------------------------------------------------------
    if _manoeuvre_kind == "finish_right_line_lock":
        global _finish_right_handoff_until
        global _finish_right_handoff_center_frames

        if raw is None:
            raw = list(active)

        # The incoming route is already curving. Move only briefly into the
        # wide node before beginning the right arc.
        if elapsed < FINISH_RIGHT_ADVANCE_SECONDS:
            _last_manoeuvre_phase = "finish_right_short_node_cover"
            return (
                FINISH_RIGHT_ADVANCE_SPEED,
                FINISH_RIGHT_ADVANCE_SPEED,
            )

        turn_elapsed = elapsed - FINISH_RIGHT_ADVANCE_SECONDS
        raw_total = sum(raw)
        max_raw = max(raw)

        white_middle = raw[2] >= FINISH_RIGHT_LINE_THRESHOLD
        white_right = raw[3] >= FINISH_RIGHT_LINE_THRESHOLD
        white_right_corner = (
            raw[4] >= FINISH_RIGHT_LINE_THRESHOLD
        )

        # Phase 1: leave the wide white node.
        #
        # CSV 31/32 show that left+middle white values during this phase are
        # only the old node surface. They must never be treated as the new
        # finish branch.
        black_gap_seen = (
            raw_total <= FINISH_RIGHT_NODE_CLEAR_RAW_SUM
            or max_raw <= FINISH_RIGHT_GAP_MAX_SENSOR
        )

        if black_gap_seen:
            _manoeuvre_node_cleared = True

        # Phase 2: after the node/gap has cleared, accept only a NEW white line
        # entering from the robot's RIGHT side.
        if (
            _manoeuvre_node_cleared
            and (white_right_corner or white_right)
        ):
            _manoeuvre_branch_seen = True

        # The new finish branch is centred only when the middle sensor is
        # strongly white and the wide node is no longer covering all sensors.
        narrow_finish_line_centred = (
            _manoeuvre_branch_seen
            and _manoeuvre_node_cleared
            and white_middle
            and raw_total <= 2.15
            and sum(
                value >= FINISH_RIGHT_LINE_THRESHOLD
                for value in raw
            ) <= 3
        )

        if narrow_finish_line_centred:
            _manoeuvre_center_frames += 1
        else:
            _manoeuvre_center_frames = 0

        if (
            _manoeuvre_center_frames
            >= FINISH_RIGHT_CENTER_REQUIRED_FRAMES
        ):
            _finish_right_handoff_until = (
                now + FINISH_RIGHT_HANDOFF_SECONDS
            )
            _finish_right_handoff_center_frames = 0

            position_now, _ = _line_position(active)
            error_now = (
                -position_now if position_now is not None else 0.0
            )
            _previous_error = error_now
            _integral = 0.0
            _last_pid_time = now

            _finish_manoeuvre(
                "MANOEUVRE finish_right_line_lock complete: "
                "NEW finish branch centred from RIGHT sensors"
            )
            return None

        # Still on the wide white node: take a smooth forward right arc.
        if not _manoeuvre_node_cleared:
            _last_manoeuvre_phase = "finish_right_leave_wide_node"
            return (
                FINISH_RIGHT_ARC_LEFT_SPEED,
                FINISH_RIGHT_ARC_RIGHT_SPEED,
            )

        # The old node is cleared. Tighten the right arc, but keep both motors
        # forward, until the new branch enters from right_corner.
        if not _manoeuvre_branch_seen:
            _last_manoeuvre_phase = "finish_right_search_new_branch"
            return (
                FINISH_RIGHT_SEARCH_LEFT_SPEED,
                FINISH_RIGHT_SEARCH_RIGHT_SPEED,
            )

        # Follow the new line across the sensor bar:
        # right_corner -> right -> middle.
        if white_right_corner and not white_right:
            _last_manoeuvre_phase = "finish_right_follow_right_corner"
            return (
                FINISH_RIGHT_CORNER_LEFT_SPEED,
                FINISH_RIGHT_CORNER_RIGHT_SPEED,
            )

        if white_right and not white_middle:
            _last_manoeuvre_phase = "finish_right_follow_right_sensor"
            return (
                FINISH_RIGHT_INNER_LEFT_SPEED,
                FINISH_RIGHT_INNER_RIGHT_SPEED,
            )

        if white_middle:
            _last_manoeuvre_phase = "finish_right_center_new_line"
            return (
                FINISH_RIGHT_CENTER_LEFT_SPEED,
                FINISH_RIGHT_CENTER_RIGHT_SPEED,
            )

        # Continue a forward-only right search. Do not finish the manoeuvre and
        # do not turn left merely because the search takes longer.
        if turn_elapsed < FINISH_RIGHT_MAX_SEARCH_SECONDS:
            _last_manoeuvre_phase = "finish_right_continue_search"
            return (
                FINISH_RIGHT_SEARCH_LEFT_SPEED,
                FINISH_RIGHT_SEARCH_RIGHT_SPEED,
            )

        _last_manoeuvre_phase = "finish_right_extended_safe_search"
        return 1.22, 0.28

    # ------------------------------------------------------------------
    # RETURN TO CIRCLE: second black node -> LEFT onto connector.
    # ------------------------------------------------------------------
    if _manoeuvre_kind == "return_black_left_connector":
        if elapsed < RETURN_BLACK_LEFT_ADVANCE_SECONDS:
            _last_manoeuvre_phase = "return_left_advance_over_black_dot"
            return (
                RETURN_BLACK_LEFT_ADVANCE_SPEED,
                RETURN_BLACK_LEFT_ADVANCE_SPEED,
            )

        turn_elapsed = elapsed - RETURN_BLACK_LEFT_ADVANCE_SECONDS
        strong_count = sum(
            value >= NODE_ACTIVE_THRESHOLD for value in active
        )
        active_total = sum(active)

        if strong_count <= 3 and active_total <= 3.25:
            _manoeuvre_node_cleared = True

        # Desired LEFT branch appears under the left sensors first.
        if (
            _manoeuvre_node_cleared
            and (active[0] >= 0.48 or active[1] >= 0.48)
        ):
            _manoeuvre_branch_seen = True

        connector_centred = (
            _manoeuvre_branch_seen
            and _manoeuvre_node_cleared
            and active[2] >= 0.42
            and (active[0] >= 0.40 or active[1] >= 0.40)
            and active[3] <= 0.24
            and active[4] <= 0.24
            and 0.42 <= active_total <= 2.85
        )

        if (
            turn_elapsed >= RETURN_BLACK_LEFT_MIN_TURN_SECONDS
            and connector_centred
        ):
            _manoeuvre_center_frames += 1
        else:
            _manoeuvre_center_frames = 0

        if (
            _manoeuvre_center_frames
            >= RETURN_BLACK_LEFT_CENTER_REQUIRED_FRAMES
        ):
            position_now, _ = _line_position(active)
            error_now = -position_now if position_now is not None else 0.0
            _previous_error = error_now
            _integral = 0.0
            _last_pid_time = now
            _finish_manoeuvre(
                "MANOEUVRE return_black_left_connector complete: "
                "connector centred; continue black PID"
            )
            return None

        # Smooth LEFT arc: right wheel faster, both motors forward.
        if not _manoeuvre_branch_seen:
            _last_manoeuvre_phase = "return_left_search_connector"
            return (
                RETURN_BLACK_LEFT_SEARCH_LEFT_SPEED,
                RETURN_BLACK_LEFT_SEARCH_RIGHT_SPEED,
            )

        if active[0] >= 0.48 and active[1] < 0.40:
            _last_manoeuvre_phase = "return_left_follow_corner"
            return (
                RETURN_BLACK_LEFT_SEARCH_LEFT_SPEED,
                RETURN_BLACK_LEFT_SEARCH_RIGHT_SPEED,
            )

        if active[1] >= 0.40 and active[2] < 0.38:
            _last_manoeuvre_phase = "return_left_follow_inner_sensor"
            return (
                RETURN_BLACK_LEFT_ACQUIRE_LEFT_SPEED,
                RETURN_BLACK_LEFT_ACQUIRE_RIGHT_SPEED,
            )

        if active[2] >= 0.38:
            _last_manoeuvre_phase = "return_left_soft_centre"
            return (
                RETURN_BLACK_LEFT_CENTER_LEFT_SPEED,
                RETURN_BLACK_LEFT_CENTER_RIGHT_SPEED,
            )

        if turn_elapsed < RETURN_BLACK_LEFT_MAX_SECONDS:
            _last_manoeuvre_phase = "return_left_safe_forward_arc"
            return 1.10, 1.46

        # Do not spin away from the connector if acquisition is delayed.
        _last_manoeuvre_phase = "return_left_timeout_slow_follow"
        return 1.08, 1.24

    # ------------------------------------------------------------------
    # RED circle entry: mirror the successful BLUE CSV-13 turn.
    # ------------------------------------------------------------------
    if _manoeuvre_kind == "red_circle_left_replay":
        global _red_entry_handoff_until

        if elapsed < RED_ENTRY_ADVANCE_SECONDS:
            _last_manoeuvre_phase = "red_enter_circle_node"
            return RED_ENTRY_ADVANCE_SPEED, RED_ENTRY_ADVANCE_SPEED

        turn_elapsed = elapsed - RED_ENTRY_ADVANCE_SECONDS

        if turn_elapsed < RED_ENTRY_FIXED_LEFT_TURN_SECONDS:
            _last_manoeuvre_phase = "red_replay_mirrored_left_turn"
            return RED_ENTRY_LEFT_SPEED, RED_ENTRY_RIGHT_SPEED

        _red_entry_handoff_until = now + RED_ENTRY_HANDOFF_SECONDS
        position_now, _ = _line_position(active)
        error_now = -position_now if position_now is not None else 0.0
        _previous_error = error_now
        _integral = 0.0
        _last_pid_time = now
        _finish_manoeuvre(
            "MANOEUVRE red_circle_left_replay complete: "
            "mirrored BLUE turn finished; continue red circle"
        )
        return None

    # ------------------------------------------------------------------
    # RED circle exit: LEFT onto the connector.
    # Mirror of the working BLUE right-exit acquisition.
    # ------------------------------------------------------------------
    if _manoeuvre_kind == "red_exit_left_acquire":
        global _red_exit_handoff_until, _red_exit_handoff_center_frames

        if elapsed < RED_EXIT_ADVANCE_SECONDS:
            _last_manoeuvre_phase = "red_exit_advance_over_circle_dot"
            return RED_EXIT_ADVANCE_SPEED, RED_EXIT_ADVANCE_SPEED

        turn_elapsed = elapsed - RED_EXIT_ADVANCE_SECONDS
        strong_count = sum(
            value >= NODE_ACTIVE_THRESHOLD for value in active
        )
        active_total = sum(active)

        if strong_count <= 3 and active_total <= 3.30:
            _manoeuvre_node_cleared = True

        # During a LEFT turn, the connector enters beneath the RIGHT sensors.
        if (
            _manoeuvre_node_cleared
            and (active[3] >= 0.55 or active[4] >= 0.55)
            and active[0] <= 0.18
            and active[1] <= 0.18
        ):
            _manoeuvre_branch_seen = True

        connector_centred = (
            _manoeuvre_node_cleared
            and _manoeuvre_branch_seen
            and active[2] >= 0.34
            and active[3] >= 0.40
            and active[4] >= 0.55
            and active[0] <= 0.18
            and active[1] <= 0.18
            and strong_count <= 3
            and 1.55 <= active_total <= 2.80
        )

        if (
            turn_elapsed >= RED_EXIT_CURVE_MIN_SECONDS
            and connector_centred
        ):
            _manoeuvre_center_frames += 1
        else:
            _manoeuvre_center_frames = 0

        if (
            _manoeuvre_center_frames
            >= RED_EXIT_CENTER_REQUIRED_FRAMES
        ):
            _red_exit_handoff_until = now + RED_EXIT_HANDOFF_SECONDS
            _red_exit_handoff_center_frames = 0
            position_now, _ = _line_position(active)
            error_now = -position_now if position_now is not None else 0.0
            _previous_error = error_now
            _integral = 0.0
            _last_pid_time = now
            _finish_manoeuvre(
                "MANOEUVRE red_exit_left_acquire complete: "
                "connector centred; continue to main path"
            )
            return None

        if turn_elapsed < RED_EXIT_CURVE_MAX_SECONDS:
            _last_manoeuvre_phase = "red_exit_curve_left_to_connector"
            return RED_EXIT_LEFT_SPEED, RED_EXIT_RIGHT_SPEED

        search_elapsed = turn_elapsed - RED_EXIT_CURVE_MAX_SECONDS
        if search_elapsed < RED_EXIT_SEARCH_SECONDS:
            _last_manoeuvre_phase = "red_exit_search_connector"
            return -RED_EXIT_SEARCH_SPEED, RED_EXIT_SEARCH_SPEED

        _red_exit_handoff_until = now + RED_EXIT_HANDOFF_SECONDS
        _red_exit_handoff_center_frames = 0
        _previous_error = 0.0
        _integral = 0.0
        _last_pid_time = now
        _finish_manoeuvre(
            "MANOEUVRE red_exit_left_acquire ended by safety timeout; "
            "continue connector capture"
        )
        return None

    # ------------------------------------------------------------------
    # RED connector -> RIGHT onto the black main path.
    # Exact duplicate of the working BLUE main-path acquisition, but it moves
    # to RED_TO_WHITE instead of changing any BLUE-route state.
    # ------------------------------------------------------------------
    if _manoeuvre_kind == "red_main_right_follow":
        global _red_main_handoff_until, _red_main_handoff_center_frames

        if raw is None:
            raw = [1.0 - (max(0.0, value) ** 0.5) for value in active]

        red_main_advance_seconds = (
            MAIN_ENTRY_ADVANCE_SECONDS + RED_MAIN_EXTRA_ADVANCE_SECONDS
        )

        if elapsed < red_main_advance_seconds:
            _last_manoeuvre_phase = "red_main_advance_over_black_dot"
            return MAIN_ENTRY_ADVANCE_SPEED, MAIN_ENTRY_ADVANCE_SPEED

        turn_elapsed = elapsed - red_main_advance_seconds

        white_right_corner = raw[4] >= 0.68
        white_right = raw[3] >= 0.68
        white_middle = raw[2] >= 0.54
        black_left_corner = raw[0] <= 0.24
        black_left = raw[1] <= 0.24

        if white_right_corner:
            _manoeuvre_branch_seen = True

        black_main_line_acquired = (
            _manoeuvre_branch_seen
            and black_left_corner
            and black_left
            and white_middle
            and white_right
            and white_right_corner
        )

        if black_main_line_acquired:
            _manoeuvre_center_frames += 1
        else:
            _manoeuvre_center_frames = 0

        if _manoeuvre_center_frames >= MAIN_ENTRY_CENTER_REQUIRED_FRAMES:
            _line_mode = "black"
            _set_state(
                RED_TO_WHITE,
                "red connector completed; black main-path line acquired",
            )
            _red_main_handoff_until = now + MAIN_ENTRY_PID_HANDOFF_SECONDS
            _red_main_handoff_center_frames = 0
            _previous_error = 0.0
            _integral = 0.0
            _last_pid_time = now
            _finish_manoeuvre(
                "MANOEUVRE red_main_right_follow complete: "
                "black main-path line locked; continue toward white section"
            )
            return None

        if not _manoeuvre_branch_seen:
            _last_manoeuvre_phase = "red_main_hard_right_search"
            return MAIN_ENTRY_HARD_RIGHT_SPEED, MAIN_ENTRY_HARD_INNER_SPEED

        if white_right_corner and not white_right:
            _last_manoeuvre_phase = "red_main_follow_right_corner"
            return MAIN_ENTRY_HARD_RIGHT_SPEED, MAIN_ENTRY_HARD_INNER_SPEED

        if white_right and not white_middle:
            _last_manoeuvre_phase = "red_main_follow_right_sensor"
            return (
                MAIN_ENTRY_MEDIUM_RIGHT_SPEED,
                MAIN_ENTRY_MEDIUM_INNER_SPEED,
            )

        if white_middle:
            _last_manoeuvre_phase = "red_main_soft_right_until_black_lock"
            return 1.55, 1.08

        if turn_elapsed < MAIN_ENTRY_MAX_SECONDS:
            _last_manoeuvre_phase = "red_main_safe_right_search"
            return 1.55, 0.62

        _last_manoeuvre_phase = "red_main_timeout_slow_forward"
        return 1.05, 1.05

    # ------------------------------------------------------------------
    # Original short manoeuvres used at later route nodes.
    # ------------------------------------------------------------------
    if elapsed >= _manoeuvre_duration:
        _finish_manoeuvre(f"MANOEUVRE {_manoeuvre_kind} complete")
        return None

    # Move slightly into the centre of later nodes before beginning the turn.
    if elapsed < 0.13:
        _last_manoeuvre_phase = "enter_node"
        return 1.65, 1.65

    _last_manoeuvre_phase = "turn"
    if _manoeuvre_kind == "left":
        return 0.05, 2.15
    if _manoeuvre_kind == "right":
        return 2.15, 0.05
    if _manoeuvre_kind == "left_soft":
        return 0.55, 1.90
    if _manoeuvre_kind == "right_soft":
        return 1.90, 0.55
    if _manoeuvre_kind == "straight":
        return 1.75, 1.75

    return None


def _raw_values(sensors):
    return [float(sensors.get(name, 0.0)) for name in SENSOR_ORDER]


def _active_values(raw):
    """Convert raw sensor values into line strengths for the selected mode."""
    if _line_mode == "black":
        values = [1.0 - value for value in raw]
    else:
        values = list(raw)

    # Squaring suppresses weak background reflections and preserves strong line
    # responses, giving a smoother centroid on both colour regimes.
    return [max(0.0, min(1.0, value)) ** 2 for value in values]


def _line_position(active):
    total = sum(active)
    if total < 0.12:
        return None, total
    position = sum(weight * value for weight, value in zip(WEIGHTS, active)) / total
    return position, total


def _detect_node(active, now):
    """Debounced detection of a highlighted route node.

    The robot starts on a wide black marker. Four sensors can see black there,
    which is almost identical to a real junction. Therefore START_TO_CIRCLE
    node detection is armed only after the robot has left that marker and
    observed an ordinary narrow line for several consecutive frames.
    """
    global _node_frames, _node_latched, _last_node_now, _last_node_event
    global _start_node_armed, _start_clear_frames, _last_strong_count
    global _circle_entry_approach, _circle_entry_approach_started

    strong_count = sum(value >= NODE_ACTIVE_THRESHOLD for value in active)
    active_total = sum(active)
    _last_strong_count = strong_count

    # Reject the initial START marker using sensor shape, not a fixed CSV frame.
    if _route_state == START_TO_CIRCLE and not _start_node_armed:
        ordinary_narrow_line = (
            strong_count <= 2
            and 0.18 <= active_total <= 2.25
        )

        if ordinary_narrow_line:
            _start_clear_frames += 1
        else:
            _start_clear_frames = 0

        if (
            now - _start_time >= START_GATE_MIN_SECONDS
            and _start_clear_frames >= START_CLEAR_REQUIRED_FRAMES
        ):
            _start_node_armed = True
            _node_frames = 0
            _node_latched = False
            _add_event(
                "START marker cleared; first real route-node detector armed"
            )

        # Never generate a node event while leaving the starting patch.
        _last_node_now = False
        _last_node_event = False
        return False

    # At the first real junction the CSV shows three sensors becoming active
    # around frame 1638, but the robot is still before the connector mouth.
    # All five sensors first cover the highlighted dot around frame 1644.  Two
    # stable five-sensor frames place the turn trigger around frame 1645, which
    # is approximately 0.39 s later and centres the axle on the node.
    if _route_state == START_TO_CIRCLE:
        node_now = (
            strong_count >= FIRST_NODE_REQUIRED_SENSORS
            and active_total >= FIRST_NODE_ACTIVE_THRESHOLD
        )
        required_node_frames = 1
        _circle_entry_approach = False

    elif _route_state == BLUE_CONNECTOR:
        # The second black spot is the circle entrance.
        approach_now = (
            strong_count >= 4
            and active_total >= CIRCLE_ENTRY_APPROACH_ACTIVE_SUM
        )

        if approach_now and not _circle_entry_approach:
            _circle_entry_approach_started = now
        elif not approach_now:
            _circle_entry_approach_started = 0.0

        _circle_entry_approach = approach_now

        perfectly_centred = (
            strong_count >= CIRCLE_ENTRY_REQUIRED_SENSORS
            and active_total >= CIRCLE_ENTRY_ACTIVE_THRESHOLD
        )
        csv13_timing_reached = (
            _circle_entry_approach_started > 0.0
            and now - _circle_entry_approach_started
                >= CIRCLE_ENTRY_FORCE_AFTER_SECONDS
        )

        # Use the perfect five-sensor centre when available. Otherwise reproduce
        # the successful CSV-13 timing after the four-sensor edge is detected.
        node_now = perfectly_centred or csv13_timing_reached
        required_node_frames = CIRCLE_ENTRY_REQUIRED_FRAMES

    else:
        _circle_entry_approach = False
        _circle_entry_approach_started = 0.0
        node_now = (
            strong_count >= NODE_REQUIRED_SENSORS
            and active_total >= 2.60
        )
        required_node_frames = 1

    if node_now:
        _node_frames += 1
    else:
        _node_frames = 0
        if _node_latched:
            _node_latched = False

    node_event = False
    if (
        _node_frames >= required_node_frames
        and not _node_latched
        and now >= _node_cooldown_until
    ):
        _node_latched = True
        node_event = True

    _last_node_now = node_now
    _last_node_event = node_event
    return node_event


def _switch_line_mode_if_needed(raw, now):
    """Switch line colour only after a real high-contrast transition."""
    global _line_mode, _line_switch_counter, _node_latched, _node_frames
    global _blue_return_left_handoff_frames

    raw_mean = sum(raw) / 5.0
    bright_count = sum(value >= 0.58 for value in raw)
    dark_count = sum(value <= 0.32 for value in raw)

    # A real WHITE line must enter beneath an INNER sensor. A bright outer
    # corner alone is only the edge of a black dot/junction.
    inner_bright_count = sum(
        value >= 0.58 for value in raw[1:4]
    )

    # Black line / white floor -> white line / black floor.
    # Require three consecutive inner-sensor frames.
    if _route_state in (BLUE_TO_WHITE, RED_TO_WHITE):
        actual_white_track = (
            raw_mean < 0.42
            and bright_count >= 1
            and inner_bright_count >= 1
            and dark_count >= 3
            and _manoeuvre_kind is None
            and now - _state_enter_time >= 0.25
        )
        if actual_white_track:
            _line_switch_counter += 1
        else:
            _line_switch_counter = 0

        if _line_switch_counter >= LINE_SWITCH_FRAMES:
            _line_mode = "white"
            _node_latched = False
            _node_frames = 0
            if _route_state == BLUE_TO_WHITE:
                _set_state(BLUE_WHITE_TO_DROP, "stable white-line section detected")
            else:
                _set_state(RED_WHITE_TO_DROP, "stable white-line section detected")
            return True

    # White line / black floor -> black line / white floor.
    elif _route_state == BLUE_TO_BLACK:
        actual_black_track = (
            raw_mean > 0.58
            and bright_count >= 3
            and dark_count >= 1
            and _manoeuvre_kind is None
            and now - _state_enter_time >= 0.25
        )
        if actual_black_track:
            _line_switch_counter += 1
        else:
            _line_switch_counter = 0

        if _line_switch_counter >= LINE_SWITCH_FRAMES:
            _line_mode = "black"
            _blue_return_left_handoff_frames = 0
            _node_latched = False
            _node_frames = 0
            _set_state(RETURN_TO_CIRCLE, "stable black-line section detected")
            return True

    return False


def _handle_node_event(now, raw=None):
    """Route decisions made at the highlighted dots/junctions."""
    global _state_node_count, _circle_start_time
    global _blue_drop_marker_time, _blue_drop_marker_state_frame
    global _blue_drop_armed
    global _blue_drop_corner_adjusted
    global _state_frame_count
    global _blue_main_skip_completed
    global _red_drop_marker_time, _red_drop_armed

    _state_node_count += 1
    _add_event(f"NODE {_state_node_count} in {_route_state}")

    # First black junction: take the robot's RIGHT branch to the circle connector.
    if _route_state == START_TO_CIRCLE:
        _set_state(BLUE_CONNECTOR, "first circle junction detected; advance then curve right")
        _start_manoeuvre("right_90")
        return

    # SECOND highlighted black spot: circle entrance for the BLUE-box lap.
    # Take the robot's RIGHT-side circle branch toward the blue box.
    if _route_state == BLUE_CONNECTOR:
        _circle_start_time = now
        _set_state(
            BLUE_CIRCLE,
            "second black spot centred; turn RIGHT toward BLUE box",
        )
        _start_manoeuvre("blue_red_side_acquire")
        return

    # THIRD highlighted black node after the full BLUE lap.
    # Turn RIGHT here onto the connector that returns to the main path.
    if _route_state == BLUE_CIRCLE:
        if _blue_picked and now - _circle_start_time > 2.20:
            _set_state(
                BLUE_EXIT_TO_MAIN,
                "blue box collected; third black node reached; turn RIGHT to connector",
            )
            _start_manoeuvre("blue_exit_right_acquire")
        else:
            _add_event("circle node ignored: blue not picked or lap too short")
        return

    # Back at the main black junction: take RIGHT toward the long route.
    if _route_state == BLUE_EXIT_TO_MAIN:
        _add_event(
            "FOURTH black node centred: turn RIGHT and lock black main-path line"
        )
        _start_manoeuvre("blue_main_right_follow")
        return

    # The first highlighted black dot on the BLUE main path is only a marker.
    # It is not a turn and must not lock both wheels to equal speed.
    #
    # CSV 41 showed that the route curves immediately after this dot. Holding
    # equal wheel speeds made the robot continue straight and leave the line.
    # Therefore acknowledge the dot and let normal black-line PID continue.
    if _route_state == BLUE_TO_WHITE:
        if (
            _state_node_count == 1
            and not _blue_main_skip_completed
        ):
            _blue_main_skip_completed = True
            _add_event(
                "BLUE main-path black dot ignored; "
                "normal PID continues immediately"
            )
        return

    # First white junction after the colour transition: take the branch that leads
    # through the diamonds toward the blue drop rectangle.
    if _route_state == BLUE_WHITE_TO_DROP:
        if _state_node_count == 1:
            _add_event(
                "FIRST white node: move to centre and take gentle LEFT branch"
            )
            _start_manoeuvre("white_left_gentle")
            return

        # High-speed adaptive BLUE drop marker.
        #
        # The physical marker immediately before the rectangle is an all-white
        # dot. Earlier short nodes may be missed, so do not require NODE 6.
        # Route progress prevents earlier all-white junctions from arming DROP.
        marker_bright_count = (
            sum(
                value >= BLUE_DROP_MARKER_BRIGHT_THRESHOLD
                for value in raw
            )
            if raw is not None and len(raw) == 5
            else 0
        )
        marker_inner_bright = (
            raw is not None
            and len(raw) == 5
            and raw[1] >= BLUE_DROP_MARKER_INNER_MIN
            and raw[2] >= BLUE_DROP_MARKER_INNER_MIN
            and raw[3] >= BLUE_DROP_MARKER_INNER_MIN
        )
        marker_shape_last_white_node = (
            marker_bright_count
                >= BLUE_DROP_MARKER_REQUIRED_BRIGHT_SENSORS
            and marker_inner_bright
        )
        late_enough_for_drop_marker = (
            _state_frame_count
            >= BLUE_DROP_MARKER_MIN_STATE_FRAME
        )

        if (
            not _blue_drop_armed
            and late_enough_for_drop_marker
            and marker_shape_last_white_node
        ):
            _blue_drop_marker_time = now
            _blue_drop_marker_state_frame = _state_frame_count
            _blue_drop_armed = True
            _add_event(
                "BLUE last white node armed for exact-frame DROP: "
                f"node={_state_node_count}, "
                f"state_frame={_state_frame_count}, "
                f"bright={marker_bright_count}/5"
            )
            return

        elif (
            _state_node_count == BLUE_DROP_CORNER_NODE
            and not _blue_drop_corner_adjusted
        ):
            # Legacy sharp-corner support for slower runs where all nodes are
            # counted. It does not run after the adaptive marker is armed.
            _blue_drop_corner_adjusted = True
            _add_event(
                "Pre-blue-drop sharp turn: advance 0.18s, then use existing PID"
            )
            _start_manoeuvre(
                "straight",
                duration=BLUE_DROP_CORNER_ADVANCE_SECONDS,
            )

        elif (
            _state_node_count >= BLUE_DROP_MARKER_NODE
            and not _blue_drop_armed
        ):
            # Legacy fallback if the slower run counts all six nodes.
            _blue_drop_marker_time = now
            _blue_drop_marker_state_frame = _state_frame_count
            _blue_drop_armed = True
            _add_event(
                "BLUE drop marker armed by legacy node-count fallback: "
                f"node={_state_node_count}, "
                f"state_frame={_state_frame_count}"
            )

        return

    # After dropping blue, the next three-way node is taken to the robot's RIGHT.
    if _route_state == BLUE_AFTER_DROP:
        _set_state(
            BLUE_RETURN_WHITE,
            "leave blue zone through sensor-guided RIGHT branch",
        )
        _start_manoeuvre("blue_return_right_lock")
        return

    # On the upper return route, ignore the first two highlighted nodes.
    # CSV 23 confirms the THIRD white node is the three-line intersection.
    # Take LEFT here, using the same proven gentle white-line branch logic.
    if _route_state == BLUE_RETURN_WHITE:
        if _state_node_count == 3:
            _set_state(
                BLUE_TO_BLACK,
                "third white node reached; select LEFT white branch "
                "and follow its curve with PID",
            )
            _start_manoeuvre("blue_return_left_black_lock")
        return

    # Returning on black:
    #   node 1 = straight marker, ignore it;
    #   node 2 = circle connector, move over dot then turn LEFT.
    if _route_state == RETURN_TO_CIRCLE:
        if _state_node_count == 1:
            _add_event("first return black node ignored; continue PID")
        elif _state_node_count == 2:
            _set_state(
                RED_CONNECTOR,
                "second return black node; enter circle connector using LEFT branch",
            )
            _start_manoeuvre("return_black_left_connector")
        return

    # Circle entrance for RED: opposite direction from the BLUE pickup.
    if _route_state == RED_CONNECTOR:
        _circle_start_time = now
        _set_state(
            RED_CIRCLE,
            "circle entrance reached; take LEFT branch toward RED box",
        )
        _start_manoeuvre("red_circle_left_replay")
        return

    # After RED is collected and the full circle is completed, turn LEFT onto
    # the same connector used for returning to the main path.
    if _route_state == RED_CIRCLE:
        if _red_picked and now - _circle_start_time > 2.20:
            _set_state(
                RED_EXIT_TO_MAIN,
                "red box collected and full circle completed; turn LEFT to connector",
            )
            _start_manoeuvre("red_exit_left_acquire")
        else:
            _add_event("circle node ignored: red not picked or lap too short")
        return

    # At the main black junction, use the same successful RIGHT acquisition as
    # the BLUE journey, without modifying the locked BLUE movement block.
    if _route_state == RED_EXIT_TO_MAIN:
        _add_event(
            "red connector complete: advance over node and turn RIGHT to main path"
        )
        _start_manoeuvre("red_main_right_follow")
        return

    # RED delivery route on the upper white line:
    #   node 1 -> cover node and take RIGHT toward the red rectangle;
    #   node 2 -> continue with PID, no turn;
    #   node 3 -> last marker before the rectangle, arm RED drop.
    if _route_state == RED_WHITE_TO_DROP:
        if _state_node_count == 1:
            _add_event(
                "RED route first white junction: cover node and take RIGHT"
            )
            _start_manoeuvre("white_right_gentle")

        elif _state_node_count == 2:
            _add_event(
                "RED route white node 2 ignored; continue PID"
            )

        elif (
            _state_node_count == RED_DROP_MARKER_NODE
            and not _red_drop_armed
        ):
            _red_drop_marker_time = now
            _red_drop_armed = True
            _add_event(
                "RED drop marker armed at white node 3; "
                "continue briefly to red rectangle"
            )

        return

    # After RED is dropped, follow the white line to the next node. Cover that
    # node and take RIGHT with the same sensor-guided manoeuvre toward the final
    # white square.
    if _route_state == RED_AFTER_DROP:
        if _state_node_count == 1:
            _set_state(
                FINISH_ROUTE,
                "post-red-drop white node reached; cover node and take sharp RIGHT",
            )
            _start_manoeuvre("finish_right_line_lock")
        return


def _sync_carrying(carrying_color):
    """Infer successful PICK/DROP from the value maintained by the main loop."""
    global _last_carrying, _blue_picked, _red_picked
    global _blue_delivered, _red_delivered

    if carrying_color is not None and _last_carrying is None:
        if carrying_color == "blue":
            _blue_picked = True
            _add_event("BLUE PICK confirmed by main loop")
        elif carrying_color == "red":
            _red_picked = True
            _add_event("RED PICK confirmed by main loop")

    elif carrying_color is None and _last_carrying is not None:
        dropped_colour = _last_carrying
        if dropped_colour == "blue":
            _blue_delivered = True
            _set_state(BLUE_AFTER_DROP, "BLUE DROP confirmed by main loop")
        elif dropped_colour == "red":
            _red_delivered = True
            _set_state(RED_AFTER_DROP, "RED DROP confirmed by main loop")

    _last_carrying = carrying_color


def _log_csv(sensors, raw, active_sum, position, error, pid, left, right):
    global _frame_number, _event_note, _pick_requested, _drop_requested
    global _turbo_factor
    global _state_frame_count, _manoeuvre_frame_count
    global _blue_drop_marker_state_frame, _blue_drop_armed

    if _csv_writer is None:
        _event_note = ""
        _pick_requested = False
        _drop_requested = False
        return

    _frame_number += 1
    row = {
        "frame": _frame_number,
        "elapsed_s": round(time.perf_counter() - _turbo_real_start, 4),
        "state": _route_state,
        "line_mode": _line_mode,
        "event": _event_note,
        "left_corner": round(raw[0], 5),
        "left": round(raw[1], 5),
        "middle": round(raw[2], 5),
        "right": round(raw[3], 5),
        "right_corner": round(raw[4], 5),
        "raw_mean": round(sum(raw) / 5.0, 5),
        "active_sum": round(active_sum, 5),
        "line_position": round(position, 5),
        "error": round(error, 5),
        "pid": round(pid, 5),
        "left_motor": round(left * _turbo_factor, 5),
        "right_motor": round(right * _turbo_factor, 5),
        "node_now": int(_last_node_now),
        "node_event": int(_last_node_event),
        "strong_count": _last_strong_count,
        "start_node_armed": int(_start_node_armed),
        "start_clear_frames": _start_clear_frames,
        "state_node_count": _state_node_count,
        "state_frame": _state_frame_count,
        "blue_drop_armed": int(_blue_drop_armed),
        "blue_drop_frames_from_marker": (
            _state_frame_count - _blue_drop_marker_state_frame
            if (
                _blue_drop_armed
                and _blue_drop_marker_state_frame >= 0
            )
            else -1
        ),
        "blue_drop_due_now": int(
            _blue_drop_armed
            and _blue_drop_marker_state_frame >= 0
            and (
                _state_frame_count - _blue_drop_marker_state_frame
                >= BLUE_DROP_DELAY_FRAMES_AFTER_MARKER
            )
            and (
                _state_frame_count - _blue_drop_marker_state_frame
                <= BLUE_DROP_FRAME_WINDOW_END
            )
        ),
        "expected_next_action": (
            "RIGHT_FROM_BLUE_DROP"
            if _route_state == BLUE_AFTER_DROP
            else (
                "FOLLOW_PID_IGNORE_NODE_1_2_THEN_NODE_3_LEFT"
                if _state_node_count < 2
                else "WHITE_NODE_3_LEFT_THEN_FOLLOW_PID"
            )
            if _route_state == BLUE_RETURN_WHITE
            else "BLACK_NODE_1_IGNORE_NODE_2_LEFT"
            if _route_state == RETURN_TO_CIRCLE
            else "CIRCLE_ENTRY_LEFT"
            if _route_state == RED_CONNECTOR
            else "PICK_RED_COMPLETE_CIRCLE_EXIT_LEFT"
            if _route_state == RED_CIRCLE
            else "MAIN_PATH_RIGHT"
            if _route_state == RED_EXIT_TO_MAIN
            else ""
        ),
        "manoeuvre": _manoeuvre_kind or "",
        "manoeuvre_phase": _last_manoeuvre_phase,
        "manoeuvre_frame": _manoeuvre_frame_count,
        "manoeuvre_elapsed": round(_last_manoeuvre_elapsed, 5),
        "sensor_mask": "".join(
            "1" if value >= NODE_ACTIVE_THRESHOLD else "0"
            for value in _active_values(raw)
        ),
        "base_left_motor": round(left, 5),
        "base_right_motor": round(right, 5),
        "turbo_factor": round(_turbo_factor, 3),
        "virtual_elapsed_s": round(_turbo_now() - _start_time, 4),
        "proximity": round(float(sensors.get("proximity", 1.0)), 5),
        "color_r": round(float(sensors.get("color_r", 0.0)), 5),
        "color_g": round(float(sensors.get("color_g", 0.0)), 5),
        "color_b": round(float(sensors.get("color_b", 0.0)), 5),
        "detected_color": _last_detected_colour or "",
        "carrying_inferred": _last_carrying or "",
        "pick_requested": int(_pick_requested),
        "drop_requested": int(_drop_requested),
    }

    try:
        _csv_writer.writerow(row)
        if _frame_number % 5 == 0 or _event_note:
            _csv_file.flush()
    except OSError as exc:
        print(f"[CSV] Write failed: {exc}")

    _event_note = ""
    _pick_requested = False
    _drop_requested = False


def _control_loop_base(sensors):
    """Return (left_speed, right_speed) for the current sensor reading.

    This function combines:
      1. explicit route states for the required junction decisions,
      2. fixed line-colour modes instead of searching for both lines everywhere,
      3. weighted-average PID line following, and
      4. CSV logging for frame-by-frame correction.
    """
    global _previous_error, _integral, _last_pid_time
    global _last_valid_error, _lost_frames
    global _last_position, _last_error, _last_pid
    global _finish_counter, _blue_entry_handoff_until
    global _blue_exit_handoff_until, _blue_exit_handoff_center_frames
    global _main_entry_handoff_until, _main_entry_handoff_center_frames
    global _red_entry_handoff_until
    global _red_exit_handoff_until, _red_exit_handoff_center_frames
    global _red_main_handoff_until, _red_main_handoff_center_frames
    global _finish_right_handoff_until
    global _finish_right_handoff_center_frames
    global _blue_return_right_handoff_frames
    global _blue_return_left_handoff_frames
    global _state_frame_count

    _state_frame_count += 1
    now = _turbo_now()
    raw = _raw_values(sensors)
    raw_mean = sum(raw) / 5.0

    _switch_line_mode_if_needed(raw, now)

    active = _active_values(raw)
    position, active_sum = _line_position(active)
    node_event = _detect_node(active, now)

    if node_event:
        _handle_node_event(now, raw)

    # Stop permanently after the final white rectangle is confirmed.
    if (
        _route_state == FINISH_ROUTE
        and _manoeuvre_kind is None
        and now - _state_enter_time > 1.30
    ):
        if min(raw) > 0.78 and raw_mean > 0.86:
            _finish_counter += 1
        else:
            _finish_counter = 0

        if _finish_counter >= 1:
            _set_state(FINISHED, "final white rectangle detected")

    if _route_state == FINISHED:
        left = 0.0
        right = 0.0
        _log_csv(sensors, raw, active_sum, 0.0, 0.0, 0.0, left, right)
        return left, right

    if _route_state == RED_WAIT_AT_WHITE_JUNCTION:
        left = 0.0
        right = 0.0
        _log_csv(sensors, raw, active_sum, 0.0, 0.0, 0.0, left, right)
        return left, right

    # Legacy timed release retained only for compatibility with old files.
    # The current BLUE return route uses blue_return_left_black_lock and never
    # releases until the centred black line is physically detected.
    if (
        _route_state == BLUE_TO_BLACK
        and _manoeuvre_kind == "white_left_gentle"
        and now - _manoeuvre_start >= RETURN_WHITE_LEFT_RELEASE_SECONDS
    ):
        position_now, _ = _line_position(active)
        error_now = -position_now if position_now is not None else 0.0
        _previous_error = error_now
        _integral = 0.0
        _last_pid_time = now
        _finish_manoeuvre(
            "Return LEFT branch acquired; release to surface-transition PID"
        )

    manoeuvre = _manoeuvre_command(now, active, raw)
    if manoeuvre is not None:
        left, right = manoeuvre
        _log_csv(
            sensors, raw, active_sum,
            _last_position, _last_error, _last_pid,
            left, right,
        )
        return left, right

    # Post-BLUE-drop continuation: the required RIGHT is already complete.
    # From here there is no open-loop movement and no node-based turn until
    # BLUE_RETURN_WHITE node 3. Use only capped PID to follow the continuous
    # white line through the first two ignored nodes.
    if (
        _route_state == BLUE_RETURN_WHITE
        and _blue_return_right_handoff_frames > 0
    ):
        _last_manoeuvre_phase = (
            "blue_return_follow_line_pid_handoff"
        )

        if position is None:
            error = _last_valid_error
            correction = 0.0
        else:
            error = -position
            requested = (
                BLUE_RETURN_RIGHT_HANDOFF_GAIN * error
            )
            correction = max(
                -BLUE_RETURN_RIGHT_HANDOFF_MAX_CORRECTION,
                min(
                    BLUE_RETURN_RIGHT_HANDOFF_MAX_CORRECTION,
                    requested,
                ),
            )
            _last_valid_error = error
            _last_position = position
            _last_error = error

        left = (
            BLUE_RETURN_RIGHT_HANDOFF_BASE_SPEED
            - correction
        )
        right = (
            BLUE_RETURN_RIGHT_HANDOFF_BASE_SPEED
            + correction
        )

        # Forward-only PID continuation.
        left = max(0.82, min(2.10, left))
        right = max(0.82, min(2.10, right))

        _previous_error = error
        _integral = 0.0
        _last_pid_time = now
        _last_pid = correction

        _blue_return_right_handoff_frames -= 1

        if _blue_return_right_handoff_frames == 0:
            _add_event(
                "BLUE return PID handoff complete; "
                "continue normal PID and ignore nodes 1/2"
            )

        _log_csv(
            sensors, raw, active_sum,
            _last_position, _last_error, _last_pid,
            left, right,
        )
        return left, right

    # Third white junction LEFT is selected. From this point use sensor PID,
    # not another open-loop turn, to follow the curved white branch.
    if (
        _route_state == BLUE_TO_BLACK
        and _line_mode == "white"
        and _blue_return_left_handoff_frames > 0
    ):
        _last_manoeuvre_phase = (
            "blue_return_left_follow_white_pid"
        )

        if position is None:
            error = _last_valid_error
        else:
            error = -position
            _last_valid_error = error
            _last_position = position
            _last_error = error

        requested = BLUE_RETURN_LEFT_HANDOFF_GAIN * error
        correction = max(
            -BLUE_RETURN_LEFT_HANDOFF_MAX_CORRECTION,
            min(
                BLUE_RETURN_LEFT_HANDOFF_MAX_CORRECTION,
                requested,
            ),
        )

        left = BLUE_RETURN_LEFT_HANDOFF_BASE_SPEED - correction
        right = BLUE_RETURN_LEFT_HANDOFF_BASE_SPEED + correction

        left = max(0.76, min(2.10, left))
        right = max(0.76, min(2.10, right))

        _previous_error = error
        _integral = 0.0
        _last_pid_time = now
        _last_pid = correction

        _blue_return_left_handoff_frames -= 1

        if _blue_return_left_handoff_frames == 0:
            _add_event(
                "Third-junction LEFT PID handoff complete; "
                "continue normal white PID until black transition"
            )

        _log_csv(
            sensors, raw, active_sum,
            _last_position, _last_error, _last_pid,
            left, right,
        )
        return left, right

    # Final branch is already centred when this handoff starts.
    # During this short capture window, the robot may move straight or make a
    # tiny additional RIGHT correction only. It may not turn LEFT and undo the
    # successful branch selection.
    if (
        _route_state == FINISH_ROUTE
        and now < _finish_right_handoff_until
    ):
        _last_manoeuvre_phase = "finish_right_straight_pid_capture"

        if position is None:
            error = 0.0
            correction = 0.0
            _finish_right_handoff_center_frames = 0
        else:
            error = -position

            # In this motor convention:
            #   negative correction -> left motor faster -> RIGHT
            #   positive correction -> LEFT (blocked during handoff)
            requested = 0.35 * error
            correction = max(
                -FINISH_RIGHT_HANDOFF_MAX_RIGHT_CORRECTION,
                min(0.0, requested),
            )

            _last_valid_error = error
            _last_position = position
            _last_error = error

            line_centred = (
                abs(position) <= 0.42
                and active[2] >= 0.42
                and active_sum <= 2.85
            )
            if line_centred:
                _finish_right_handoff_center_frames += 1
            else:
                _finish_right_handoff_center_frames = 0

        left = FINISH_RIGHT_HANDOFF_BASE_SPEED - correction
        right = FINISH_RIGHT_HANDOFF_BASE_SPEED + correction
        left = max(0.95, min(1.75, left))
        right = max(0.95, min(1.75, right))

        _previous_error = error
        _integral = 0.0
        _last_pid_time = now
        _last_pid = correction

        if (
            _finish_right_handoff_center_frames
            >= FINISH_RIGHT_HANDOFF_CENTER_FRAMES
        ):
            _finish_right_handoff_until = now
            _finish_right_handoff_center_frames = 0
            _previous_error = error
            _integral = 0.0
            _last_pid_time = now
            _add_event(
                "Finish branch stable; normal white PID enabled"
            )

        _log_csv(
            sensors, raw, active_sum,
            _last_position, _last_error, _last_pid,
            left, right,
        )
        return left, right

    # RED circle-entry handoff: limited PID after the mirrored LEFT turn.
    if _route_state == RED_CIRCLE and now < _red_entry_handoff_until:
        _last_manoeuvre_phase = "red_circle_line_handoff"

        if position is None:
            error = _last_valid_error
            correction = 0.0
        else:
            error = -position
            correction = max(
                -RED_ENTRY_HANDOFF_MAX_CORRECTION,
                min(RED_ENTRY_HANDOFF_MAX_CORRECTION, 0.55 * error),
            )
            _last_valid_error = error
            _last_position = position
            _last_error = error

        left = RED_ENTRY_HANDOFF_BASE_SPEED - correction
        right = RED_ENTRY_HANDOFF_BASE_SPEED + correction
        left = max(0.85, min(2.20, left))
        right = max(0.85, min(2.20, right))

        _previous_error = error
        _integral = 0.0
        _last_pid_time = now
        _last_pid = correction

        _log_csv(
            sensors, raw, active_sum,
            _last_position, _last_error, _last_pid,
            left, right,
        )
        return left, right

    # After the RED circle-exit LEFT turn, do not let PID immediately undo it.
    # Allow straight motion or a small additional LEFT correction only.
    if _route_state == RED_EXIT_TO_MAIN and now < _red_exit_handoff_until:
        _last_manoeuvre_phase = "red_exit_connector_capture"

        if position is None:
            error = _last_valid_error
            correction = 0.0
            _red_exit_handoff_center_frames = 0
        else:
            error = -position
            requested = 0.45 * error

            # correction > 0 produces a LEFT turn; correction < 0 would undo it.
            correction = max(
                0.0,
                min(RED_EXIT_HANDOFF_MAX_CORRECTION, requested),
            )

            _last_valid_error = error
            _last_position = position
            _last_error = error

            connector_centred = (
                abs(position) <= 0.38
                and active[2] >= 0.52
                and active_sum <= 2.85
            )
            if connector_centred:
                _red_exit_handoff_center_frames += 1
            else:
                _red_exit_handoff_center_frames = 0

        left = RED_EXIT_HANDOFF_BASE_SPEED - correction
        right = RED_EXIT_HANDOFF_BASE_SPEED + correction
        left = max(1.05, min(2.10, left))
        right = max(1.05, min(2.10, right))

        _previous_error = error
        _integral = 0.0
        _last_pid_time = now
        _last_pid = correction

        if _red_exit_handoff_center_frames >= 2:
            _red_exit_handoff_until = now
            _red_exit_handoff_center_frames = 0
            _previous_error = error
            _integral = 0.0
            _last_pid_time = now
            _add_event(
                "RED exit connector stable; normal black PID enabled"
            )

        _log_csv(
            sensors, raw, active_sum,
            _last_position, _last_error, _last_pid,
            left, right,
        )
        return left, right

    # RED connector-to-main-path handoff, kept separate from the locked BLUE one.
    if _route_state == RED_TO_WHITE and now < _red_main_handoff_until:
        _last_manoeuvre_phase = "red_main_black_pid_handoff"

        if position is None:
            error = _last_valid_error
            correction = max(-0.18, min(0.18, 0.25 * error))
            _red_main_handoff_center_frames = 0
        else:
            error = -position
            correction = max(
                -MAIN_ENTRY_PID_HANDOFF_MAX_CORRECTION,
                min(MAIN_ENTRY_PID_HANDOFF_MAX_CORRECTION, 0.55 * error),
            )
            _last_valid_error = error
            _last_position = position
            _last_error = error

            if abs(position) <= 0.38 and active[2] >= 0.42:
                _red_main_handoff_center_frames += 1
            else:
                _red_main_handoff_center_frames = 0

        left = MAIN_ENTRY_PID_HANDOFF_BASE - correction
        right = MAIN_ENTRY_PID_HANDOFF_BASE + correction
        left = max(0.90, min(1.95, left))
        right = max(0.90, min(1.95, right))

        _previous_error = error
        _integral = 0.0
        _last_pid_time = now
        _last_pid = correction

        if _red_main_handoff_center_frames >= 3:
            _red_main_handoff_until = now
            _red_main_handoff_center_frames = 0
            _previous_error = error
            _integral = 0.0
            _last_pid_time = now
            _add_event(
                "RED main-path black line stable; normal PID enabled"
            )

        _log_csv(
            sensors, raw, active_sum,
            _last_position, _last_error, _last_pid,
            left, right,
        )
        return left, right

    # Fourth-node turn completed: follow the newly acquired BLACK main-path line.
    # This is genuine sensor-based line following, not another fixed junction
    # movement. Both correction directions are allowed, but the correction is
    # limited so the robot cannot jerk away from the branch.
    if _route_state == BLUE_TO_WHITE and now < _main_entry_handoff_until:
        _last_manoeuvre_phase = "main_entry_black_pid_handoff"

        if position is None:
            error = _last_valid_error
            correction = max(
                -0.18,
                min(0.18, 0.25 * error),
            )
            _main_entry_handoff_center_frames = 0
        else:
            error = -position
            correction = max(
                -MAIN_ENTRY_PID_HANDOFF_MAX_CORRECTION,
                min(
                    MAIN_ENTRY_PID_HANDOFF_MAX_CORRECTION,
                    0.55 * error,
                ),
            )

            _last_valid_error = error
            _last_position = position
            _last_error = error

            # In black-line mode, active[2] means the black line is under the
            # middle sensor. End the gentle handoff only after stable centring.
            if abs(position) <= 0.38 and active[2] >= 0.42:
                _main_entry_handoff_center_frames += 1
            else:
                _main_entry_handoff_center_frames = 0

        left = MAIN_ENTRY_PID_HANDOFF_BASE - correction
        right = MAIN_ENTRY_PID_HANDOFF_BASE + correction
        left = max(0.90, min(1.95, left))
        right = max(0.90, min(1.95, right))

        _previous_error = error
        _integral = 0.0
        _last_pid_time = now
        _last_pid = correction

        if _main_entry_handoff_center_frames >= 3:
            _main_entry_handoff_until = now
            _main_entry_handoff_center_frames = 0
            _previous_error = error
            _integral = 0.0
            _last_pid_time = now
            _add_event(
                "Fourth-node black line stable; normal black PID enabled"
            )

        _log_csv(
            sensors, raw, active_sum,
            _last_position, _last_error, _last_pid,
            left, right,
        )
        return left, right

    # After the THIRD-node right turn, do not allow the controller to undo the
    # turn. CSV 17 showed an immediate unwanted LEFT command:
    #     left=1.20, right=1.90
    # followed by an even stronger PID left correction.
    #
    # During this capture window the robot may:
    #   1. continue STRAIGHT, or
    #   2. make a small additional RIGHT correction.
    # It may NOT turn left. Once the connector is centred for two frames, normal
    # PID is enabled with its history reset.
    if _route_state == BLUE_EXIT_TO_MAIN and now < _blue_exit_handoff_until:
        _last_manoeuvre_phase = "blue_exit_straight_line_capture"

        if position is None:
            error = _last_valid_error
            correction = 0.0
            _blue_exit_handoff_center_frames = 0
        else:
            error = -position

            # In this motor convention:
            #   correction > 0 -> left motor slower -> LEFT turn (forbidden here)
            #   correction < 0 -> left motor faster -> RIGHT turn (allowed)
            requested = 0.45 * error
            correction = max(
                -BLUE_EXIT_HANDOFF_MAX_CORRECTION,
                min(0.0, requested),
            )

            _last_valid_error = error
            _last_position = position
            _last_error = error

            connector_centred = (
                abs(position) <= 0.38
                and active[2] >= 0.52
                and active_sum <= 2.85
            )
            if connector_centred:
                _blue_exit_handoff_center_frames += 1
            else:
                _blue_exit_handoff_center_frames = 0

        left = BLUE_EXIT_HANDOFF_BASE_SPEED - correction
        right = BLUE_EXIT_HANDOFF_BASE_SPEED + correction
        left = max(1.05, min(2.10, left))
        right = max(1.05, min(2.10, right))

        _previous_error = error
        _integral = 0.0
        _last_pid_time = now
        _last_pid = correction

        if _blue_exit_handoff_center_frames >= 2:
            _blue_exit_handoff_until = now
            _blue_exit_handoff_center_frames = 0
            _previous_error = 0.0
            _integral = 0.0
            _last_pid_time = now
            _add_event(
                "BLUE exit connector stable; normal PID enabled without left kick"
            )

        _log_csv(
            sensors, raw, active_sum,
            _last_position, _last_error, _last_pid,
            left, right,
        )
        return left, right

    # Smoothly continue on the circle line after the CSV-13 turn. This is not a
    # second turn or junction action. It is only a limited PID ramp that prevents
    # the abrupt reverse-wheel correction seen at frame 1817.
    if _route_state == BLUE_CIRCLE and now < _blue_entry_handoff_until:
        _last_manoeuvre_phase = "blue_continue_line_handoff"

        if position is None:
            error = _last_valid_error
            correction = 0.0
        else:
            error = -position
            correction = max(
                -BLUE_ENTRY_HANDOFF_MAX_CORRECTION,
                min(BLUE_ENTRY_HANDOFF_MAX_CORRECTION, 0.55 * error),
            )
            _last_valid_error = error
            _last_position = position
            _last_error = error

        left = BLUE_ENTRY_HANDOFF_BASE_SPEED - correction
        right = BLUE_ENTRY_HANDOFF_BASE_SPEED + correction
        left = max(0.85, min(2.20, left))
        right = max(0.85, min(2.20, right))

        # Keep PID history aligned so normal PID begins without derivative kick.
        _previous_error = error
        _integral = 0.0
        _last_pid_time = now
        _last_pid = correction

        _log_csv(
            sensors, raw, active_sum,
            _last_position, _last_error, _last_pid,
            left, right,
        )
        return left, right

    # The sensors are touching the edge of the SECOND highlighted black spot.
    # Move slowly straight until all five sensors centre over the spot; only then
    # will _detect_node() start the right turn.  This removes run-to-run timing
    # dependence and keeps the already-working first main-path turn unchanged.
    if _route_state == BLUE_CONNECTOR and _circle_entry_approach:
        left = 1.05
        right = 1.05
        _log_csv(
            sensors, raw, active_sum,
            _last_position, _last_error, _last_pid,
            left, right,
        )
        return left, right

    # BLUE main-path black dot: all five sensors seeing black means a wide,
    # centred section of the same route. No turn and no special manoeuvre.
    # Reset PID history only to remove the derivative kick caused by the sudden
    # change from a narrow line to a wide black dot.
    blue_main_dot_strong_count = sum(
        value >= NODE_ACTIVE_THRESHOLD for value in active
    )
    blue_main_dot_all_black = (
        _route_state == BLUE_TO_WHITE
        and _line_mode == "black"
        and blue_main_dot_strong_count == 5
        and active_sum >= 4.45
        and _manoeuvre_kind is None
    )

    if blue_main_dot_all_black:
        position = 0.0
        _last_valid_error = 0.0
        _previous_error = 0.0
        _integral = 0.0
        _last_pid_time = now

    # Short gaps/dashed line: preserve heading first, then search in the direction
    # where the line was last observed.
    if position is None:
        _lost_frames += 1
        if _lost_frames <= 7:
            correction = max(-0.55, min(0.55, -_last_valid_error * 0.30))
            left = 1.42 - correction
            right = 1.42 + correction
        elif _last_valid_error >= 0.0:
            left, right = 0.45, 1.45
        else:
            left, right = 1.45, 0.45

        _log_csv(
            sensors, raw, active_sum,
            _last_position, _last_error, _last_pid,
            left, right,
        )
        return left, right

    _lost_frames = 0
    _last_valid_error = -position

    # The setpoint is zero. A negative position means the line is left, so error
    # becomes positive and the right motor is made faster.
    error = -position
    dt = max(0.02, min(0.20, now - _last_pid_time))
    _last_pid_time = now

    _integral += error * dt
    _integral = max(-1.5, min(1.5, _integral))
    derivative = error - _previous_error

    pid = KP * error + KI * _integral + KD * derivative
    _previous_error = error

    base = BASE_SPEED - 0.40 * min(abs(error), 2.0)
    base = max(MIN_BASE_SPEED, base)

    left = base - pid
    right = base + pid
    left = max(-0.65, min(MAX_MOTOR_SPEED, left))
    right = max(-0.65, min(MAX_MOTOR_SPEED, right))

    _last_position = position
    _last_error = error
    _last_pid = pid

    _log_csv(sensors, raw, active_sum, position, error, pid, left, right)
    return left, right



def control_loop(sensors):
    """Run the locked route with adaptive motor scaling and normal timers."""
    global _turbo_factor

    factor = _choose_turbo_factor(sensors)
    _turbo_factor = factor
    _advance_turbo_clock(factor)

    left, right = _control_loop_base(sensors)

    if _route_state == FINISHED:
        return 0.0, 0.0

    scaled_left = max(
        -TURBO_MAX_ABS_MOTOR,
        min(TURBO_MAX_ABS_MOTOR, left * factor),
    )
    scaled_right = max(
        -TURBO_MAX_ABS_MOTOR,
        min(TURBO_MAX_ABS_MOTOR, right * factor),
    )

    return scaled_left, scaled_right

def detect_color(sensors):
    """Identify a confidently dominant red or blue sensor reading."""
    global _last_detected_colour

    red = float(sensors.get("color_r", 0.0))
    green = float(sensors.get("color_g", 0.0))
    blue = float(sensors.get("color_b", 0.0))

    colour = None
    if (
        red >= 0.16
        and red >= green * 1.25
        and red >= blue * 1.25
        and red - max(green, blue) >= 0.035
    ):
        colour = "red"
    elif (
        blue >= 0.16
        and blue >= green * 1.20
        and blue >= red * 1.20
        and blue - max(green, red) >= 0.035
    ):
        colour = "blue"

    _last_detected_colour = colour
    return colour


def should_pick(sensors, carrying_color):
    """Pick BLUE on the first circle visit and RED on the second visit."""
    global _pick_cooldown_until, _pick_colour_frames, _pick_requested

    _sync_carrying(carrying_color)
    _pick_requested = False

    if carrying_color is not None:
        _pick_colour_frames = 0
        return False

    expected_colour = None
    if _route_state == BLUE_CIRCLE and not _blue_picked:
        expected_colour = "blue"
    elif _route_state == RED_CIRCLE and not _red_picked:
        expected_colour = "red"
    else:
        _pick_colour_frames = 0
        return False

    now = _turbo_now()
    proximity = float(sensors.get("proximity", 1.0))
    colour = detect_color(sensors)

    if (
        0.0 < proximity < PROXIMITY_PICK_THRESHOLD
        and colour == expected_colour
    ):
        _pick_colour_frames += 1
    else:
        _pick_colour_frames = 0

    if _pick_colour_frames >= 1 and now >= _pick_cooldown_until:
        _pick_cooldown_until = now + 1.0
        _pick_colour_frames = 0
        _pick_requested = True
        _add_event(
            f"PICK request: expected={expected_colour}, proximity={proximity:.3f}"
        )
        return True

    return False


def should_drop(sensors, carrying_color):
    """Drop only in the matching coloured zone on the white-line section."""
    global _drop_cooldown_until, _drop_colour_frames, _drop_requested
    global _state_frame_count

    _sync_carrying(carrying_color)
    _drop_requested = False

    if carrying_color is None or _line_mode != "white":
        _drop_colour_frames = 0
        return False

    if carrying_color == "blue" and _route_state == BLUE_WHITE_TO_DROP:
        expected_colour = "blue"
    elif carrying_color == "red" and _route_state == RED_WHITE_TO_DROP:
        expected_colour = "red"
    else:
        _drop_colour_frames = 0
        return False

    now = _turbo_now()
    colour = detect_color(sensors)
    proximity = float(sensors.get("proximity", 1.0))

    if expected_colour == "blue":
        # CSV 22:
        #   node 6 -> blue rectangle = about 5.578 seconds.
        # The carried blue box itself keeps proximity near 0.1406 and makes the
        # colour sensor read blue, so proximity > 0.42 was an impossible gate.
        #
        # Use both:
        #   1. exact route marker + measured travel time, and
        #   2. stable centred white-line readings at the rectangle.
        middle = float(sensors.get("middle", 0.0))
        left_corner = float(sensors.get("left_corner", 0.0))
        left_sensor = float(sensors.get("left", 0.0))
        right_sensor = float(sensors.get("right", 0.0))
        right_corner = float(sensors.get("right_corner", 0.0))

        marker_elapsed = (
            now - _blue_drop_marker_time
            if _blue_drop_armed and _blue_drop_marker_time > 0.0
            else -1.0
        )
        marker_frames = (
            _state_frame_count - _blue_drop_marker_state_frame
            if (
                _blue_drop_armed
                and _blue_drop_marker_state_frame >= 0
            )
            else -1
        )

        centred_white_line = (
            middle >= BLUE_DROP_LINE_MIDDLE_MIN
            and left_corner <= BLUE_DROP_SIDE_MAX
            and left_sensor <= BLUE_DROP_SIDE_MAX
            and right_sensor <= BLUE_DROP_SIDE_MAX
            and right_corner <= BLUE_DROP_SIDE_MAX
        )

        still_holding_blue = (
            colour == "blue"
            and 0.0 < proximity < PROXIMITY_PICK_THRESHOLD
        )

        exact_drop_frame_reached = (
            marker_frames >= BLUE_DROP_DELAY_FRAMES_AFTER_MARKER
            and marker_frames <= BLUE_DROP_FRAME_WINDOW_END
        )

        zone_candidate = (
            _blue_drop_armed
            and _route_state == BLUE_WHITE_TO_DROP
            and exact_drop_frame_reached
            and still_holding_blue
        )
    else:
        # The carried RED box keeps both the colour sensor and proximity active,
        # exactly like the carried BLUE box. Therefore proximity > 0.42 cannot
        # identify the drop zone.
        middle = float(sensors.get("middle", 0.0))
        left_corner = float(sensors.get("left_corner", 0.0))
        left_sensor = float(sensors.get("left", 0.0))
        right_sensor = float(sensors.get("right", 0.0))
        right_corner = float(sensors.get("right_corner", 0.0))

        marker_elapsed = (
            now - _red_drop_marker_time
            if _red_drop_armed and _red_drop_marker_time > 0.0
            else -1.0
        )

        centred_white_line = (
            middle >= RED_DROP_LINE_MIDDLE_MIN
            and left_corner <= RED_DROP_SIDE_MAX
            and left_sensor <= RED_DROP_SIDE_MAX
            and right_sensor <= RED_DROP_SIDE_MAX
            and right_corner <= RED_DROP_SIDE_MAX
        )

        still_holding_red = (
            colour == "red"
            and 0.0 < proximity < PROXIMITY_PICK_THRESHOLD
        )

        zone_candidate = (
            _red_drop_armed
            and _state_node_count >= RED_DROP_MARKER_NODE
            and marker_elapsed >= RED_DROP_DELAY_AFTER_NODE
            and centred_white_line
            and still_holding_red
        )

    if zone_candidate:
        _drop_colour_frames += 1
    else:
        _drop_colour_frames = 0

    required_drop_frames = (
        BLUE_DROP_STABLE_FRAMES
        if expected_colour == "blue"
        else RED_DROP_STABLE_FRAMES
    )

    if (
        _drop_colour_frames >= required_drop_frames
        and now >= _drop_cooldown_until
    ):
        _drop_cooldown_until = now + 1.2
        _drop_colour_frames = 0
        _drop_requested = True

        if expected_colour == "blue":
            marker_elapsed = now - _blue_drop_marker_time
            marker_frames = (
                _state_frame_count
                - _blue_drop_marker_state_frame
            )
            _add_event(
                "BLUE DROP request at rectangle: "
                f"exact relative frame {marker_frames} "
                f"after last white node"
            )
        else:
            marker_elapsed = now - _red_drop_marker_time
            _add_event(
                "RED DROP request at rectangle: "
                f"{marker_elapsed:.2f}s after white node 3"
            )
        return True

    return False

# =============================================================================
#  Main loop (Don't Edit this)
# =============================================================================
def main():
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to bridge_v1_2b. Running... (Ctrl+C to stop)")

    last_sensors   = None
    carrying_color = None   # colour of the box currently held, or None
    delivered      = 0      # number of boxes released so far

    try:
        while True:
            sensors = client.receive_sensor_data()
            if sensors is not None:
                last_sensors = sensors
            if last_sensors is None:
                time.sleep(0.02)
                continue

            # --- Pick (empty-handed only) ---
            if carrying_color is None and should_pick(last_sensors, carrying_color):
                colour_seen = detect_color(last_sensors)     # read BEFORE picking
                success = client.send_pick()
                print(f"PICK attempted (saw {colour_seen!r}) — success={success}")
                if success:
                    carrying_color = colour_seen

            # --- Drop (only while carrying) ---
            if carrying_color is not None and should_drop(last_sensors, carrying_color):
                success = client.send_drop()
                print(f"DROP attempted ({carrying_color!r}) — success={success}")
                if success:
                    delivered += 1
                    carrying_color = None
                    print(f"Delivered {delivered} box(es) so far.")

            # --- Motor command ---
            left, right = control_loop(last_sensors)
            client.send_motor_command(left, right)

            time.sleep(0.05)   # ~20 Hz control loop

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            client.send_motor_command(0.0, 0.0)
        except Exception:
            pass
        client.close()


if __name__ == "__main__":
    main()
