"""
apriltag_auto.py - EDUCATIONAL EXAMPLE: Trench Run + Goal Shot
================================================================

+-----------------------------------------------------------------------+
|                                                                       |
|   THIS IS A TEACHING EXAMPLE — NOT THE REAL COMPETITION AUTO!         |
|                                                                       |
|   This file shows students HOW to build an autonomous mode.           |
|   It demonstrates every technique you need, but it does NOT           |
|   actually grab balls or shoot them (the scorer isn't wired in).      |
|   The robot just drives through the motions so you can see how        |
|   each piece works.                                                   |
|                                                                       |
|   YOUR JOB: Use this as a reference to build the REAL competition     |
|   auto mode. Copy the techniques, adjust the distances and speeds     |
|   for YOUR robot on the REAL field, and wire in the real scorer.      |
|                                                                       |
+-----------------------------------------------------------------------+

WHAT THIS EXAMPLE DOES:
  The robot drives through a trench, turns toward the goal, finds the
  goal AprilTag, and aligns in front of it. No balls are collected or
  shot — it's just showing the movement and vision patterns.

WHAT THE REAL COMPETITION AUTO SHOULD DO:
  1. Actually grab balls (intake running during trench drive)
  2. Actually shoot (scorer fires when aligned to goal)
  3. Use tuned distances/speeds measured on the real field
  4. Handle edge cases (what if we miss a tag? what if we hit a wall?)
  5. Maybe collect from multiple spots and shoot multiple times

===========================================================================
STEP-BY-STEP GUIDE: HOW TO BUILD YOUR OWN AUTONOMOUS MODE
===========================================================================

  STEP 1: CREATE A NEW FILE
  --------------------------
  Create a new .py file in the autonomous/ folder. MagicBot will
  automatically discover it. Give your class a unique MODE_NAME.

  STEP 2: SET UP MAGICBOT INJECTION
  ----------------------------------
  Declare the components you need as class-level type annotations:

      swerve_drive: SwerveDrive
      thunder_viking_super_scorer: ThunderVikingSuperScorer

  MagicBot will automatically give you access to these components.
  The variable names must EXACTLY match what's in robot.py.

  STEP 3: DETECT YOUR ALLIANCE
  -----------------------------
  In on_enable(), call:

      alliance = wpilib.DriverStation.getAlliance()

  This tells you Red or Blue, which determines:
    - Which AprilTag IDs to look for (the field is mirrored)
    - Which direction to drive (Red and Blue start on opposite sides)

  STEP 4: PLAN YOUR ROUTE
  -------------------------
  Open the field layout JSON (resources/2026-rebuilt-welded.json) and
  find the positions of the tags you care about. Plan an L-shaped or
  curved route from your starting position to each target.

  Use this math to figure out drive times:
      distance = speed × time
      time = distance / speed

  Example: To drive 4.5 meters at 3.0 m/s → time = 4.5 / 3.0 = 1.5 sec

  STEP 5: BUILD A STATE MACHINE
  ------------------------------
  Break your route into steps (states). Each state does ONE thing:
    - Drive in a direction for a set time
    - Turn a specific angle
    - Scan for a tag
    - Align to a tag
    - Intake / shoot

  Use a timer to control how long each state lasts. When the timer
  runs out (or you reach your goal), move to the next state.

  STEP 6: USE VISION FOR PRECISION
  ----------------------------------
  Time-based driving gets you CLOSE, but AprilTags get you PRECISE.
  Use time-based driving to get near the target, then use vision
  alignment to lock on perfectly.

  Limelight NetworkTables values:
    tv  = 1 if a tag is visible, 0 if not
    tid = the tag ID number
    tx  = degrees the tag is left/right of center (+ = right, - = left)
    botpose_wpiblue = estimated robot position on the field

  STEP 7: TUNE ON THE REAL ROBOT
  --------------------------------
  The distances and speeds in this example are GUESSES for simulation.
  On the real robot you MUST:
    - Measure actual distances on the field with a tape measure
    - Test each state individually and adjust times
    - Tune the kP value for alignment (start at 0.02, go up slowly)
    - Add safety timeouts to every state so the robot doesn't get stuck

===========================================================================
FIELD LAYOUT REFERENCE (2026)
===========================================================================

  Blue alliance:
    - Trench tag = 28 at (4.588, 0.644)  ← bottom-left area
    - Goal tag   = 26 at (4.022, 4.035)  ← center-left area

  Red alliance:
    - Trench tag =  7 at (11.953, 0.644) ← bottom-right area
    - Goal tag   = 10 at (12.519, 4.035) ← center-right area

  The trench runs along the bottom edge of the field (y ≈ 0.6m).
  The goal is in the center of the field (y ≈ 4.0m).
  Route: drive along the bottom → turn up → go to goal.
  Full field is 16.54m long (x-axis) × 8.21m wide (y-axis).

===========================================================================
STATE MACHINE OVERVIEW
===========================================================================

  INIT → DRIVE_TO_TRENCH → CHECK_TRENCH_TAG → DRIVE_THROUGH_TRENCH
       → TURN_TO_GOAL → DRIVE_TO_GOAL → FIND_GOAL_TAG → ALIGN_TO_GOAL
       → SHOOT_STUB → DONE

  Total time budget: ~14 seconds (well under the 20-second limit)

  Timing breakdown:
    INIT:                 instant
    DRIVE_TO_TRENCH:      1.5s  (3.0 m/s × 1.5s = 4.5m)
    CHECK_TRENCH_TAG:     1.0s  (slow drive + vision check)
    DRIVE_THROUGH_TRENCH: 1.5s  (2.5 m/s × 1.5s = 3.75m)
    TURN_TO_GOAL:         0.6s  (90° turn at 2.5 rad/s)
    DRIVE_TO_GOAL:        1.2s  (3.0 m/s × 1.2s = 3.6m)
    FIND_GOAL_TAG:        up to 3.0s (scan for tag)
    ALIGN_TO_GOAL:        up to 2.5s (proportional alignment)
    SHOOT_STUB:           2.0s  (wait for shot)
    DONE:                 sit still

===========================================================================
"""

import math
import wpilib
from ntcore import NetworkTableInstance

from components.swerve_drive import SwerveDrive
import constants


# =============================================================================
# APRIL TAG IDS FOR EACH ALLIANCE
# =============================================================================
# These come from the 2026 field layout JSON (resources/2026-rebuilt-welded.json).
# Each alliance has its own set of tags because the field is mirrored.
#
# TO FIND TAG IDS FOR YOUR GAME:
# 1. Open resources/2026-rebuilt-welded.json
# 2. Find the tags near your target (goal, trench, etc.)
# 3. Note the ID and position (x, y coordinates in meters)
# 4. Put them here so your auto code knows what to look for

BLUE_TRENCH_TAG = 28    # Tag at the Blue trench entrance (4.588, 0.644)
BLUE_GOAL_TAG = 26      # Tag at the Blue goal (4.022, 4.035)
RED_TRENCH_TAG = 7      # Tag at the Red trench entrance (11.953, 0.644)
RED_GOAL_TAG = 10       # Tag at the Red goal (12.519, 4.035)


class AprilTagDemoAuto:
    """
    EDUCATIONAL EXAMPLE autonomous mode.

    This is NOT the competition auto — it's a teaching tool that shows
    students every technique needed to build a real autonomous mode.
    The robot drives through the motions but does not actually intake
    or shoot (those are stubs for when the scorer is wired in).
    """

    MODE_NAME = "Trench Run Auto (EXAMPLE)"
    DEFAULT = True

    # =========================================================================
    # MAGICBOT INJECTION — HOW YOUR AUTO GETS ACCESS TO ROBOT COMPONENTS
    # =========================================================================
    # By declaring this type annotation, MagicBot will automatically set
    # self.swerve_drive to the SwerveDrive instance from robot.py.
    #
    # RULE: The variable name here must EXACTLY match what's in robot.py.
    # In robot.py:  swerve_drive: SwerveDrive
    # In this file: swerve_drive: SwerveDrive  ← same name = auto-injected
    swerve_drive: SwerveDrive

    # =========================================================================
    # HOW TO ADD SCORER ACCESS (for the real competition auto)
    # =========================================================================
    # When ThunderVikingSuperScorer is declared in robot.py, add this line
    # to get automatic access to it in your autonomous mode:
    #
    #     thunder_viking_super_scorer: ThunderVikingSuperScorer
    #
    # Then you can call these methods in your auto code:
    #     self.thunder_viking_super_scorer.intake()            # deploy intake + run rollers
    #     self.thunder_viking_super_scorer.prepare_to_shoot()  # spin up shooter wheels
    #     self.thunder_viking_super_scorer.shoot()             # fire! (hopper feeds ball in)
    #
    # You also need the import at the top of the file:
    #     from components.thunder_viking_super_scorer import ThunderVikingSuperScorer

    def __init__(self):
        # Timer — we use this to control how long each state lasts
        # Call timer.restart() when entering a new state to reset it to 0
        self.timer = wpilib.Timer()

        # State machine — tracks which step of the auto we're on
        self.state = "INIT"

        # Alliance — set in on_enable() based on Driver Station
        self.alliance = "BLUE"

        # Tag IDs — set in on_enable() based on alliance
        self.trench_tag_id = BLUE_TRENCH_TAG
        self.goal_tag_id = BLUE_GOAL_TAG

        # Limelight NetworkTables — same table whether sim or real robot
        # In sim, physics.py publishes fake data to this table.
        # On a real robot, the actual Limelight camera publishes to it.
        self.ll = NetworkTableInstance.getDefault().getTable("limelight")

    # =========================================================================
    # on_enable() — RUNS ONCE when autonomous starts
    # =========================================================================
    # This is where you do one-time setup: detect alliance, reset state,
    # zero the gyro, etc. This runs at the very start of the match.
    def on_enable(self):
        """Called once at the start of autonomous (match clock starts)."""

        self.timer.restart()
        self.state = "INIT"

        # -----------------------------------------------------------------
        # TECHNIQUE: ALLIANCE DETECTION
        # -----------------------------------------------------------------
        # The Driver Station tells us which side of the field we're on.
        # This is CRITICAL because:
        #   - The field is mirrored — Red and Blue have different tag IDs
        #   - Starting positions are on opposite ends of the field
        #   - Drive directions may need to be flipped
        #
        # In simulation, getAlliance() may return None (no DS connected),
        # so we default to Blue. On a real field, this always works.
        alliance = wpilib.DriverStation.getAlliance()
        if alliance == wpilib.DriverStation.Alliance.kRed:
            self.alliance = "RED"
            self.trench_tag_id = RED_TRENCH_TAG
            self.goal_tag_id = RED_GOAL_TAG
        else:
            self.alliance = "BLUE"
            self.trench_tag_id = BLUE_TRENCH_TAG
            self.goal_tag_id = BLUE_GOAL_TAG

        print(f"[Auto] === TRENCH RUN AUTO (EXAMPLE) ===")
        print(f"[Auto] Alliance: {self.alliance}")
        print(f"[Auto] Looking for trench tag: {self.trench_tag_id}, goal tag: {self.goal_tag_id}")
        print(f"[Auto] NOTE: This is a teaching example — no balls will be collected or shot")

    # =========================================================================
    # on_iteration() — RUNS EVERY 20ms during autonomous
    # =========================================================================
    # This is the main loop. It runs 50 times per second for the entire
    # autonomous period. Each call checks which state we're in and does
    # the appropriate action.
    def on_iteration(self, time_elapsed: float):
        """Called every 20ms during autonomous mode."""

        t = self.timer.get()

        # Post to SmartDashboard so you can see what's happening in real time
        wpilib.SmartDashboard.putString("auto/state", self.state)
        wpilib.SmartDashboard.putNumber("auto/timer", t)

        # =================================================================
        # STATE: INIT (runs for one cycle only, then immediately moves on)
        # =================================================================
        # WHAT:  Zero the gyroscope
        # WHY:   The gyro must be zeroed so "0 degrees" matches the
        #        direction the robot is currently facing. Without this,
        #        field-relative driving will go the wrong direction.
        # WHEN:  Always do this first in any autonomous mode.
        if self.state == "INIT":
            self.swerve_drive.zero_heading()
            self.state = "DRIVE_TO_TRENCH"
            self.timer.restart()
            print("[Auto] Gyro zeroed — driving to trench!")

        # =================================================================
        # STATE: DRIVE_TO_TRENCH
        # =================================================================
        # WHAT:  Drive forward at high speed toward the trench area.
        # WHY:   The trench has balls we want to collect. We need to get
        #        there as fast as possible (auto is only 20 seconds!).
        #
        # TECHNIQUE: SPEED × TIME = DISTANCE
        # ----------------------------------
        # This is the simplest way to drive a specific distance.
        # It's not perfectly accurate (wheels can slip, battery voltage
        # varies), but it gets you CLOSE enough. You use AprilTags
        # later for precise positioning.
        #
        #   We want: ~4.5 meters forward
        #   Speed:   3.0 m/s
        #   Time:    4.5m / 3.0 m/s = 1.5 seconds
        #
        # HOW set_drive_command() WORKS:
        #   The speed values are FRACTIONS of kMaxSpeed (-1.0 to 1.0).
        #   For 3.0 m/s when kMaxSpeed = 4.0 → fraction = 3.0/4.0 = 0.75
        #
        # FIELD RELATIVE vs ROBOT RELATIVE:
        #   field_relative=True  → "forward" = toward far wall (uses gyro)
        #   field_relative=False → "forward" = wherever robot is pointing
        #   Use True for driving in straight lines across the field.
        #   Use False for spinning in place or robot-relative movements.
        #
        # FOR YOUR REAL AUTO:
        #   - Measure the actual distance on the field with a tape measure
        #   - Test and adjust the time until the robot stops in the right spot
        #   - Add self.thunder_viking_super_scorer.intake() here to grab balls!
        elif self.state == "DRIVE_TO_TRENCH":
            drive_speed = 3.0       # meters per second (fast!)
            strafe_speed = -0.3     # slight rightward drift to line up with trench
            drive_time = 1.5        # seconds
            # Result: ~4.5 meters forward, ~0.45 meters to the right

            self.swerve_drive.set_drive_command(
                drive_speed / constants.kMaxSpeed,       # x = forward
                strafe_speed / constants.kMaxSpeed,      # y = strafe (negative = right)
                0.0,                                     # rot = no rotation
                True,                                    # field_relative = True
                False,                                   # rate_limit = False (instant response)
            )

            # ---------------------------------------------------------
            # FOR YOUR REAL AUTO: Run the intake while driving through
            # the trench so you collect balls along the way:
            #
            # self.thunder_viking_super_scorer.intake()
            # ---------------------------------------------------------

            if t > drive_time:
                self.state = "CHECK_TRENCH_TAG"
                self.timer.restart()
                dist = drive_speed * drive_time
                print(f"[Auto] Drove ~{dist:.1f}m toward trench, checking for tag {self.trench_tag_id}")

        # =================================================================
        # STATE: CHECK_TRENCH_TAG
        # =================================================================
        # WHAT:  Slow down and look for the trench AprilTag.
        # WHY:   To confirm we actually made it to the trench area.
        #        Time-based driving is approximate — the tag confirms
        #        our real position on the field.
        #
        # TECHNIQUE: READING LIMELIGHT / APRILTAG DATA
        # ---------------------------------------------
        # The Limelight camera (or our simulated one) publishes data
        # to NetworkTables. We read it like this:
        #
        #   tv  = self.ll.getNumber("tv", 0)     ← 1 = tag visible, 0 = not
        #   tid = self.ll.getNumber("tid", -1)    ← which tag ID
        #   tx  = self.ll.getNumber("tx", 0.0)    ← degrees off-center
        #
        # The second argument (0, -1, 0.0) is the DEFAULT value returned
        # if the key doesn't exist yet (e.g. before the camera starts).
        #
        # FOR YOUR REAL AUTO:
        #   - You could use botpose_wpiblue to get your exact field position
        #   - Use that position to calculate exactly how far to drive next
        elif self.state == "CHECK_TRENCH_TAG":
            check_time = 1.0   # look for 1 second while slowly driving

            # Keep driving slowly while checking vision
            self.swerve_drive.set_drive_command(
                1.0 / constants.kMaxSpeed,   # 1 m/s forward (slow)
                0.0,
                0.0,
                True,
                False,
            )

            # Read the Limelight data
            tv = self.ll.getNumber("tv", 0)
            if tv >= 1:
                tid = int(self.ll.getNumber("tid", -1))
                tx = self.ll.getNumber("tx", 0.0)
                if tid == self.trench_tag_id:
                    print(f"[Auto] Confirmed trench tag {tid}! tx={tx:.1f}° — we're in the right spot")
                else:
                    print(f"[Auto] Saw tag {tid} (not trench tag {self.trench_tag_id}), continuing anyway")

            # Move on after the check time, whether or not we saw the tag
            # (time-based driving should have gotten us close enough)
            if t > check_time:
                self.state = "DRIVE_THROUGH_TRENCH"
                self.timer.restart()

        # =================================================================
        # STATE: DRIVE_THROUGH_TRENCH
        # =================================================================
        # WHAT:  Continue driving through the trench to grab more balls.
        # WHY:   The trench has multiple balls spread along its length.
        #
        # distance = 2.5 m/s × 1.5s = 3.75 meters
        #
        # FOR YOUR REAL AUTO:
        #   - Keep the intake running: self.thunder_viking_super_scorer.intake()
        #   - Adjust speed based on how well the intake picks up balls
        #   - Slower = more reliable intake, faster = more ground covered
        elif self.state == "DRIVE_THROUGH_TRENCH":
            drive_speed = 2.5   # slightly slower for ball collection
            drive_time = 1.5

            self.swerve_drive.set_drive_command(
                drive_speed / constants.kMaxSpeed,
                0.0,
                0.0,
                True,
                False,
            )

            # ---------------------------------------------------------
            # FOR YOUR REAL AUTO: Keep intaking!
            #
            # self.thunder_viking_super_scorer.intake()
            # ---------------------------------------------------------

            if t > drive_time:
                self.state = "TURN_TO_GOAL"
                self.timer.restart()
                print("[Auto] Exiting trench, turning toward the goal!")

        # =================================================================
        # STATE: TURN_TO_GOAL
        # =================================================================
        # WHAT:  Rotate ~90° to face toward the goal.
        # WHY:   The goal is "above" us on the field (higher y value).
        #        We were driving along the bottom (y ≈ 0.6), and the goal
        #        is at y ≈ 4.0, so we need to turn to face that direction.
        #
        # TECHNIQUE: TIMED ROTATION
        # --------------------------
        # For turning a specific angle:
        #   time = angle_in_radians / angular_speed_in_radians_per_second
        #
        #   90° = π/2 radians = 1.571 radians
        #   At 2.5 rad/s: time = 1.571 / 2.5 = 0.63 seconds
        #
        # We use field_relative=False for rotation because we want to
        # spin the robot in place regardless of field orientation.
        #
        # FOR YOUR REAL AUTO:
        #   - The turn angle might not be exactly 90°, measure it
        #   - You could also use the gyro to turn to an exact heading
        #     instead of using timed rotation (more accurate)
        #   - Example: while get_heading() < 90: keep rotating
        elif self.state == "TURN_TO_GOAL":
            turn_speed_radps = 2.5                      # how fast to spin
            turn_angle_rad = math.pi / 2                # 90 degrees
            turn_time = turn_angle_rad / turn_speed_radps  # ≈ 0.63 seconds

            rot_fraction = turn_speed_radps / constants.kMaxAngularSpeed

            self.swerve_drive.set_drive_command(
                0.0,          # no forward/backward
                0.0,          # no strafe
                rot_fraction, # rotate counter-clockwise (positive = CCW)
                False,        # robot-relative (spin in place)
                False,        # no rate limiting
            )

            if t > turn_time:
                self.state = "DRIVE_TO_GOAL"
                self.timer.restart()
                print("[Auto] Turn complete, driving toward goal area!")

        # =================================================================
        # STATE: DRIVE_TO_GOAL
        # =================================================================
        # WHAT:  Drive toward the goal area.
        # WHY:   We need to get close enough to see the goal AprilTag
        #        and be within shooting range.
        #
        # The goal is ~3.4 meters from the trench (y: 0.6 → 4.0).
        # We drive at 3.0 m/s for 1.2s = 3.6 meters.
        #
        # We use field_relative=True with strafing (+y direction) because
        # after the turn, our "forward" might not perfectly point at the goal.
        # Field-relative lets us drive in the +y direction on the field
        # regardless of which way the robot is actually facing.
        #
        # FOR YOUR REAL AUTO:
        #   - You could start spinning up the shooter wheels here to
        #     save time: self.thunder_viking_super_scorer.prepare_to_shoot()
        elif self.state == "DRIVE_TO_GOAL":
            drive_speed = 3.0
            drive_time = 1.2

            # Drive in the +y field direction (toward the goal)
            self.swerve_drive.set_drive_command(
                0.0,                                # no forward (field x)
                drive_speed / constants.kMaxSpeed,  # strafe toward goal (field +y)
                0.0,                                # no rotation
                True,                               # field-relative
                False,                              # no rate limiting
            )

            if t > drive_time:
                self.state = "FIND_GOAL_TAG"
                self.timer.restart()
                print(f"[Auto] Near goal area, scanning for goal tag {self.goal_tag_id}...")

        # =================================================================
        # STATE: FIND_GOAL_TAG
        # =================================================================
        # WHAT:  Rotate slowly to find the goal AprilTag.
        # WHY:   We might not be facing the goal perfectly after driving.
        #        Scanning (slow rotation) sweeps the camera across the
        #        field until we see the right tag.
        #
        # TECHNIQUE: TAG FILTERING
        # -------------------------
        # We don't just look for ANY tag — we specifically check if
        # tid matches our goal_tag_id. The field has many tags, and
        # we only care about the one in front of our goal.
        #
        # FOR YOUR REAL AUTO:
        #   - Start the shooter spin-up here so wheels are ready:
        #     self.thunder_viking_super_scorer.prepare_to_shoot()
        #   - If the scan times out, you have options:
        #     a) Try driving a bit and scanning again
        #     b) Shoot anyway (might miss but still scores sometimes)
        #     c) Go collect more balls and try again
        elif self.state == "FIND_GOAL_TAG":
            scan_timeout = 3.0    # give up after 3 seconds
            scan_speed = 0.2      # slow rotation to scan

            self.swerve_drive.set_drive_command(
                0.0,
                0.0,
                scan_speed,   # slow counter-clockwise rotation
                False,        # robot-relative (spin in place)
                False,
            )

            # Read vision data
            tv = self.ll.getNumber("tv", 0)
            if tv >= 1:
                tid = int(self.ll.getNumber("tid", -1))
                tx = self.ll.getNumber("tx", 0.0)

                # Only move to alignment if we see OUR goal tag
                if tid == self.goal_tag_id:
                    print(f"[Auto] Found GOAL tag {tid}! tx={tx:.1f}° — aligning!")
                    self.state = "ALIGN_TO_GOAL"
                    self.timer.restart()
                # If we see a different tag, keep scanning

            # Safety timeout — don't spin forever
            if t > scan_timeout:
                print(f"[Auto] Could not find goal tag {self.goal_tag_id}, attempting shoot anyway")
                self.state = "SHOOT_STUB"
                self.timer.restart()

        # =================================================================
        # STATE: ALIGN_TO_GOAL
        # =================================================================
        # WHAT:  Precisely rotate until the goal tag is centered in the camera.
        # WHY:   For accurate shooting, the robot needs to face the goal
        #        dead-on. The AprilTag tells us exactly how far off we are.
        #
        # TECHNIQUE: PROPORTIONAL CONTROL (P-controller)
        # -----------------------------------------------
        # This is one of the most important concepts in robotics!
        #
        # tx = degrees the tag is off-center:
        #   tx > 0 → tag is to the RIGHT → rotate CLOCKWISE
        #   tx < 0 → tag is to the LEFT  → rotate COUNTER-CLOCKWISE
        #
        # We calculate: rotation = -tx × kP
        #
        # WHAT IS kP (proportional gain)?
        #   It controls how aggressively we correct:
        #   kP too LOW  (0.01) → robot aligns slowly, might not finish in time
        #   kP too HIGH (0.1)  → robot overshoots, oscillates back and forth
        #   kP just right (0.02-0.04) → smooth, fast alignment
        #
        # TUNING kP ON THE REAL ROBOT:
        #   1. Start with kP = 0.02
        #   2. Run the auto and watch the robot align
        #   3. If it's too slow, increase kP by 0.005
        #   4. If it overshoots (wobbles back and forth), decrease kP
        #   5. You want it to settle quickly without oscillating
        #
        # We also CAP the rotation speed so it doesn't whip around
        # dangerously. max_rot = 0.4 means max 40% rotation speed.
        #
        # FOR YOUR REAL AUTO:
        #   - Keep shooter wheels spinning: self.thunder_viking_super_scorer.prepare_to_shoot()
        #   - Consider also driving forward/backward to get to the
        #     ideal shooting distance (use tag distance or botpose)
        elif self.state == "ALIGN_TO_GOAL":
            align_timeout = 2.5    # don't spend more than 2.5s aligning
            kP = 0.03              # proportional gain (tune this!)
            tx_tolerance = 1.5     # degrees — "close enough" to shoot
            max_rot = 0.4          # cap rotation at 40% speed

            tv = self.ll.getNumber("tv", 0)
            tid = int(self.ll.getNumber("tid", -1))

            if tv >= 1 and tid == self.goal_tag_id:
                tx = self.ll.getNumber("tx", 0.0)

                # THE P-CONTROLLER: rotation = -tx × kP
                rot_command = -tx * kP

                # Cap it so the robot doesn't spin too fast
                rot_command = max(-max_rot, min(max_rot, rot_command))

                self.swerve_drive.set_drive_command(
                    0.0,
                    0.0,
                    rot_command,    # proportional rotation
                    False,          # robot-relative
                    False,
                )

                # Check if we're close enough to center
                if abs(tx) < tx_tolerance:
                    print(f"[Auto] ALIGNED to goal! tx={tx:.1f}° — ready to shoot!")
                    self.state = "SHOOT_STUB"
                    self.timer.restart()
            else:
                # Lost sight of the tag — stop and wait (it might come back)
                self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, False, False)

            # Safety timeout
            if t > align_timeout:
                print("[Auto] Alignment timed out, shooting anyway!")
                self.state = "SHOOT_STUB"
                self.timer.restart()

        # =================================================================
        # STATE: SHOOT_STUB
        # =================================================================
        # WHAT:  This is where you would fire the balls.
        # WHY:   We're aligned to the goal — time to score!
        #
        # THIS IS A STUB — no balls are actually shot because the
        # scorer isn't wired into robot.py yet. The commented code
        # below shows exactly what you'd call in the real auto.
        #
        # TECHNIQUE: SHOOTING SEQUENCE
        # ------------------------------
        # 1. Call prepare_to_shoot() every cycle to keep wheels spinning
        # 2. Check shooter.is_at_speed — are wheels fast enough?
        # 3. If yes, call shoot() to push the ball into the wheels
        # 4. Wait a moment for the ball to leave, then you can shoot again
        #
        # FOR YOUR REAL AUTO:
        #   - You need the ThunderVikingSuperScorer wired into robot.py
        #   - Uncomment the injection line at the top of this class
        #   - Uncomment the scoring commands below
        #   - After shooting, you could drive back to collect more balls
        #     and repeat the whole sequence!
        elif self.state == "SHOOT_STUB":
            shoot_time = 2.0  # how long to "shoot" (time for all balls to fire)

            # Stop the robot while shooting
            self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, False, False)

            # ---------------------------------------------------------
            # REAL SCORING COMMANDS (uncomment when scorer is wired in)
            # ---------------------------------------------------------
            #
            # # Keep shooter wheels spinning
            # self.thunder_viking_super_scorer.prepare_to_shoot()
            #
            # # When wheels are fast enough, fire!
            # if self.thunder_viking_super_scorer.shooter.is_at_speed:
            #     self.thunder_viking_super_scorer.shoot()
            #     print("[Auto] BALL LAUNCHED!")
            #
            # ---------------------------------------------------------

            if t < 0.1:
                print("[Auto]")
                print("[Auto] *** SHOOT STUB — THIS IS WHERE YOU WOULD SCORE ***")
                print("[Auto] The scorer is not wired in yet.")
                print("[Auto] See the commented code above for what to call.")
                print("[Auto]")

            if t > shoot_time:
                self.state = "DONE"
                self.timer.restart()
                print("[Auto] Autonomous complete!")

        # =================================================================
        # STATE: DONE
        # =================================================================
        # WHAT:  Stop the robot and wait for autonomous to end.
        # WHY:   Don't keep driving after you're done — the robot
        #        should sit still until teleop starts.
        #
        # FOR YOUR REAL AUTO:
        #   - You could use remaining time to collect more balls
        #   - Or drive to a strategic starting position for teleop
        elif self.state == "DONE":
            self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, False, False)

    # =========================================================================
    # on_disable() — RUNS ONCE when autonomous ends
    # =========================================================================
    # Always stop the robot here as a safety measure.
    def on_disable(self):
        """Called when autonomous mode ends."""
        self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, False, False)
        print("[Auto] Autonomous mode ended")
