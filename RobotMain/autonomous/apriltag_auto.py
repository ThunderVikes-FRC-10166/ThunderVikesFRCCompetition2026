"""
apriltag_auto.py - Trench Run + Goal Shot Autonomous Mode
============================================================

This is the "real game" autonomous mode for the 2026 season.

WHAT THE ROBOT DOES (20 seconds max):
  1. Start against the alliance wall, facing the field
  2. Drive fast down the trench to collect balls
  3. See the trench AprilTag to confirm position
  4. Continue through the trench a bit
  5. Turn toward the goal and drive up to it
  6. Find the goal AprilTag and align precisely in front of it
  7. Shoot!

FIELD LAYOUT (2026):
  Blue alliance:
    - Trench tag = 28 at (4.588, 0.644)  ← bottom-left area
    - Goal tag   = 26 at (4.022, 4.035)  ← center-left area
  Red alliance:
    - Trench tag =  7 at (11.953, 0.644) ← bottom-right area
    - Goal tag   = 10 at (12.519, 4.035) ← center-right area

  The trench runs along the bottom edge (y ≈ 0.6m).
  The goal is in the center (y ≈ 4.0m).
  So the route is: drive along the bottom → turn up → go to goal.

TEACHING CONCEPTS:
  - Alliance detection (red vs blue changes which tags we look for)
  - Speed × time = distance (how to drive a specific distance)
  - AprilTag reading from NetworkTables (tv, tid, tx)
  - Proportional control for alignment (rotate until tx ≈ 0)
  - Scorer command stubs (intake, shoot — for when scorer is wired in)

HOW MAGICBOT INJECTION WORKS:
  In robot.py we have:   swerve_drive: SwerveDrive
  In this file we have:  swerve_drive: SwerveDrive
  MagicBot automatically connects them — no manual wiring needed!

STATE MACHINE:
  INIT → DRIVE_TO_TRENCH → CHECK_TRENCH_TAG → DRIVE_THROUGH_TRENCH
       → TURN_TO_GOAL → DRIVE_TO_GOAL → FIND_GOAL_TAG → ALIGN_TO_GOAL
       → SHOOT_STUB → DONE
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
# Each alliance has its own set of tags. Change these if the field layout changes.

BLUE_TRENCH_TAG = 28
BLUE_GOAL_TAG = 26
RED_TRENCH_TAG = 7
RED_GOAL_TAG = 10


class AprilTagDemoAuto:
    MODE_NAME = "Trench Run Auto"
    DEFAULT = True

    # =========================================================================
    # MAGICBOT INJECTION
    # =========================================================================
    # MagicBot sees this type annotation and fills in the swerve_drive
    # component from robot.py automatically. The name must EXACTLY match.
    swerve_drive: SwerveDrive

    # =========================================================================
    # FUTURE: SCORING SYSTEM INJECTION
    # =========================================================================
    # When ThunderVikingSuperScorer is wired into robot.py, uncomment this:
    #
    # from components.thunder_viking_super_scorer import ThunderVikingSuperScorer
    # thunder_viking_super_scorer: ThunderVikingSuperScorer

    def __init__(self):
        self.timer = wpilib.Timer()
        self.state = "INIT"
        self.alliance = "BLUE"

        self.trench_tag_id = BLUE_TRENCH_TAG
        self.goal_tag_id = BLUE_GOAL_TAG

        self.ll = NetworkTableInstance.getDefault().getTable("limelight")

    # =========================================================================
    # on_enable — runs ONCE when autonomous starts
    # =========================================================================
    def on_enable(self):
        """Called once at the start of autonomous (match begins)."""

        self.timer.restart()
        self.state = "INIT"

        # -----------------------------------------------------------------
        # ALLIANCE DETECTION
        # -----------------------------------------------------------------
        # The Driver Station tells us Red or Blue. This changes which
        # AprilTags we look for (the field is mirrored).
        #
        # In sim, getAlliance() may return None, so we default to Blue.
        alliance = wpilib.DriverStation.getAlliance()
        if alliance == wpilib.DriverStation.Alliance.kRed:
            self.alliance = "RED"
            self.trench_tag_id = RED_TRENCH_TAG
            self.goal_tag_id = RED_GOAL_TAG
        else:
            self.alliance = "BLUE"
            self.trench_tag_id = BLUE_TRENCH_TAG
            self.goal_tag_id = BLUE_GOAL_TAG

        print(f"[Auto] === TRENCH RUN AUTO ===")
        print(f"[Auto] Alliance: {self.alliance}")
        print(f"[Auto] Trench tag: {self.trench_tag_id}, Goal tag: {self.goal_tag_id}")

    # =========================================================================
    # on_iteration — runs every 20ms during autonomous
    # =========================================================================
    def on_iteration(self, time_elapsed: float):
        """Called every 20ms during autonomous mode."""

        t = self.timer.get()

        # Post current state to SmartDashboard so you can see what's happening
        wpilib.SmartDashboard.putString("auto/state", self.state)
        wpilib.SmartDashboard.putNumber("auto/timer", t)

        # =================================================================
        # STATE: INIT (instant — one cycle only)
        # =================================================================
        # Zero the gyroscope so the robot's current facing direction
        # becomes "0 degrees". This is critical for field-relative driving.
        if self.state == "INIT":
            self.swerve_drive.zero_heading()
            self.state = "DRIVE_TO_TRENCH"
            self.timer.restart()
            print("[Auto] Gyro zeroed — driving to trench!")

        # =================================================================
        # STATE: DRIVE_TO_TRENCH (fast forward drive)
        # =================================================================
        # Drive forward at high speed toward the trench area.
        #
        # THE MATH: distance = speed × time
        #   We want to cover about 4.5 meters to reach the trench.
        #   At 3.0 m/s for 1.5 seconds: 3.0 × 1.5 = 4.5 meters
        #
        # set_drive_command takes a FRACTION of kMaxSpeed (-1.0 to 1.0).
        # So for 3.0 m/s with kMaxSpeed = 4.0: fraction = 3.0 / 4.0 = 0.75
        #
        # We also strafe slightly toward y = 0.6 (the trench y-position)
        # to line up with the trench entrance.
        #
        # field_relative=True: "forward" stays toward the far wall even if
        # the robot rotates. This uses the gyroscope.
        elif self.state == "DRIVE_TO_TRENCH":
            drive_speed = 3.0
            strafe_speed = -0.3
            drive_time = 1.5
            # distance forward = 3.0 m/s × 1.5s = 4.5 meters
            # slight rightward drift to line up with trench

            self.swerve_drive.set_drive_command(
                drive_speed / constants.kMaxSpeed,
                strafe_speed / constants.kMaxSpeed,
                0.0,
                True,
                False,
            )

            # ---------------------------------------------------------
            # INTAKE STUB: In a real match, you'd run the intake here
            # to pick up balls as you drive through the trench.
            #
            # self.thunder_viking_super_scorer.intake()
            # ---------------------------------------------------------

            if t > drive_time:
                self.state = "CHECK_TRENCH_TAG"
                self.timer.restart()
                dist = drive_speed * drive_time
                print(f"[Auto] Drove ~{dist:.1f}m toward trench, checking for tag {self.trench_tag_id}")

        # =================================================================
        # STATE: CHECK_TRENCH_TAG (brief pause to read vision)
        # =================================================================
        # Slow down and check if we can see the trench AprilTag.
        # This confirms we're in the right area of the field.
        #
        # We keep driving slowly forward while checking. If we see the
        # trench tag, great — we know where we are. If not, we continue
        # anyway (the time-based driving should have gotten us close).
        #
        # HOW TO READ LIMELIGHT DATA:
        #   tv  = 1 if any tag is visible, 0 if not
        #   tid = which tag ID is visible
        #   tx  = horizontal angle offset in degrees (+ = right, - = left)
        elif self.state == "CHECK_TRENCH_TAG":
            check_time = 1.0

            self.swerve_drive.set_drive_command(
                1.0 / constants.kMaxSpeed,
                0.0,
                0.0,
                True,
                False,
            )

            tv = self.ll.getNumber("tv", 0)
            if tv >= 1:
                tid = int(self.ll.getNumber("tid", -1))
                tx = self.ll.getNumber("tx", 0.0)
                if tid == self.trench_tag_id:
                    print(f"[Auto] Confirmed trench tag {tid}! tx={tx:.1f}° — we're in the right spot")
                else:
                    print(f"[Auto] Saw tag {tid} (not trench tag {self.trench_tag_id}), continuing anyway")

            if t > check_time:
                self.state = "DRIVE_THROUGH_TRENCH"
                self.timer.restart()

        # =================================================================
        # STATE: DRIVE_THROUGH_TRENCH (continue forward to collect balls)
        # =================================================================
        # Keep driving forward through the trench to grab more balls.
        #
        # distance = 2.5 m/s × 1.5s = 3.75 meters through the trench
        elif self.state == "DRIVE_THROUGH_TRENCH":
            drive_speed = 2.5
            drive_time = 1.5

            self.swerve_drive.set_drive_command(
                drive_speed / constants.kMaxSpeed,
                0.0,
                0.0,
                True,
                False,
            )

            # ---------------------------------------------------------
            # INTAKE STUB: Still intaking balls as we drive through
            #
            # self.thunder_viking_super_scorer.intake()
            # ---------------------------------------------------------

            if t > drive_time:
                self.state = "TURN_TO_GOAL"
                self.timer.restart()
                print("[Auto] Exiting trench, turning toward the goal!")

        # =================================================================
        # STATE: TURN_TO_GOAL (rotate to face the goal)
        # =================================================================
        # The goal is "above" us on the field (higher y value).
        # We need to turn left (counter-clockwise) roughly 90 degrees
        # to face toward the goal area.
        #
        # We rotate at a specific speed for a calculated time:
        #   For a 90° turn at 2.5 rad/s:
        #   time = angle / speed = (π/2) / 2.5 = 0.63 seconds
        #
        # field_relative=False: rotation is relative to the robot,
        # not the field. We spin in place.
        elif self.state == "TURN_TO_GOAL":
            turn_speed_radps = 2.5
            turn_angle_rad = math.pi / 2
            turn_time = turn_angle_rad / turn_speed_radps

            rot_fraction = turn_speed_radps / constants.kMaxAngularSpeed

            self.swerve_drive.set_drive_command(
                0.0,
                0.0,
                rot_fraction,
                False,
                False,
            )

            if t > turn_time:
                self.state = "DRIVE_TO_GOAL"
                self.timer.restart()
                print("[Auto] Turn complete, driving toward goal area!")

        # =================================================================
        # STATE: DRIVE_TO_GOAL (drive toward the goal)
        # =================================================================
        # The goal is about 3.4 meters away (y goes from ~0.6 to ~4.0).
        # We also need to adjust our x position to line up.
        #
        # Since we just turned ~90° left, our "forward" now points
        # toward the goal (in the +y direction on the field).
        #
        # distance = 3.0 m/s × 1.2s = 3.6 meters toward goal
        #
        # We use field_relative=True here so "forward" is toward +y
        # (the direction we need to go) regardless of small rotation errors.
        elif self.state == "DRIVE_TO_GOAL":
            drive_speed = 3.0
            drive_time = 1.2

            self.swerve_drive.set_drive_command(
                0.0,
                drive_speed / constants.kMaxSpeed,
                0.0,
                True,
                False,
            )

            if t > drive_time:
                self.state = "FIND_GOAL_TAG"
                self.timer.restart()
                print(f"[Auto] Near goal area, scanning for goal tag {self.goal_tag_id}...")

        # =================================================================
        # STATE: FIND_GOAL_TAG (scan for the goal AprilTag)
        # =================================================================
        # Rotate slowly to find the goal AprilTag. Once we see it, we
        # move to precise alignment.
        #
        # We specifically look for our goal tag ID — if we see a different
        # tag, we keep scanning.
        elif self.state == "FIND_GOAL_TAG":
            scan_timeout = 3.0
            scan_speed = 0.2

            self.swerve_drive.set_drive_command(
                0.0,
                0.0,
                scan_speed,
                False,
                False,
            )

            tv = self.ll.getNumber("tv", 0)
            if tv >= 1:
                tid = int(self.ll.getNumber("tid", -1))
                tx = self.ll.getNumber("tx", 0.0)

                if tid == self.goal_tag_id:
                    print(f"[Auto] Found GOAL tag {tid}! tx={tx:.1f}° — aligning!")
                    self.state = "ALIGN_TO_GOAL"
                    self.timer.restart()
                else:
                    pass

            if t > scan_timeout:
                print(f"[Auto] Could not find goal tag {self.goal_tag_id}, attempting shoot anyway")
                self.state = "SHOOT_STUB"
                self.timer.restart()

            # ---------------------------------------------------------
            # SHOOTER PREP STUB: Start spinning up shooter wheels
            # while we're still aligning so they're ready when we are.
            #
            # self.thunder_viking_super_scorer.prepare_to_shoot()
            # ---------------------------------------------------------

        # =================================================================
        # STATE: ALIGN_TO_GOAL (precise alignment using AprilTag)
        # =================================================================
        # Rotate the robot until the goal tag is centered in the camera.
        #
        # HOW PROPORTIONAL CONTROL WORKS:
        #   tx = degrees the tag is off-center (+ = right, - = left)
        #   We set rotation = -tx × kP
        #   When tx is large → fast correction
        #   When tx is near 0 → tiny correction → we're aligned!
        #
        #   kP = 0.03 (proportional gain — tune this on the real robot)
        #   Higher kP = faster but might overshoot and oscillate
        #   Lower kP = smoother but slower
        #
        # We also cap the rotation speed at ±0.4 so it doesn't whip around.
        elif self.state == "ALIGN_TO_GOAL":
            align_timeout = 2.5
            kP = 0.03
            tx_tolerance = 1.5
            max_rot = 0.4

            tv = self.ll.getNumber("tv", 0)
            tid = int(self.ll.getNumber("tid", -1))

            if tv >= 1 and tid == self.goal_tag_id:
                tx = self.ll.getNumber("tx", 0.0)

                rot_command = -tx * kP
                rot_command = max(-max_rot, min(max_rot, rot_command))

                self.swerve_drive.set_drive_command(
                    0.0,
                    0.0,
                    rot_command,
                    False,
                    False,
                )

                if abs(tx) < tx_tolerance:
                    print(f"[Auto] ALIGNED to goal! tx={tx:.1f}° — SHOOTING!")
                    self.state = "SHOOT_STUB"
                    self.timer.restart()
            else:
                self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, False, False)

            # ---------------------------------------------------------
            # SHOOTER PREP STUB: Keep wheels spinning during alignment
            #
            # self.thunder_viking_super_scorer.prepare_to_shoot()
            # ---------------------------------------------------------

            if t > align_timeout:
                print("[Auto] Alignment timed out, shooting anyway!")
                self.state = "SHOOT_STUB"
                self.timer.restart()

        # =================================================================
        # STATE: SHOOT_STUB (fire the balls!)
        # =================================================================
        # This is where you command the scoring system to shoot.
        #
        # When ThunderVikingSuperScorer is wired into robot.py:
        #   1. Uncomment the injection at the top of this class
        #   2. Uncomment the scoring commands below
        #
        # The scoring sequence:
        #   prepare_to_shoot() → spins up shooter wheels (call every cycle)
        #   Wait for shooter.is_at_speed to be True
        #   shoot() → hopper pushes ball into spinning wheels
        elif self.state == "SHOOT_STUB":
            shoot_time = 2.0

            self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, False, False)

            # ---------------------------------------------------------
            # SCORING COMMANDS (uncomment when scorer is wired in)
            # ---------------------------------------------------------
            # self.thunder_viking_super_scorer.prepare_to_shoot()
            #
            # if self.thunder_viking_super_scorer.shooter.is_at_speed:
            #     self.thunder_viking_super_scorer.shoot()
            #     print("[Auto] BALL LAUNCHED!")
            # ---------------------------------------------------------

            if t < 0.1:
                print("[Auto] *** SHOOT STUB *** scorer not wired in yet")
                print("[Auto] In a real match, balls would be launched here!")

            if t > shoot_time:
                self.state = "DONE"
                self.timer.restart()
                print("[Auto] Autonomous complete!")

        # =================================================================
        # STATE: DONE
        # =================================================================
        elif self.state == "DONE":
            self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, False, False)

    # =========================================================================
    # on_disable — runs once when autonomous ends
    # =========================================================================
    def on_disable(self):
        """Called when autonomous mode ends."""
        self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, False, False)
        print("[Auto] Autonomous mode ended")
