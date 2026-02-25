"""
apriltag_auto.py - AprilTag Demo Autonomous Mode
===================================================

This autonomous mode shows students how to:
  1. Detect which alliance (red/blue) you're on
  2. Drive the robot a specific distance using speed × time
  3. Read AprilTag data from the Limelight (or simulated Limelight)
  4. Align to an AprilTag by rotating until tx ≈ 0
  5. Command the scoring system (stubs for future use)

HOW MAGICBOT AUTONOMOUS INJECTION WORKS:
=========================================
MagicBot can automatically give your autonomous mode access to robot
components. If you declare a variable with the SAME NAME as a component
in robot.py, MagicBot fills it in for you.

For example, in robot.py we have:
    swerve_drive: SwerveDrive

So in this file we declare:
    swerve_drive: SwerveDrive

And MagicBot automatically sets self.swerve_drive to the same
SwerveDrive instance that robot.py uses. No manual wiring needed!

STATE MACHINE PATTERN:
=======================
Autonomous code usually runs as a "state machine". Think of it like a
checklist — the robot does step 1, then step 2, then step 3, etc.
Each step is a "state". A timer controls how long each step lasts.

States in this auto:
  INIT → DRIVE_FORWARD → LOOK_FOR_TAG → ALIGN_TO_TAG → SHOOT_STUB → DONE
"""

import wpilib
from ntcore import NetworkTableInstance

from components.swerve_drive import SwerveDrive
import constants


class AprilTagDemoAuto:
    MODE_NAME = "AprilTag Demo Auto"
    DEFAULT = True

    # =========================================================================
    # MAGICBOT INJECTION
    # =========================================================================
    # MagicBot sees this type annotation and automatically fills in the
    # swerve_drive component from robot.py. The variable name must EXACTLY
    # match what's declared in robot.py (swerve_drive: SwerveDrive).
    swerve_drive: SwerveDrive

    # =========================================================================
    # FUTURE INJECTION - SCORING SYSTEM
    # =========================================================================
    # When the ThunderVikingSuperScorer is wired into robot.py, uncomment
    # this line and MagicBot will inject it automatically:
    #
    # from components.thunder_viking_super_scorer import ThunderVikingSuperScorer
    # thunder_viking_super_scorer: ThunderVikingSuperScorer
    #
    # Then you can call:
    #   self.thunder_viking_super_scorer.intake()
    #   self.thunder_viking_super_scorer.prepare_to_shoot()
    #   self.thunder_viking_super_scorer.shoot()

    def __init__(self):
        self.timer = wpilib.Timer()
        self.state = "INIT"
        self.alliance = "unknown"

        self.ll = NetworkTableInstance.getDefault().getTable("limelight")

    # =========================================================================
    # AUTONOMOUS LIFECYCLE
    # =========================================================================

    def on_enable(self):
        """Called once when autonomous mode starts (the match begins)."""

        self.timer.restart()
        self.state = "INIT"

        # -----------------------------------------------------------------
        # ALLIANCE DETECTION
        # -----------------------------------------------------------------
        # The Driver Station tells us which alliance we're on (Red or Blue).
        # This matters because the field is mirrored — what's on the left
        # for Blue is on the right for Red.
        #
        # In simulation, getAlliance() might return None (no DS connected),
        # so we default to Blue.
        alliance = wpilib.DriverStation.getAlliance()
        if alliance == wpilib.DriverStation.Alliance.kRed:
            self.alliance = "RED"
        elif alliance == wpilib.DriverStation.Alliance.kBlue:
            self.alliance = "BLUE"
        else:
            self.alliance = "BLUE"

        print(f"[Auto] Starting AprilTag Demo Auto — Alliance: {self.alliance}")

        # -----------------------------------------------------------------
        # HOW TO USE ALLIANCE INFO
        # -----------------------------------------------------------------
        # You can flip coordinates based on alliance. For example:
        #
        #   if self.alliance == "RED":
        #       target_x = FIELD_LENGTH - blue_target_x  # mirror X
        #   else:
        #       target_x = blue_target_x
        #
        # The 2026 field is 16.54m long (FIELD_LENGTH in constants).
        # AprilTags on the Blue side have different IDs than Red side.
        # For example, the goal on Blue might be tag 7, Red might be tag 4.

    def on_iteration(self, time_elapsed: float):
        """Called every 20ms during autonomous mode."""

        t = self.timer.get()

        # =================================================================
        # STATE: INIT
        # =================================================================
        # Reset the gyro so "forward" matches our starting direction.
        # This only takes one cycle, then we move to the next state.
        if self.state == "INIT":
            self.swerve_drive.zero_heading()
            self.state = "DRIVE_FORWARD"
            self.timer.restart()
            print("[Auto] Gyro zeroed, starting to drive forward")

        # =================================================================
        # STATE: DRIVE_FORWARD
        # =================================================================
        # Drive forward at a set speed for a set time.
        #
        # THE MATH: distance = speed × time
        #   Example: 1.5 m/s × 2.0 seconds = 3.0 meters
        #
        # We use set_drive_command() which takes values from -1.0 to 1.0
        # as a fraction of kMaxSpeed. So to drive at 1.5 m/s when
        # kMaxSpeed is 4.0 m/s, we pass 1.5/4.0 = 0.375
        #
        # IMPORTANT: field_relative=True means "forward" is always toward
        # the far end of the field (uses the gyroscope). If the robot
        # turns, it still drives the same direction on the field.
        elif self.state == "DRIVE_FORWARD":
            drive_speed_mps = 1.5
            drive_time_seconds = 2.0
            # distance = 1.5 m/s × 2.0 s = 3.0 meters forward

            speed_fraction = drive_speed_mps / constants.kMaxSpeed

            self.swerve_drive.set_drive_command(
                speed_fraction,
                0.0,
                0.0,
                True,
                False,
            )

            if t > drive_time_seconds:
                self.state = "LOOK_FOR_TAG"
                self.timer.restart()
                print(f"[Auto] Drove ~{drive_speed_mps * drive_time_seconds:.1f}m forward, now looking for AprilTags")

        # =================================================================
        # STATE: LOOK_FOR_TAG
        # =================================================================
        # Stop and look around for AprilTags. The Limelight (or simulated
        # Limelight) publishes tag data to NetworkTables.
        #
        # We slowly rotate to scan for tags. When we see one, we print
        # the tag info and move to alignment.
        #
        # KEY LIMELIGHT VALUES:
        #   tv  = 1 if a tag is visible, 0 if not
        #   tid = the AprilTag ID number (each tag on the field has a unique ID)
        #   tx  = how many degrees left/right the tag is from our camera center
        #         negative = tag is to the left, positive = to the right
        #   botpose_wpiblue = our estimated field position based on the tag
        #
        # We use field_relative=False here so the rotation is relative to
        # the robot itself, not the field. This way we spin in place
        # regardless of which direction the robot is facing on the field.
        elif self.state == "LOOK_FOR_TAG":
            scan_timeout = 4.0
            scan_rotate_speed = 0.15

            self.swerve_drive.set_drive_command(
                0.0,
                0.0,
                scan_rotate_speed,
                False,
                False,
            )

            tv = self.ll.getNumber("tv", 0)
            if tv >= 1:
                tid = int(self.ll.getNumber("tid", -1))
                tx = self.ll.getNumber("tx", 0.0)
                botpose = self.ll.getNumberArray("botpose_wpiblue", [])

                print(f"[Auto] Found AprilTag {tid}! tx={tx:.1f}° botpose={botpose}")
                self.state = "ALIGN_TO_TAG"
                self.timer.restart()

            elif t > scan_timeout:
                print("[Auto] No tags found after scanning, moving to done")
                self.state = "DONE"
                self.timer.restart()

        # =================================================================
        # STATE: ALIGN_TO_TAG
        # =================================================================
        # Rotate the robot until the tag is centered in the camera (tx ≈ 0).
        #
        # HOW ALIGNMENT WORKS:
        # tx tells us how far off-center the tag is:
        #   tx > 0 → tag is to the right → rotate clockwise (negative rot)
        #   tx < 0 → tag is to the left → rotate counter-clockwise (positive rot)
        #
        # We use a simple proportional controller: rotation = -tx * kP
        # When tx is large, we rotate fast. As tx gets closer to 0, we slow down.
        #
        # WHAT IS kP? (Proportional gain)
        # It's a tuning number that controls how aggressively we correct.
        # Too small = slow to align. Too large = overshoots and oscillates.
        # Start at 0.02 and adjust from there.
        elif self.state == "ALIGN_TO_TAG":
            align_timeout = 3.0
            kP = 0.02
            tx_tolerance = 2.0

            tv = self.ll.getNumber("tv", 0)
            if tv >= 1:
                tx = self.ll.getNumber("tx", 0.0)

                rot_command = -tx * kP
                rot_command = max(-0.3, min(0.3, rot_command))

                self.swerve_drive.set_drive_command(
                    0.0,
                    0.0,
                    rot_command,
                    False,
                    False,
                )

                if abs(tx) < tx_tolerance:
                    print(f"[Auto] Aligned to tag! tx={tx:.1f}° — ready to shoot")
                    self.state = "SHOOT_STUB"
                    self.timer.restart()
            else:
                self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, False, False)
                print("[Auto] Lost sight of tag during alignment")

            if t > align_timeout:
                print("[Auto] Alignment timed out, moving to shoot stub")
                self.state = "SHOOT_STUB"
                self.timer.restart()

        # =================================================================
        # STATE: SHOOT_STUB
        # =================================================================
        # This is where you would command the scoring system to shoot.
        # Right now it's just stubs showing HOW to do it.
        #
        # When ThunderVikingSuperScorer is wired into robot.py, you would:
        #   1. Add the injection line at the top of this class (uncomment it)
        #   2. Uncomment the scoring commands below
        #
        # The scoring sequence is:
        #   prepare_to_shoot() → spins up shooter wheels
        #   Wait until shooter is at speed
        #   shoot() → hopper pushes ball into spinning wheels
        elif self.state == "SHOOT_STUB":
            # ---------------------------------------------------------
            # SCORING COMMANDS (uncomment when scorer is wired in)
            # ---------------------------------------------------------
            # Step 1: Spin up the shooter wheels
            # self.thunder_viking_super_scorer.prepare_to_shoot()
            #
            # Step 2: Check if wheels are at speed, then shoot
            # if self.thunder_viking_super_scorer.shooter.is_at_speed:
            #     self.thunder_viking_super_scorer.shoot()
            #     print("[Auto] SHOOTING!")
            # ---------------------------------------------------------

            print("[Auto] SHOOT STUB — scorer not wired in yet")
            print("[Auto] In a real match, the ball would be launched here!")
            self.state = "DONE"
            self.timer.restart()

        # =================================================================
        # STATE: DONE
        # =================================================================
        # Stop everything. The robot sits still for the rest of auto.
        elif self.state == "DONE":
            self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, True, False)

    def on_disable(self):
        """Called when autonomous mode ends."""
        self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, True, False)
        print("[Auto] Autonomous mode ended")
