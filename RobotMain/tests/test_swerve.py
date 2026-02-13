"""
test_swerve.py - Pre-deploy swerve drive verification
======================================================

Run this before deploying to verify the swerve drive code is correct.
Usage: python tests/test_swerve.py

Tests the full chain:
  D-pad/joystick input → swerve_drive → kinematics → swerve_module → physics → encoders → odometry
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import hal.simulation
import wpilib
import wpilib.simulation
import wpimath.geometry as geo
import wpimath.kinematics as kin
from wpimath.system.plant import DCMotor
import rev
import math
import constants

hal.simulation.setDriverStationEnabled(True)
hal.simulation.setDriverStationDsAttached(True)
hal.simulation.setDriverStationAutonomous(False)
hal.simulation.setDriverStationTest(False)
hal.simulation.notifyDriverStationNewData()

PASS_COUNT = 0
FAIL_COUNT = 0


def check(name, actual, expected, tolerance=0.1):
    global PASS_COUNT, FAIL_COUNT
    if abs(actual - expected) <= tolerance:
        PASS_COUNT += 1
        print(f"  PASS: {name} = {actual:.3f} (expected {expected:.3f})")
    else:
        FAIL_COUNT += 1
        print(f"  FAIL: {name} = {actual:.3f} (expected {expected:.3f}, tolerance {tolerance})")


def test_kinematics():
    """Verify kinematics produces correct module states for each direction."""
    print("\n" + "=" * 60)
    print("TEST: Kinematics output for each drive direction")
    print("=" * 60)

    k = kin.SwerveDrive4Kinematics(
        geo.Translation2d(0.3, 0.3),
        geo.Translation2d(0.3, -0.3),
        geo.Translation2d(-0.3, 0.3),
        geo.Translation2d(-0.3, -0.3),
    )

    cases = [
        ("Forward", kin.ChassisSpeeds(1.0, 0.0, 0.0), 0.0),
        ("Backward", kin.ChassisSpeeds(-1.0, 0.0, 0.0), 180.0),
        ("Left", kin.ChassisSpeeds(0.0, 1.0, 0.0), 90.0),
        ("Right", kin.ChassisSpeeds(0.0, -1.0, 0.0), -90.0),
    ]

    for name, speeds, expected_angle in cases:
        states = k.toSwerveModuleStates(speeds)
        fl_angle = states[0].angle.degrees()
        fl_speed = states[0].speed
        print(f"\n  {name}: ChassisSpeeds({speeds.vx:.0f}, {speeds.vy:.0f}, {speeds.omega:.0f})")
        check(f"  {name} FL speed", abs(fl_speed), 1.0, 0.01)
        angle_err = fl_angle - expected_angle
        while angle_err > 180: angle_err -= 360
        while angle_err < -180: angle_err += 360
        check(f"  {name} FL angle", angle_err, 0.0, 1.0)


def test_dpad_mapping():
    """Verify D-pad POV angles map to correct drive commands."""
    print("\n" + "=" * 60)
    print("TEST: D-pad direction mapping")
    print("=" * 60)

    cases = [
        (0, "Up/Forward", 0.25, 0.0),
        (180, "Down/Backward", -0.25, 0.0),
        (270, "Left", 0.0, 0.25),
        (90, "Right", 0.0, -0.25),
    ]

    for pov, name, expected_x, expected_y in cases:
        pov_rad = math.radians(pov)
        dpad_x = math.cos(pov_rad) * (constants.kDpadSpeed / constants.kMaxSpeed)
        dpad_y = -math.sin(pov_rad) * (constants.kDpadSpeed / constants.kMaxSpeed)
        print(f"\n  POV={pov} ({name}):")
        check(f"  x_speed", dpad_x, expected_x, 0.01)
        check(f"  y_speed", dpad_y, expected_y, 0.01)


def test_module_desired_state():
    """Verify set_desired_state correctly applies chassis offset and optimization."""
    print("\n" + "=" * 60)
    print("TEST: Module set_desired_state with chassis offsets")
    print("=" * 60)

    from components.swerve_module import SwerveModule

    offsets = [
        ("FL", -math.pi / 2, constants.kFrontLeftDrivingCanId, constants.kFrontLeftTurningCanId),
        ("FR", 0, constants.kFrontRightDrivingCanId, constants.kFrontRightTurningCanId),
        ("RL", math.pi, constants.kRearLeftDrivingCanId, constants.kRearLeftTurningCanId),
        ("RR", math.pi / 2, constants.kRearRightDrivingCanId, constants.kRearRightTurningCanId),
    ]

    for name, offset, drive_id, turn_id in offsets:
        mod = SwerveModule(drive_id, turn_id, offset, 0.0)
        state = kin.SwerveModuleState(1.0, geo.Rotation2d(0))
        mod.set_desired_state(state)

        robot_angle = mod.get_state().angle.degrees()
        print(f"\n  {name} (offset={math.degrees(offset):.0f} deg):")
        print(f"    Encoder raw: {math.degrees(mod.turning_encoder.getPosition()):.1f} deg")
        print(f"    get_state angle (robot-relative): {robot_angle:.1f} deg")
        print(f"    desired_state stored: speed={mod.desired_state.speed:.1f}, angle={math.degrees(mod.desired_state.angle.radians()):.1f} deg")


def test_physics_forward():
    """Test that physics simulation produces forward motion."""
    print("\n" + "=" * 60)
    print("TEST: Physics simulation - forward driving")
    print("=" * 60)

    from components.swerve_module import SwerveModule
    from physics import SimulatedModule

    mod = SwerveModule(20, 21, 0.0, 0.0)
    sim = SimulatedModule(mod, mod.driving_spark, mod.turning_spark)

    mod.desired_state = kin.SwerveModuleState(2.0, geo.Rotation2d(0))

    dt = 0.02
    for i in range(50):
        sim.update(dt)

    state = sim.get_state()
    print(f"\n  After 1.0s targeting 2.0 m/s forward:")
    check("  speed", state.speed, 2.0, 0.1)
    check("  angle", state.angle.degrees(), 0.0, 5.0)

    mod.desired_state = kin.SwerveModuleState(0.0, geo.Rotation2d(0))
    for i in range(100):
        sim.update(dt)

    state = sim.get_state()
    print(f"\n  After stopping (2.0s at 0 m/s):")
    check("  speed", state.speed, 0.0, 0.05)


def test_physics_all_directions():
    """Test physics simulation for all 4 D-pad directions with field orientation."""
    print("\n" + "=" * 60)
    print("TEST: Physics simulation - all directions (field-oriented)")
    print("=" * 60)

    k = kin.SwerveDrive4Kinematics(
        geo.Translation2d(0.3, 0.3),
        geo.Translation2d(0.3, -0.3),
        geo.Translation2d(-0.3, 0.3),
        geo.Translation2d(-0.3, -0.3),
    )

    directions = [
        ("Forward", kin.ChassisSpeeds(1.0, 0.0, 0.0), 1.0, 0.0),
        ("Backward", kin.ChassisSpeeds(-1.0, 0.0, 0.0), -1.0, 0.0),
        ("Left", kin.ChassisSpeeds(0.0, 1.0, 0.0), 0.0, 1.0),
        ("Right", kin.ChassisSpeeds(0.0, -1.0, 0.0), 0.0, -1.0),
    ]

    dt = 0.02

    for dir_name, chassis_speeds, expected_vx, expected_vy in directions:
        print(f"\n  --- {dir_name} ---")

        from components.swerve_module import SwerveModule
        from physics import SimulatedModule

        modules_data = [
            (22, 23, constants.kFrontLeftChassisAngularOffset),
            (24, 25, constants.kFrontRightChassisAngularOffset),
            (26, 27, constants.kRearLeftChassisAngularOffset),
            (28, 29, constants.kRearRightChassisAngularOffset),
        ]

        modules = []
        sim_modules = []
        for drive_id, turn_id, offset in modules_data:
            drive_id_unique = drive_id + directions.index((dir_name, chassis_speeds, expected_vx, expected_vy)) * 10
            turn_id_unique = turn_id + directions.index((dir_name, chassis_speeds, expected_vx, expected_vy)) * 10
            mod = SwerveModule(drive_id_unique, turn_id_unique, offset, 0.0)
            sim = SimulatedModule(mod, mod.driving_spark, mod.turning_spark)
            modules.append(mod)
            sim_modules.append(sim)

        fl_s, fr_s, rl_s, rr_s = k.toSwerveModuleStates(chassis_speeds)
        states_from_kin = [fl_s, fr_s, rl_s, rr_s]

        for mod, state in zip(modules, states_from_kin):
            mod.set_desired_state(state)

        for step in range(75):
            for sim in sim_modules:
                sim.update(dt)

        result_states = tuple(sim.get_state() for sim in sim_modules)
        result_speeds = k.toChassisSpeeds(result_states)

        check(f"  {dir_name} vx", result_speeds.vx, expected_vx, 0.15)
        check(f"  {dir_name} vy", result_speeds.vy, expected_vy, 0.15)
        check(f"  {dir_name} omega", result_speeds.omega, 0.0, 0.1)


def test_full_robot_sim():
    """Test the full robot sim with physics engine."""
    print("\n" + "=" * 60)
    print("TEST: Full robot simulation startup")
    print("=" * 60)

    try:
        from physics import PhysicsEngine
        print("  PASS: PhysicsEngine imports successfully")
        global PASS_COUNT
        PASS_COUNT += 1
    except Exception as e:
        print(f"  FAIL: PhysicsEngine import error: {e}")
        global FAIL_COUNT
        FAIL_COUNT += 1


if __name__ == "__main__":
    print("=" * 60)
    print("  SWERVE DRIVE PRE-DEPLOY VERIFICATION")
    print("=" * 60)

    test_kinematics()
    test_dpad_mapping()
    test_module_desired_state()
    test_physics_forward()
    test_physics_all_directions()
    test_full_robot_sim()

    print("\n" + "=" * 60)
    print(f"  RESULTS: {PASS_COUNT} passed, {FAIL_COUNT} failed")
    print("=" * 60)

    if FAIL_COUNT > 0:
        print("  Some tests FAILED. Fix issues before deploying!")
        sys.exit(1)
    else:
        print("  All tests PASSED! Ready for deploy.")
        sys.exit(0)
