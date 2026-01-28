# from magicbot import MagicRobot
# import wpilib
# # import rev
#
# from ..components.sample_drive_motor import SampleDriveMotor
#
#
# class MyRobot(MagicRobot):
#
#     sample_drive_motor: SampleDriveMotor
#
#     def createObjects(self):
#         self.joystick = wpilib.Joystick(0)
#
#         # Correct for your version of REVLib
#         self.motor = rev.SparkMax(
#             1,
#             rev.SparkMax.MotorType.kBrushless
#         )
#
#     def configureComponents(self):
#         self.sample_drive_motor.inject(self.motor)
#
#     def teleopPeriodic(self):
#         power = -self.joystick.getY()
#         self.sample_drive_motor.set_power(power)
#
#
# if __name__ == "__main__":
#     wpilib.run(MyRobot)