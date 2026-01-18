import rev
import math

class SwerveMotor:
    move = None #Drive motor (injected by magicbot)
    rotate = None # Steering motr (injected
    abs_encoder = None
    def __init__(self):
        self.turn_pid = rev.SparkMaxPIDController(self.rotate)
        self.drive_pid = rev.SparkMaxPIDController(self.move)

    def set_angle(self, target_angle_deg):
        current = self.abs_encoder.getPostion()
        # convert target to encoder units, run PID

        self.turn_pid.setReference(target_angle_deg, rev.ControlType.kPosition)

    def set_speed(self, speed_mps):
        # convert m/s to motor RPM using wheel diamter + gear ratio

        rpm = speed_mps * 60 / (0.1016 * math.pi)

        self.drive_pid.setReference(rpm, rev.ControlType.kVelocity)

    def drive(self,speed_mps, angle_deg):
        self.set_angle(angle_deg)
        self.set_speed(speed_mps)