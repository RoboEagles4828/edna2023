from hardware_interface.drivetrain import DriveTrain
from hardware_interface.joystick import Joystick
import wpilib

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.drive_train = DriveTrain()
        self.joystick = Joystick()

    def teleopPeriodic(self):
        self.drive_train.arcadeDrive(self.joystick)