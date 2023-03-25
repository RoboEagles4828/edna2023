import wpilib

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        print("Initalizing")
        self.limitSwitch = wpilib.DigitalInput(0)
        self.pastState = 0

        self.timer = wpilib.Timer
        self.input = wpilib.Joystick(0)

    def teleopInit(self):
        print("Entering teleop")

    def teleopPeriodic(self):
        if (self.limitSwitch.get() != self.pastState):
            print(f"State Changed from {self.pastState} to {self.limitSwitch.get()}")
            self.pastState = self.limitSwitch.get()

    def teleopExit(self):
        print("Exitted teleop")

if __name__ == "__main__":
    wpilib.run(MyRobot) 