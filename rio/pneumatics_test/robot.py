import wpilib

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        print("Initalizing")

        self.compressor = wpilib.Compressor(0, wpilib.PneumaticsModuleType.CTREPCM)
        self.solenoids = [
            [wpilib.Solenoid(0, wpilib.PneumaticsModuleType.CTREPCM, 0), wpilib.Solenoid(0, wpilib.PneumaticsModuleType.CTREPCM, 7)],
            [wpilib.Solenoid(0, wpilib.PneumaticsModuleType.CTREPCM, 3), wpilib.Solenoid(0, wpilib.PneumaticsModuleType.CTREPCM, 4)]
        ]

        self.timer = wpilib.Timer
        self.input = wpilib.Joystick(0)

    def teleopInit(self):
        print("Starting Compressor")
        self.compressor.start()
        self.compressor.enableDigital()
        for solenoidarr in self.solenoids:
            solenoidarr[0].set(False)
            solenoidarr[1].set(True)

    def teleopPeriodic(self):
        if self.input.getRawButtonPressed(5): # LB (Xbox)
            print("Toggling First...")
            for solenoid in self.solenoids[0]:
                solenoid.toggle()

        if self.input.getRawButtonPressed(6): # RB (Xbox)
            print("Toggling Second...")
            for solenoid in self.solenoids[1]:
                solenoid.toggle()

    def teleopExit(self):
        print("Turning off compressor...")
        self.compressor.disable()
        self.compressor.stop()


if __name__ == "__main__":
    wpilib.run(MyRobot) 