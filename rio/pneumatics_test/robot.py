import wpilib

# does not work yet

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        print("Initalizing")

        # make rev hub object
        self.hub = wpilib.PneumaticHub(1)

        # make single solenoid objects (pass in channel as parameter)
        self.solenoids = [self.hub.makeSolenoid(0), self.hub.makeSolenoid(1)]

        # make double solenoid objects (pass in forward channel and reverse channel as parameters)
        self.double_solenoids = [self.hub.makeDoubleSolenoid(2, 3), self.hub.makeDoubleSolenoid(4, 5)]

        # make compressor
        self.compressor = self.hub.makeCompressor(1)

        self.timer = wpilib.Timer
        self.input = wpilib.Joystick(0)

    def teleopInit(self):
        print("Starting Compressor")
        self.compressor.enableDigital()
        for solenoid in self.double_solenoids:
            solenoid.set(wpilib.DoubleSolenoid.Value.kOff)


    def teleopPeriodic(self):
        if self.input.getRawButtonPressed(5): # LB (Xbox)
            print("Toggling First...")
            for solenoid in self.double_solenoids:
                solenoid.toggle()

        if self.input.getRawButtonPressed(6): # RB (Xbox)
            print("Toggling Second...")
            for solenoid in self.double_solenoids:
                solenoid.toggle()

    def teleopExit(self):
        print("Turning off compressor...")
        self.compressor.disable()


if __name__ == "__main__":
    wpilib.run(MyRobot) 