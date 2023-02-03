import wpilib
import logging

CONTROLLER_PORT = 0
SCALING_FACTOR_FIX = 10000

class Joystick:
    def __init__(self):
        self.joystick = wpilib.XboxController(CONTROLLER_PORT)

    def getData(self):
        axes = [
            self.joystick.getLeftX(),
            self.joystick.getLeftY(),
            self.joystick.getLeftTriggerAxis(),
            self.joystick.getRightX(),
            self.joystick.getRightY(),
            self.joystick.getRightTriggerAxis()
        ]
        buttons = [
            self.joystick.getAButton(),
            self.joystick.getBButton(),
            self.joystick.getXButton(),
            self.joystick.getYButton(),
            self.joystick.getLeftBumper(),
            self.joystick.getRightBumper(),
            self.joystick.getBackButton(),
            self.joystick.getStartButton(),
            0,
            self.joystick.getLeftStickButton(),
            self.joystick.getRightStickButton()
        ]

        def scale(a):
            return int(a * SCALING_FACTOR_FIX)
        def toInt(b):
            return int(b)

        axes = map(scale, axes)
        buttons = map(toInt, buttons)

        return {"axes": list(axes), "buttons": list(buttons)}

