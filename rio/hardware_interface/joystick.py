import wpilib
import logging

CONTROLLER_PORT = 0
SCALING_FACTOR_FIX = 10000

DPAD_REF = {
    "up": 0,
    "down": 180,
    "left": 270,
    "right": 90,
}

class Joystick:
    def __init__(self):
        self.joystick = wpilib.XboxController(CONTROLLER_PORT)
        self.deadzone = 0.1

    def getData(self):
        axes = [
            self.joystick.getLeftX() if abs(self.joystick.getLeftX()) > self.deadzone else 0.0,
            self.joystick.getLeftY() if abs(self.joystick.getLeftY()) > self.deadzone else 0.0,
            self.joystick.getLeftTriggerAxis() if abs(self.joystick.getLeftTriggerAxis()) > self.deadzone else 0.0,
            self.joystick.getRightX() if abs(self.joystick.getRightX()) > self.deadzone else 0.0,
            self.joystick.getRightY() if abs(self.joystick.getRightY()) > self.deadzone else 0.0,
            self.joystick.getRightTriggerAxis() if abs(self.joystick.getRightTriggerAxis()) > self.deadzone else 0.0
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
            self.getDPAD("up"),
            self.getDPAD("down"),
            self.getDPAD("left"),
            self.getDPAD("right"),
            self.joystick.getLeftStickButton(),
            self.joystick.getRightStickButton()
        ]

        def scale(a):
            return int(a * SCALING_FACTOR_FIX * -1)
        def toInt(b):
            return int(b)

        axes = map(scale, axes)
        buttons = map(toInt, buttons)

        return {"axes": list(axes), "buttons": list(buttons)}
    
    def getDPAD(self, dir):
        pov = self.joystick.getPOV()

        if pov == -1:
            return 0.0
        else:
            return 1.0 if self.joystick.getPOV() == DPAD_REF(dir) else 0.0

