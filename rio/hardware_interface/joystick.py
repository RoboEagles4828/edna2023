import wpilib
import logging

CONTROLLER_PORT = 0
SCALING_FACTOR_FIX = 10000

DPAD_REF = {
    "DPAD_UP": 0,
    "DPAD_RIGHT": 90,
    "DPAD_DOWN": 180,
    "DPAD_LEFT": 270
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
            getDpad("DPAD_UP"),
            getDpad("DPAD_DOWN"),
            getDpad("DPAD_LEFT"),
            getDpad("DPAD_RIGHT"),
            self.joystick.getLeftStickButton(),
            self.joystick.getRightStickButton()
        ]

        def scale(a):
            return int(a * SCALING_FACTOR_FIX * -1)
        def toInt(b):
            return int(b)
        
        def getDpad(direction: str):
            dpadValue = self.joystick.getPOV()
            value = DPAD_REF[direction]
            if dpadValue == value or dpadValue == ((value + 45) % 360) or dpadValue == ((value + 315) % 360):
                return 1
            else:
                return 0
    
        axes = map(scale, axes)
        buttons = map(toInt, buttons)

        return {"axes": list(axes), "buttons": list(buttons)}

