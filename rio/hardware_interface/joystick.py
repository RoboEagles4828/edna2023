import wpilib
import logging

CONTROLLER_PORT = 0
SCALING_FACTOR_FIX = 10000

ENABLE_THROTTLE = True

pov_x_map = {
    -1: 0.0,
    0: 0.0,
    45: -1.0,
    90: -1.0,
    135: -1.0,
    180: 0.0,
    225: 1.0,
    270: 1.0,
    315: 1.0,
}

pov_y_map = {
    -1: 0.0,
    0: 1.0,
    45: 1.0,
    90: 0.0,
    135: -1.0,
    180: -1.0,
    225: -1.0,
    270: 0.0,
    315: 1.0,
}

class Joystick:
    def __init__(self):
        self.joystick = wpilib.XboxController(CONTROLLER_PORT)
        self.deadzone = 0.15
        self.last_joystick_data = self.getEmptyData()
        self.count = 0

    def scaleAxis(self, axis):
        return int(axis * SCALING_FACTOR_FIX * -1)
    
    def scaleTrigger(self, trigger):
        return int(trigger * SCALING_FACTOR_FIX * -1)
    
    def getEmptyData(self):
        return {
            "axes": [0.0] * 8,
            "buttons": [0] * 11,
        }
    
    def getData(self):
        pov = self.joystick.getPOV(0)
        leftX = self.joystick.getLeftX()
        leftY = self.joystick.getLeftY()
        rightX = self.joystick.getRightX()
        rightY = self.joystick.getRightY()
        leftTrigger = self.joystick.getLeftTriggerAxis()
        rightTrigger = self.joystick.getRightTriggerAxis()

        axes = [
            self.scaleAxis(leftX) if abs(leftX) > self.deadzone else 0.0,
            self.scaleAxis(leftY) if abs(leftY) > self.deadzone else 0.0,
            self.scaleTrigger(leftTrigger),
            self.scaleAxis(rightX) if abs(rightX) > self.deadzone else 0.0,
            
            self.scaleAxis(rightY) if abs(rightY) > self.deadzone else 0.0,
            self.scaleTrigger(rightTrigger),
            
            pov_x_map[pov], # left 1.0 right -1.0
            pov_y_map[pov], # up 1.0 down -1.0
        ]
        buttons = [
            int(self.joystick.getAButton()),
            int(self.joystick.getBButton()),
            int(self.joystick.getXButton()),
            int(self.joystick.getYButton()),
            int(self.joystick.getLeftBumper()),
            int(self.joystick.getRightBumper()),
            int(self.joystick.getBackButton()),
            int(self.joystick.getStartButton()),
            0,
            int(self.joystick.getLeftStickButton()),
            int(self.joystick.getRightStickButton())
        ]

        data = {"axes": axes, "buttons": buttons}

        if ENABLE_THROTTLE:
            if self.is_equal(data, self.last_joystick_data):
                self.count += 1
                if self.count >= 10:
                    return None
            else:
                self.count = 0
                self.last_joystick_data = data
                return data
        else:
            return data

    def is_equal(self, d, d1):
        res = all((d1.get(k) == v for k, v in d.items()))
        return res
    
    

