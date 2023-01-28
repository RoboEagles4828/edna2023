import wpilib

CONTROLLER_PORT = 0

def get_joystick() -> wpilib.XboxController:
    return wpilib.XboxController(CONTROLLER_PORT)