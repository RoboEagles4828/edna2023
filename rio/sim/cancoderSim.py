from ctre.sensors import CANCoder, CANCoderSimCollection

import math

CANCODER_TICKS_PER_REV = 4096

class CancoderSim():
    def __init__(self, cancoder : CANCoder, offsetDegress : float = 0.0, sensorPhase: bool = False):
        self.cancoder : CANCoder = cancoder
        self.cancoderSim : CANCoderSimCollection = self.cancoder.getSimCollection()
        self.cancoderSim.setRawPosition(self.radiansToEncoderTicks(0, "position"))
        self.offset = math.radians(offsetDegress)
        self.sensorPhase = -1 if sensorPhase else 1
        self.velocity = 0.0
        self.position = 0.0
    
    def update(self, period : float, velocityRadians : float):
        self.position = velocityRadians * period * self.sensorPhase
        self.velocity = velocityRadians * self.sensorPhase
        
        # Update the encoder sensors on the motor
        self.cancoderSim = self.cancoder.getSimCollection()
        self.cancoderSim.addPosition(self.radiansToEncoderTicks(self.position, "position"))
        self.cancoderSim.setVelocity(self.radiansToEncoderTicks(self.velocity, "velocity"))
    
    def radiansToEncoderTicks(self, radians : float, displacementType : str) -> int:
        ticks = radians * CANCODER_TICKS_PER_REV / (2 * math.pi)
        if displacementType == "position":
            return int(ticks)
        else:
            return int(ticks / 10)