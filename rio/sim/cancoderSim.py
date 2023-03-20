from ctre.sensors import CANCoder, CANCoderSimCollection

import math

CANCODER_TICKS_PER_REV = 4096

class CancoderSim():
    def __init__(self, cancoder : CANCoder, offsetDegress : float = 0.0, sensorPhase: bool = False):
        self.encoder : CANCoder = cancoder
        self.encoderSim : CANCoderSimCollection = None
        self.offset = math.radians(offsetDegress)
        self.sensorPhase = sensorPhase
        self.velocity = 0.0
        self.position = 0.0
        self.encoder.getSimCollection().setRawPosition(self.radiansToEncoderTicks(0, "position"))
    
    def update(self, period : float, velocityRadians : float):
        if self.sensorPhase:
            velocityRadians *= -1
        self.position = velocityRadians * period
        self.velocity = velocityRadians
        
        # Update the encoder sensors on the motor
        self.encoderSim = self.encoder.getSimCollection()
        self.encoderSim.addPosition(self.radiansToEncoderTicks(self.position, "position"))
        self.encoderSim.setVelocity(self.radiansToEncoderTicks(self.velocity, "velocity"))
    
    def radiansToEncoderTicks(self, radians : float, displacementType : str) -> int:
        ticks = radians * CANCODER_TICKS_PER_REV / (2 * math.pi)
        if displacementType == "position":
            return int(ticks)
        else:
            return int(ticks / 10)

    def getSimulatedPosition(self):
        return math.radians( self.encoder.getAbsolutePosition() )

    def getSimulatedVelocity(self):
        return math.radians( self.encoder.getVelocity() )