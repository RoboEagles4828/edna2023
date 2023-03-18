from ctre.sensors import CANCoder, CANCoderSimCollection

import math

class CancoderSim():
    def __init__(self, cancoder : CANCoder, offset : float = 0.0):
        self.encoder : CANCoder = cancoder
        self.encoderSim : CANCoderSimCollection = None
        self.offset = offset
        # self.encoder.getSimCollection().setRawPosition(self.offset)
    
    def radiansToTicks(self, radians : float) -> int:
        return int(radians * 4096 / (2 * math.pi))

    def update(self, period : float, velocity : float, position : float):
        self.encoderSim = self.encoder.getSimCollection()
        self.encoderSim.setRawPosition(self.radiansToTicks(position + math.radians(self.offset)))
        self.encoderSim.setVelocity(self.radiansToTicks(velocity) * 10)