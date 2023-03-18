from ctre import TalonFX, TalonFXSimCollection
from wpilib import RobotController
import random
import math
# from pyfrc.physics.motor_cfgs import MOTOR_CFG_FALCON_500

class TalonFxSim:

    def __init__(self, talonFx : TalonFX, accelToFullTime : float, fullVel : float, sensorPhase : bool ) -> None:
        self.talon : TalonFX = talonFx
        self.talonSim : TalonFXSimCollection = None
        self.accelToFullTime = accelToFullTime
        self.fullVel = fullVel
        self.sensorPhase = sensorPhase
        self.velocity = 0.0
        self.pos = 0.0
        self.deltaPos = 0.0
        self.supplyCurrent = 0.0
        self.statorCurrent = 0.0
    
    
    # Simulates the movement of falcon 500 motors by getting the voltages from the
    # the motor model that is being controlled by the robot code.
    def update(self, period : float):
        self.talonSim = self.talon.getSimCollection()
        accelAmount = self.fullVel / self.accelToFullTime * period
        outPerc = self.talonSim.getMotorOutputLeadVoltage() / 12
        if self.sensorPhase:
            outPerc *= -1
        
        # Calculate the velocity
        theoreticalVel = outPerc * self.fullVel * self.randomFloatRange(0.95, 1.0)
        if theoreticalVel > self.velocity + accelAmount:
            self.velocity += accelAmount
        elif theoreticalVel < self.velocity - accelAmount:
            self.velocity -= accelAmount
        else:
            self.velocity = 0.9 * (theoreticalVel - self.velocity)
        
        if abs(outPerc) < 0.01:
            self.velocity = 0
        
        # Calculate the position
        self.deltaPos = self.velocity * period * 10
        self.pos += self.deltaPos

        # Update the encoder sensors on the motor
        self.talonSim.addIntegratedSensorPosition(int(self.deltaPos))
        self.talonSim.setIntegratedSensorVelocity(int(self.velocity))

        # Update the current and voltage
        self.supplyCurrent = abs(outPerc) * 30 * self.randomFloatRange(0.95, 1.05)
        self.statorCurrent = 0 if outPerc == 0 else self.supplyCurrent / abs(outPerc)
        self.talonSim.setSupplyCurrent(self.supplyCurrent)
        self.talonSim.setStatorCurrent(self.statorCurrent)
        self.talonSim.setBusVoltage(RobotController.getBatteryVoltage())
    
    def getSupplyCurrent(self) -> float:
        return self.supplyCurrent

    def getStatorCurrent(self) -> float:
        return self.statorCurrent
    
    def randomFloatRange(self, max, min):
        offset = (max - min) * random.random()
        return round(min + offset, 4)