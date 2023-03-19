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
        
        # Determine a small change in velocity for this period
        accelAmount = self.fullVel / self.accelToFullTime * period
        # Determine how much effort the motor should be doing
        outPerc = self.talonSim.getMotorOutputLeadVoltage() / 12
        if self.sensorPhase:
            outPerc *= -1
        

        # Calculate the velocity
        # The wheel should move at some proportion of the full velocity with some noise 
        theoreticalVel = outPerc * self.fullVel * self.randomFloatRange(0.95, 1.0)
        # If the theoretical velocity is greater than how much we can accelerate in this period, max accelerate
        if theoreticalVel > self.velocity + accelAmount:
            self.velocity += accelAmount
        # If the theoretical velocity is less than how much we can deccelerate in this period, max deccelerate
        elif theoreticalVel < self.velocity - accelAmount:
            self.velocity -= accelAmount
        # Otherwise, we are close enough to the theoretical velocity, so just set it
        else:
            self.velocity = 0.9 * (theoreticalVel - self.velocity)
        
        # Tolerance to set the velocity to 0
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


    # For viewing and testings purposes
    def getSimulatedPosition(self) -> float:
        return self.talon.getSelectedSensorPosition()

    def getSimulatedVelocity(self) -> float:
        return self.talon.getSelectedSensorVelocity()