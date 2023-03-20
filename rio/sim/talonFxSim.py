from ctre import TalonFX, TalonFXSimCollection
from wpilib import RobotController
import random
import math
# from pyfrc.physics.motor_cfgs import MOTOR_CFG_FALCON_500

import wpilib.simulation
import wpimath.system.plant

FALCON_SENSOR_TICKS_PER_REV = 2048

class TalonFxSim:

    def __init__(self, talonFx : TalonFX, moi : float, gearRatio : float, sensorPhase : bool ) -> None:
        self.talon : TalonFX = talonFx
        self.talonSim : TalonFXSimCollection = None
        self.moi = moi
        self.gearRatio = gearRatio
        self.sensorPhase = -1 if sensorPhase else 1
        self.gearbox = wpimath.system.plant.DCMotor.falcon500(1)        
        self.motor = wpilib.simulation.DCMotorSim(self.gearbox, self.gearRatio, self.moi, [0.0, 0.0])
        self.velocity = 0.0
        self.position = 0.0
        self.fwdLimitEnabled = False
        self.fwdLimit = 0.0
        self.revLimitEnabled = False
        self.revLimit = 0.0

    # Simulates the movement of falcon 500 motors by getting the voltages from the
    # the motor model that is being controlled by the robot code.
    def update(self, period : float):
        self.talonSim = self.talon.getSimCollection()
        
        # Update the motor model
        voltage = self.talonSim.getMotorOutputLeadVoltage() * self.sensorPhase
        self.motor.setInputVoltage(voltage)
        self.motor.update(period)
        newPosition = self.motor.getAngularPosition()
        self.deltaPosition = newPosition - self.position
        self.position = newPosition
        self.velocity = self.motor.getAngularVelocity()

        if self.fwdLimitEnabled and self.position >= self.fwdLimit:
            self.talonSim.setLimitFwd(True)
            self.position = self.fwdLimit
        else:
            self.talonSim.setLimitFwd(False)
        
        if self.revLimitEnabled and self.position <= self.revLimit:
            self.talonSim.setLimitRev(True)
            self.position = self.revLimit
        else:
            self.talonSim.setLimitRev(False)

        # Update the encoder sensors on the motor
        positionShaftTicks = int(self.radiansToSensorTicks(self.position * self.gearRatio, "position"))
        velocityShaftTicks = int(self.radiansToSensorTicks(self.velocity * self.gearRatio, "velocity"))
        self.talonSim.setIntegratedSensorRawPosition(positionShaftTicks)
        self.talonSim.setIntegratedSensorVelocity(velocityShaftTicks)

        # Update the current and voltage
        self.talonSim.setSupplyCurrent(self.motor.getCurrentDraw())
        self.talonSim.setBusVoltage(RobotController.getBatteryVoltage())
    
    def radiansToSensorTicks(self, radians : float, displacementType : str) -> int:
        ticks = radians * FALCON_SENSOR_TICKS_PER_REV / (2 * math.pi)
        if displacementType == "position":
            return ticks
        else:
            return ticks / 10
    
    def getPositionRadians(self) -> float:
        return self.position

    def getVelocityRadians(self) -> float:
        return self.velocity

    def getSupplyCurrent(self) -> float:
        return self.motor.getCurrentDraw()
    
    def addLimitSwitch(self, limitType: str, positionRadians: float):
        if limitType == "fwd":
            self.fwdLimitEnabled = True
            self.fwdLimit = positionRadians
        elif limitType == "rev":
            self.revLimitEnabled = True
            self.revLimit = positionRadians
        else:
            print("Invalid limit type")