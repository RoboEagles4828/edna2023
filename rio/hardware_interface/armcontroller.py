import wpilib
import ctre
import time
import logging

NAMESPACE = 'real'
CMD_TIMEOUT_SECONDS = 1

# Port Numbers for all of the Solenoids and other connected things
# The numbers below will **need** to be changed to fit the robot wiring
PORTS = {
    # Modules
    'HUB': 1,
    'COMPRESSOR': 0,
    'MODULE': 0,
    # Pistons
    'ARM_ROLLER_BAR': [0, 1, 2, 3],
    'TOP_GRIPPER_SLIDER': [4, 5, 6, 7],
    'TOP_GRIPPER': [8, 9],
    'BOTTOM_GRIPPER': [10, 11],
}


class ArmController():

    def __init__(self):
        self.last_cmds_time = time.time()
        self.warn_timeout = True
        self.hub = wpilib.PneumaticHub(PORTS['HUB'])

        self.compressor = self.hub.makeCompressor()

        self.arm_roller_bar =       DoublePiston(self.hub, PORTS['ARM_ROLLER_BAR'])
        self.top_gripper_slider =   DoublePiston(self.hub, PORTS['TOP_GRIPPER_SLIDER'])
        self.top_gripper =          Piston(self.hub, PORTS['TOP_GRIPPER'])
        self.bottom_gripper =       Piston(self.hub, PORTS['BOTTOM_GRIPPER'])
        
        self.JOINT_MAP : dict[str, DoublePiston | Piston] = {
            # Pneumatics
            'arm_roller_bar_joint':     self.arm_roller_bar,
            'top_gripper_slider_joint': self.top_gripper_slider,
            'top_gripper_joint':        self.top_gripper,
            'bottom_gripper_joint':     self.bottom_gripper
        }
        

    def getEncoderData(self):
        names = [""]*6
        positions = [0]*6
        velocities = [0]*6

        # Iterate over the JOINT_MAP and run the get() function for each of them
        for index, joint_name in enumerate(self.JOINT_MAP.keys()):
            names[index] = joint_name
            positions[index] = self.JOINT_MAP[joint_name].getPosition()
            velocities[index] = self.JOINT_MAP[joint_name].getVelocity()
        return { "name" : names, "position": positions, "velocity": velocities}

    # TODO: Add this later!!!
    def stop(self): 0

    def sendCommands(self, commands):
        if commands:
            self.last_cmds_time = time.time()
            self.warn_timeout = True
            for i in range(len(commands["name"])):
                self.JOINT_MAP[commands["name"][i]].setPosition(commands['position'][i])
        
        elif (time.time() - self.last_cmds_time > CMD_TIMEOUT_SECONDS):
            self.stop()
            if self.warn_timeout:
                logging.warning(f"Didn't recieve any commands for {CMD_TIMEOUT_SECONDS} second(s). Halting...")
                self.warn_timeout = False

class Piston():

    def __init__(self, hub : wpilib.PneumaticHub, ports : list[int]):
        self.solenoid = hub.makeDoubleSolenoid(ports[0], ports[1])

    def getPosition(self) -> int: # Off - 0, Forward - 1, Reverse - 2
        return 1 if self.solenoid.get().value == wpilib.DoubleSolenoid.Value.kForward else 0
    
    # The Solenoids don't have a velocity value, so we set it to zero here
    def getVelocity(self) -> int: return 0

    def setPosition(self, position : int):
        self.solenoid.set(wpilib.DoubleSolenoid.Value.kReverse if position == 0 else wpilib.DoubleSolenoid.Value.kForward)

class DoublePiston():
    
    def __init__(self, hub : wpilib.PneumaticHub, ports : list[int]):
        self.pistonA = Piston(hub, ports[:2])
        self.pistonB = Piston(hub, ports[2:])

    def getPosition(self) -> int:
        return self.pistonA.getPosition()
    
    def getVelocity(self) -> int: return 0

    def setPosition(self, position : int):
        self.pistonA.setPosition(position)
        self.pistonB.setPosition(position)
    