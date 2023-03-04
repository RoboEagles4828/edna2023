import wpilib
import time
import logging

NAMESPACE = 'real'
CMD_TIMEOUT_SECONDS = 1

PORTS = {
    'HUB': 1,
    'COMPRESSOR': 0,
    'MODULE': 0,
    'ARM_ROLLER_BAR': {
        'OPEN': 0,
        'CLOSED': 7
    },
    'TOP_GRIPPER': {
        'OPEN': 3,
        'CLOSED': 4
    },
}


class ArmController():



    def __init__(self):
        self.last_cmds_time = time.time()
        self.warn_timeout = True
        self.hub = wpilib.PneumaticHub(PORTS['HUB'])

        self.compressor = wpilib.Compressor(PORTS['COMPRESSOR'])

        self.JOINT_MAP = {
            # Pneumatics
            'arm_roller_bar_joint':     {'function': self.runPneumatic, 'object': self.arm_roller_bar},
            'top_gripper_joint':        {'function': self.runPneumatic, 'object': self.top_gripper},
            # 'top_gripper_slider_joint': {'function': self.runPneumatic, 'object': self.top_gripper_slider}

        }

        self.arm_roller_bar = wpilib.DoubleSolenoid(
            PORTS["MODULE"], 
            wpilib.PneumaticsModuleType.CTREPCM, 
            PORTS["ARM_ROLLER_BAR"]["OPEN"], 
            PORTS["ARM_ROLLER_BAR"]["CLOSED"])
        
        self.top_gripper = wpilib.DoubleSolenoid(
            PORTS["MODULE"],
            wpilib.PneumaticsModuleType.CTREPCM,
            PORTS["TOP_GRIPPER"]["OPEN"],
            PORTS["TOP_GRIPPER"]["CLOSED"]
        )

    def initPneumatics(self, pneumatic):
        print("Starting Compressor")
        self.compressor.start()

    def runPneumatic(self, pneumatic, position):
        print(f"Moving {list(filter(lambda x: self.JOINT_MAP[x] == pneumatic, self.JOINT_MAP))[0]} to position {position}")
        pneumatic.set


    
    def getEncoderData():
        print("Get encoder data here")

    def stop(self):
        1+1
        # print("Stopping")

    def sendCommands(self, commands):
        if commands:
            self.last_cmds_time = time.time()
            logging.info(f"Recieved command {str(commands)}")
            self.warn_timeout = True
            for i in range(len(commands["name"])):
                joint = self.JOINT_MAP[commands.name[i]]
                joint['function'](self, joint['object'], commands['position'][i])
        
        elif (time.time() - self.last_cmds_time > CMD_TIMEOUT_SECONDS):
            self.stop()
            if self.warn_timeout:
                logging.warning(f"Didn't recieve any commands for {CMD_TIMEOUT_SECONDS} second(s). Halting...")
                self.warn_timeout = False
        # print("Recieving commands")