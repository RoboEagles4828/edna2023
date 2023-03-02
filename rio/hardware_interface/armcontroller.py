import time
import logging

NAMESPACE = 'real'

CMD_TIMEOUT_SECONDS = 1

class ArmController():
    def __init__(self):
        self.last_cmds_time = time.time()
        self.warn_timeout = True
    
    def getEncoderData():
        print("Get encoder data here")

    def stop():
        print("Stopping")

    def sendCommands(self, commands):
        if commands:
            self.last_cmds_time = time.time()
            logging.info(f"Recieved command {str(commands)}")
            self.warn_timeout = True
        elif (time.time() - self.last_cmds_time > CMD_TIMEOUT_SECONDS):
            self.stop()
            if self.warn_timeout:
                logging.warning(f"Didn't recieve any commands for {CMD_TIMEOUT_SECONDS} second(s). Halting...")
                self.warn_timeout = False
        print("Recieving commands")