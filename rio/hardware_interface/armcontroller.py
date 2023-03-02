import time

NAMESPACE = 'real'

class ArmController():
    def __init__(self):
        self.last_cmds_time = time.time()
        self.warn_timeout = True