import logging
import wpilib
import threading
import traceback
import time





######### Robot Class #########
class motor_poc(wpilib.TimedRobot):

    def robotInit(self) -> None:
        logging.info("Entering Robot Init")

    def teleopInit(self) -> None:
        logging.info("Entering Teleop")
    
    def teleopPeriodic(self) -> None:
        pass

    def teleopExit(self) -> None:
        logging.info("Exiting Teleop")


if __name__ == '__main__':
    wpilib.run(motor_poc)

