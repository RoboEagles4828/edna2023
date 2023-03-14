from robot import edna_robot
import wpilib

if __name__ == '__main__':
    wpilib.run(edna_robot, use_threading=True, use_mocks=True)