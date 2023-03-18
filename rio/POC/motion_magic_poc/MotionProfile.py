import argparse
import logging
import math
from collections import deque
import copy
import matplotlib.pyplot as plt

logger = logging.getLogger(f"{__name__}")

#Constants
VPROG = 4.00
T1 = 400
T2 = 200
ITP = 10

if __name__ == '__main__':
    # Instantiate the parser
    parser = argparse.ArgumentParser(description='Talon SRX motion profiler')

    # Required Positional Arguments
    parser.add_argument('-d', '--distance', type=str, required=True,
                        help="Distance to move")
    parser.add_argument('-l', "--loglevel", type=str,
                        default='INFO', help="logging level")
    parser.add_argument('-v', '--max_vel', type=str, help="max velocity", default="4.0")
    parser.add_argument('-a', '--max_accel', type=str, help="max_acclereation", default="10.0")

    args = parser.parse_args()

    # Setup logger
    numeric_level = getattr(logging, args.loglevel.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError(f"Invalid log level: {args.loglevel}")
    logging.basicConfig(format='%(asctime)s-%(levelname)s-%(message)s',
                        datefmt='%H:%M:%S', level=numeric_level)

    VPROG = float(args.max_vel)
    max_accel = float(args.max_accel)

    if max_accel != 0.0:
        T1 = VPROG/max_accel * 1000

    # Calculation constants
    dist = float(args.distance)
    t4 = (dist/VPROG) * 1000
    fl1 = float(math.trunc(T1 / ITP))
    fl2 = math.trunc(T2 / ITP)
    N = float(t4 / ITP)

    step = 1
    time = 0.0
    velocity = 0.0
    input = False
    runningSum = deque(fl2*[0.0], fl2)
    trajectory = list()
    trajectory.append({"step": step, "time": time, "velocity": velocity})
    runningSum.append(0.0)
    zeropt = list()
    # Now we print out the trajectory
    fs1 = 0.0
    exit = False
    while not exit:
        step += 1
        time = ((step - 1) * ITP)/1000
        input = 1 if step < (N+2) else 0
        addition = (1/fl1) if input==1 else (-1/fl1)
        fs1 = max(0, min(1, fs1 + addition))
        # logging.log(numeric_level, f"fs1 = {fs1}")
        fs2 = sum(runningSum)
        # logging.log(numeric_level, f"fs2 = {fs2}")
        runningSum.append(fs1)
        if fs1 == 0:
            if fs2 == 0:
                zeropt.append(1)
            else:
                zeropt.append(0)
        else:
            zeropt.append(0)
        output_included = 1 if sum(zeropt[0:step]) <= 2 else 0
        if output_included == 1:
            velocity = ((fs1+fs2)/(1.0/fl2))*VPROG if fs2 != 0 else 0
            trajectory.append({"step": step, "time": time, "velocity": velocity})
        else:
            exit = True
            logging.log(numeric_level, "EXITTING")

    print(trajectory)
    x = list()
    y = list()

    for i in trajectory:
        x.append(i["time"])
        y.append(i["velocity"])
    plt.plot(x, y)
    plt.xlabel("TIME")
    plt.ylabel("VELOCITY")
    plt.show()