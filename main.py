# Copyright information
#
# © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

import os
import sys
import time
import logging
from functools import partial
import limxsdk.datatypes as datatypes
import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.robot.Rate as Rate

logger = logging.getLogger("Main")

# Callback function for receiving sensor joy data
def sensor_joy_callback(sensor_joy: datatypes.SensorJoy):
    # Both L1 (button index 4) and Y (button index 3) are pressed to start the stand controller
    if sensor_joy.buttons[4] == 1 and sensor_joy.buttons[3] == 1:
      os.system(f"python3 -m limxsdk.ability.cli switch 'mimic damping walk' 'stand'")

    # Both L1 (button index 4) and B (button index 1) are pressed to start the mimic controller
    if sensor_joy.buttons[4] == 1 and sensor_joy.buttons[1] == 1:
      os.system(f"python3 -m limxsdk.ability.cli switch 'stand damping walk' 'mimic'")
    
    # Both R1 (button index 7) and X (button index 2) are pressed to start the walk controller
    if sensor_joy.buttons[7] == 1 and sensor_joy.buttons[2] == 1:
      os.system(f"python3 -m limxsdk.ability.cli switch 'stand damping mimic' 'walk'")

    # Both L1 (button index 4) and A (button index 0) are pressed to start the damping controller
    if sensor_joy.buttons[4] == 1 and sensor_joy.buttons[0] == 1:
      os.system(f"python3 -m limxsdk.ability.cli switch 'stand mimic walk' 'damping'")

    # Both L1 (button index 4) and X (button index 2) are pressed to exit
    if sensor_joy.buttons[4] == 1 and sensor_joy.buttons[2] == 1:
      os.system(f"python3 -m limxsdk.ability.cli switch 'stand mimic damping walk' ''")
        
if __name__ == '__main__':
    # Get the robot type from the environment variable
    robot_type = os.getenv("ROBOT_TYPE")
    
    # Check if the ROBOT_TYPE environment variable is set, otherwise exit with an error
    if not robot_type:
        print("Error: Please set the ROBOT_TYPE using 'export ROBOT_TYPE=<robot_type>'.")
        sys.exit(1)

    # Create a Robot instance of the specified type
    robot = Robot(RobotType.Humanoid)

    # Default IP address for the robot
    robot_ip = "127.0.0.1"
    
    # Check if command-line argument is provided for robot IP
    if len(sys.argv) > 1:
        robot_ip = sys.argv[1]

    # Initialize the robot with the provided IP address
    if not robot.init(robot_ip):
        sys.exit()
    
    # Set ROBOT_IP environment variable for controllers
    os.environ["ROBOT_IP"] = robot_ip

    script_dir = os.path.dirname(os.path.abspath(__file__))
    controller_path = f"{script_dir}/controllers/{robot_type}/controllers.yaml"

    # Set up a callback to receive updated SensorJoy
    sensor_joy_callback_partial = partial(sensor_joy_callback)
    robot.subscribeSensorJoy(sensor_joy_callback_partial)
    
    # Load controllers
    os.system(f"python3 -m limxsdk.ability.cli load --config {controller_path}")
