# Copyright information
#
# © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

import os
import yaml
import time
import limxsdk.robot.Rate as Rate
import limxsdk.datatypes as datatypes
from limxsdk.ability.base_ability import BaseAbility
from limxsdk.ability.registry import register_ability

@register_ability("damping/controller")
class DampingController(BaseAbility):
    """Robot standing controller with smooth transition"""
    
    def initialize(self, config):
        """Initialize controller parameters"""
        self.robot = self.get_robot_instance()
        self.update_rate = config.get("update_rate", 1000.0)
        self.logger.info(f"DampingController initialized with rate: {self.update_rate}Hz")
        
        # Load joint parameters from YAML
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            yaml_path = os.path.join(script_dir, 'joint_params.yaml')
            with open(yaml_path, 'r') as f:
                self.joint_params = yaml.safe_load(f)
            self.logger.info("Joint parameters loaded successfully")
        except (FileNotFoundError, yaml.YAMLError) as e:
            self.logger.error(f"Failed to load joint parameters: {e}")
            return False
        
        # Extract parameters
        self.damping_kd = self.joint_params.get("damping_kd", [])
        self.num_joints = len(self.damping_kd)
        
        # Initialize robot command structure
        self.robot_cmd = datatypes.RobotCmd()
        self.robot_cmd.motor_names = [''] * self.num_joints
        self.robot_cmd.mode = [0] * self.num_joints
        self.robot_cmd.q = [0.0] * self.num_joints
        self.robot_cmd.dq = [0.0] * self.num_joints
        self.robot_cmd.tau = [0.0] * self.num_joints
        self.robot_cmd.Kp = [0.0] * self.num_joints
        self.robot_cmd.Kd = self.damping_kd
        
        return True
    
    def _init_robot_command(self):
        """Initialize robot command structure"""
    
    def on_start(self):
        """Called when the ability starts"""
        # Wait for valid robot state
        while not self.get_robot_state() and self.running:
            self.logger.warn("Waiting for robot state data")
            time.sleep(1.0)
        
        # Initialize with current robot state
        self.robot_cmd.motor_names = self.get_robot_state().motor_names
        self.logger.info("DampingController started")
    
    def on_main(self):
        """Main control loop"""
        rate = Rate(self.update_rate)
        while self.running:
            # Publish command
            self.robot.publishRobotCmd(self.robot_cmd)
            
            # Maintain update rate
            rate.sleep()
    
    def on_stop(self):
        """Called when the ability stops"""
        self.logger.info("DampingController stopped")
