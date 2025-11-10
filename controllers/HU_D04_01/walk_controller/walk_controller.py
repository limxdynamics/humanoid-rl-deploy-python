# Copyright information
#
# © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

import glob
import os
import time
import math
import yaml
import copy
import onnxruntime as ort
import numpy as np
from functools import partial
from scipy.spatial.transform import Rotation as R
import limxsdk.robot.Rate as Rate
import limxsdk.datatypes as datatypes
from limxsdk.ability.base_ability import BaseAbility
from limxsdk.ability.registry import register_ability


def sensor_joy_callback(sensor_joy: datatypes.SensorJoy):
    global joy_axes
    joy_axes = sensor_joy.axes

@register_ability("walk/controller")
class WalkController(BaseAbility):
    """Robot walking controller"""

    def initialize(self, config):
        """Initialize controller parameters"""
        
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Load joint parameters from YAML
        try:
            yaml_path = os.path.join(self.script_dir, 'walk_param.yaml')
            with open(yaml_path, 'r') as f:
                self.walking_param = yaml.safe_load(f)
            self.logger.info("Walking parameters loaded successfully")
        except (FileNotFoundError, yaml.YAMLError) as e:
            self.logger.error(f"Failed to load walking parameters: {e}")
            return False
          
        self.robot = self.get_robot_instance()
        self.update_rate = config.get("update_rate", 1000.0)
        
        # HumanoidRobotCfg/policy_dir
        policy_dir = self.walking_param.get("HumanoidRobotCfg", {}).get("policy_dir", "/policy/default/")
        self.model_dir = f'{self.script_dir}/{policy_dir}'
        self.logger.info(f"Policy path: {self.model_dir}")

        # HumanoidRobotCfg/control
        control_params = self.walking_param.get("HumanoidRobotCfg", {}).get("control", {})
        self.action_scale = control_params.get("action_scale", [])
        self.decimation = control_params.get("decimation", 10)
        self.user_torque_limit = control_params.get("user_torque_limit", [])
        self.kp = control_params.get("kp", [])
        self.kd = control_params.get("kd", [])
        self.default_angle = control_params.get("default_angle", [])

        # HumanoidRobotCfg/normalization
        normalization_params = self.walking_param.get("HumanoidRobotCfg", {}).get("normalization", {})
        self.clip_observations = normalization_params.get("clip_scales", {}).get("clip_observations", 100)
        self.clip_actions = normalization_params.get("clip_scales", {}).get("clip_actions", 100)
        self.lin_vel = normalization_params.get("obs_scales", {}).get("lin_vel", 1.0)
        self.ang_vel = normalization_params.get("obs_scales", {}).get("ang_vel", 0.25)
        self.dof_pos = normalization_params.get("obs_scales", {}).get("dof_pos", 1.0)
        self.dof_vel = normalization_params.get("obs_scales", {}).get("dof_vel", 0.05)
        self.height_measurements = normalization_params.get("obs_scales", {}).get("height_measurements", 5)
        
        # HumanoidRobotCfg/size
        size_params = self.walking_param.get("HumanoidRobotCfg", {}).get("size", {})
        self.actions_size = size_params.get("actions_size", 31)
        self.observations_size = size_params.get("observations_size", 100)
        self.obs_history_length = size_params.get("obs_history_length", 5)
        self.commands = np.zeros(3)
        self.gait_phase = 0.
        self.gait_freq = size_params.get("gait_freq", 0.85)
        self.gait_offset = size_params.get("gait_offset", 0.5)

        # Initialize robot command structure
        self.robot_cmd = datatypes.RobotCmd()
        self.robot_cmd.motor_names = [''] * self.actions_size
        self.robot_cmd.mode = [0] * self.actions_size
        self.robot_cmd.q = [0.0] * self.actions_size
        self.robot_cmd.dq = [0.0] * self.actions_size
        self.robot_cmd.tau = [0.0] * self.actions_size
        self.robot_cmd.Kp = self.kp
        self.robot_cmd.Kd = self.kd

        # Minimal joystick -> velocity
        self.max_vx = control_params.get("max_vx", 0.5)
        self.max_vy = control_params.get("max_vy", 0.5)
        self.max_vz = control_params.get("max_vz", 0.5)
        self._joy_axes = None

        self.robot.subscribeSensorJoy(sensor_joy_callback)
        self.logger.info("Subscribed SensorJoy for basic velocity command!!!")
        
        return self.initialize_onnx_models()
    
    def initialize_onnx_models(self):
        # Configure ONNX Runtime session options to optimize CPU usage
        session_options = ort.SessionOptions()
        # Limit the number of threads used for parallel computation within individual operators
        session_options.intra_op_num_threads = 1
        # Limit the number of threads used for parallel execution of different operators
        session_options.inter_op_num_threads = 1
        # Enable all possible graph optimizations to improve inference performance
        session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        # Disable CPU memory arena to reduce memory fragmentation
        session_options.enable_cpu_mem_arena = False
        # Disable memory pattern optimization to have more control over memory allocation
        session_options.enable_mem_pattern = False

        # Define execution providers to use CPU only, ensuring no GPU inference
        cpu_providers = ['CPUExecutionProvider']

        # Load ONNX model policy
        self.model_policy = f'{self.model_dir}/policy.onnx'
        self.policy_session = ort.InferenceSession(self.model_policy, sess_options=session_options, providers=cpu_providers)
        self.policy_input_names = [self.policy_session.get_inputs()[i].name for i in range(self.policy_session.get_inputs().__len__())]
        self.policy_output_names = [self.policy_session.get_outputs()[i].name for i in range(self.policy_session.get_outputs().__len__())]
        self.policy_input_shapes = [self.policy_session.get_inputs()[i].shape for i in range(self.policy_session.get_inputs().__len__())]
        self.policy_output_shapes = [self.policy_session.get_outputs()[i].shape for i in range(self.policy_session.get_outputs().__len__())]
        
        return True
    
    def compute_gait_observation(self):
        self.gait_freq = 0.8 + 0.15 * np.abs(self.commands[0])
        command_norm = np.linalg.norm(self.commands)
        
        if command_norm < 0.1:
            if self.gait_phase > 0.5:
                self.gait_phase = 0.5
            else:
                self.gait_phase = 0.0
        else:
            self.gait_phase = math.fmod(self.gait_phase + 0.01 * self.gait_freq, 1.0)
        
        # Return only the 5 gait-related observations
        return np.array([
            self.gait_freq,                      # gait frequency
            self.gait_offset,                    # gait offset
            0.12,                               # gait height(fixed value)
            math.sin(2 * math.pi * self.gait_phase),  # phase sine value
            math.cos(2 * math.pi * self.gait_phase)   # phase cosine value
        ])

    def _update_commands_from_joy(self):
        """Update the commands from the joystick."""
        self.commands[0] = np.clip(joy_axes[1], -self.max_vx, self.max_vx)
        self.commands[1] = np.clip(joy_axes[0], -self.max_vy, self.max_vy)
        self.commands[2] = np.clip(joy_axes[3], -self.max_vz, self.max_vz)
      
    def compute_observation(self):
        # Convert IMU orientation from quaternion to Euler angles (ZYX convention)
        imu_orientation = np.array(self.imu_data.quat)
        q_wi = R.from_quat(imu_orientation).as_euler('zyx')  # Quaternion to Euler ZYX conversion
        inverse_rot = R.from_euler('zyx', q_wi).inv().as_matrix()  # Get the inverse rotation matrix

        # Project the gravity vector (pointing downwards) into the body frame
        gravity_vector = np.array([0, 0, -1])  # Gravity in world frame (z-axis down)
        projected_gravity = np.dot(inverse_rot, gravity_vector)  # Transform gravity into body frame

        # Retrieve base angular velocity from the IMU data
        base_ang_vel = np.array(self.imu_data.gyro)

        # Retrieve joint positions and velocities from the robot state
        joint_positions = np.array(self.robot_state.q) - self.default_angle
        joint_velocities = np.array(self.robot_state.dq)

        # Retrieve the last actions that were applied to the robot
        actions = np.array(self.last_actions)
        
        # self.commands[0] = 0.5  # Placeholder for command inputs (e.g., desired velocities)

        gait_obs = self.compute_gait_observation()

        # Create the observation vector by concatenating various state variables:
        # - Base angular velocity (scaled)
        # - Projected gravity vector
        # - Joint positions (difference from initial angles, scaled)
        # - Joint velocities (scaled)
        # - Last actions applied to the robot
        obs = np.concatenate([
            base_ang_vel * self.ang_vel,  # Scaled base angular velocity
            projected_gravity,  # Projected gravity vector in body frame
            self.commands,  # Scaled command inputs
            joint_positions * self.dof_pos,  # Scaled joint positions
            joint_velocities * self.dof_vel,  # Scaled joint velocities
            actions,  # Last actions taken by the robot
            # gait_obs # Gait observations
        ])

        # Check if this is the first recorded observation
        if self.is_first_rec_obs:
            # Initialize the proprioceptive history buffer with zeros
            self.proprio_history_buffer = np.zeros(self.observations_size * self.obs_history_length)

            # Fill the proprioceptive history buffer with the current observation for the entire history length
            for i in range(self.obs_history_length):
                self.proprio_history_buffer[i * self.observations_size:(i + 1) * self.observations_size] = obs

            # Update the flag to indicate that the first observation has been processed
            self.is_first_rec_obs = False
        
        # Shift the existing proprioceptive history buffer to the right
        self.proprio_history_buffer[self.observations_size:] = self.proprio_history_buffer[:-self.observations_size]

        # Add the current observation to the end of the proprioceptive history buffer
        self.proprio_history_buffer[:self.observations_size] = obs

        # Clip the observation values to within the specified limits for stability
        self.observations = np.clip(
            obs, 
            -self.clip_observations,  # Lower limit for clipping
            self.clip_observations    # Upper limit for clipping
        )

    def compute_actions(self):
        """
        Computes the actions based on the current observations using the policy session.
        """
        # Concatenate observations into a single tensor and convert to float32
        # command_filtered = np.zeros(3)
        # input_tensor = np.concatenate([self.observations, command_filtered], axis=0)
        input_tensor = self.proprio_history_buffer
        # Add the batch dimension
        input_tensor = input_tensor[np.newaxis, :]
        input_tensor = input_tensor.astype(np.float32)
        
        # Create a dictionary of inputs for the policy session
        inputs = {self.policy_input_names[0]: input_tensor}
        
        # Run the policy session and get the output
        output = self.policy_session.run(self.policy_output_names, inputs)
        
        # Flatten the output and store it as actions
        self.actions = np.array(output).flatten()
        
    def on_start(self):
        """Called when the ability starts"""
        # Wait for valid robot state
        while not self.get_robot_state() and self.running:
            self.logger.warn("Waiting for robot state data")
            time.sleep(1.0)
        
        # Initialize variables for actions, observations, and commands
        self.actions = np.zeros(self.actions_size)
        self.observations = np.zeros(self.observations_size)
        self.last_actions = np.zeros(self.actions_size)
        self.loop_count = 0  # loop iteration count
        self.imu_data = None
        self.robot_state = None
        self.is_first_rec_obs = True
        self.logger.info("WalkingController started")

    def on_main(self):
        """Main control loop"""
        rate = Rate(self.update_rate)
        t_s = time.perf_counter()
        while self.running:
            self._update_commands_from_joy()
            self.robot_state = copy.deepcopy(self.get_robot_state())
            self.imu_data = copy.deepcopy(self.get_imu_data())
            
            # Rotate quaternion elements (w,x,y,z -> x,y,z,w)
            self.imu_data.quat = self.imu_data.quat[1:] + self.imu_data.quat[:1] 
            
            # Execute actions every 'decimation' iterations
            if self.loop_count % self.decimation == 0:
                self.compute_observation()
                self.compute_actions()
                
                # Clip the actions within predefined limits
                action_min = -self.clip_actions
                action_max = self.clip_actions
                self.actions = np.clip(self.actions, action_min, action_max)

                # Save the last action for reference
                self.last_actions = self.actions
            
            # Iterate over the joints and set commands based on actions
            joint_pos = np.array(self.robot_state.q)
            joint_vel = np.array(self.robot_state.dq)
            soft_torque_limit = 0.95
            motor_names = self.robot_state.motor_names
            for i in range(len(joint_pos)):
                # Compute the limits for the action based on joint position and velocity
                action_min = (joint_pos[i] - self.default_angle[i] + (self.kd[i] * joint_vel[i] - self.user_torque_limit[i] * soft_torque_limit) / self.kp[i])
                action_max = (joint_pos[i] - self.default_angle[i] + (self.kd[i] * joint_vel[i] + self.user_torque_limit[i] * soft_torque_limit) / self.kp[i])

                # Clip action within limits
                self.actions[i] = max(action_min / self.action_scale[i], min(action_max / self.action_scale[i], self.actions[i]))

                # Compute the desired joint position and set it
                pos_des = self.actions[i] * self.action_scale[i] + self.default_angle[i]
                
                # Set joint command
                self.robot_cmd.q[i] = pos_des
                self.robot_cmd.dq[i] = 0
                self.robot_cmd.tau[i] = 0
                self.robot_cmd.Kp[i] = self.kp[i]
                self.robot_cmd.Kd[i] = self.kd[i]
                self.robot_cmd.motor_names[i] = motor_names[i]
            
            # Publish robot command
            self.robot.publishRobotCmd(self.robot_cmd)
            
            # Increment the loop count
            self.loop_count += 1

            # Compute inference fps
            fps = self.loop_count / (time.perf_counter() - t_s)
            print(f"\rInference FPS: {fps:.2f}", end="")
            
            # Maintain update rate
            rate.sleep()

    def on_stop(self):
        """Called when the ability stops"""
        self.logger.info("WalkingController stopped")

if __name__ == "__main__":
    walking_controller = WalkingController()
    walking_controller.initialize()
    walking_controller.on_start()
    walking_controller.on_main()
    walking_controller.on_stop()