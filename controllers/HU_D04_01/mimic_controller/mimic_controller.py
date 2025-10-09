# Copyright information
#
# © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.

import os
import time
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

@register_ability("mimic/controller")
class MimicController(BaseAbility):
    """Robot mimic controller with smooth transition"""

    def initialize(self, config):
        """Initialize controller parameters"""
        
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Load joint parameters from YAML
        try:
            yaml_path = os.path.join(self.script_dir, 'mimic_param.yaml')
            with open(yaml_path, 'r') as f:
                self.mimic_param = yaml.safe_load(f)
            self.logger.info("Mimic parameters loaded successfully")
        except (FileNotFoundError, yaml.YAMLError) as e:
            self.logger.error(f"Failed to load mimic parameters: {e}")
            return False
          
        self.robot = self.get_robot_instance()
        self.update_rate = config.get("update_rate", 1000.0)
        
        # HumanoidRobotCfg/policy_dir
        policy_dir = self.mimic_param.get("HumanoidRobotCfg", {}).get("policy_dir", "/policy/default/")
        self.model_dir = f'{self.script_dir}/{policy_dir}'
        self.logger.info(f"Policy path: {self.model_dir}")
        
        # HumanoidRobotCfg/motion_frames
        self.motion_frames = self.mimic_param.get("HumanoidRobotCfg", {}).get("motion_frames", 820)
        
        # HumanoidRobotCfg/control
        control_params = self.mimic_param.get("HumanoidRobotCfg", {}).get("control", {})
        self.action_scale = control_params.get("action_scale", [])
        self.decimation = control_params.get("decimation", 10)
        self.user_torque_limit = control_params.get("user_torque_limit", [])
        self.kp = control_params.get("kp", [])
        self.kd = control_params.get("kd", [])
        
        # HumanoidRobotCfg/normalization
        normalization_params = self.mimic_param.get("HumanoidRobotCfg", {}).get("normalization", {})
        self.clip_observations = normalization_params.get("clip_scales", {}).get("clip_observations", 100)
        self.clip_actions = normalization_params.get("clip_scales", {}).get("clip_actions", 100)
        self.lin_vel = normalization_params.get("obs_scales", {}).get("lin_vel", 1.0)
        self.ang_vel = normalization_params.get("obs_scales", {}).get("ang_vel", 0.25)
        self.dof_pos = normalization_params.get("obs_scales", {}).get("dof_pos", 1.0)
        self.dof_vel = normalization_params.get("obs_scales", {}).get("dof_vel", 0.05)
        self.height_measurements = normalization_params.get("obs_scales", {}).get("height_measurements", 5)
        
        # HumanoidRobotCfg/size
        size_params = self.mimic_param.get("HumanoidRobotCfg", {}).get("size", {})
        self.motion_ref_size = size_params.get("motion_ref_size", 39)
        self.actions_size = size_params.get("actions_size", 31)
        self.observations_size = size_params.get("observations_size", 100)
        self.obs_history_length = size_params.get("obs_history_length", 10)
        self.encoder_output_size = size_params.get("encoder_output_size", [3, 32])

        # Initialize robot command structure
        self.robot_cmd = datatypes.RobotCmd()
        self.robot_cmd.motor_names = [''] * self.actions_size
        self.robot_cmd.mode = [0] * self.actions_size
        self.robot_cmd.q = [0.0] * self.actions_size
        self.robot_cmd.dq = [0.0] * self.actions_size
        self.robot_cmd.tau = [0.0] * self.actions_size
        self.robot_cmd.Kp = self.kp
        self.robot_cmd.Kd = self.kd
        
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
        
        # Load ONNX model lin_encoder
        self.model_lin_encoder = f'{self.model_dir}/lin_encoder.onnx'
        self.lin_encoder_session = ort.InferenceSession(self.model_lin_encoder, sess_options=session_options, providers=cpu_providers)
        self.lin_encoder_input_names = [self.lin_encoder_session.get_inputs()[i].name for i in range(self.lin_encoder_session.get_inputs().__len__())]
        self.lin_encoder_output_names = [self.lin_encoder_session.get_outputs()[i].name for i in range(self.lin_encoder_session.get_outputs().__len__())]
        self.lin_encoder_input_shapes = [self.lin_encoder_session.get_inputs()[i].shape for i in range(self.lin_encoder_session.get_inputs().__len__())]
        self.lin_encoder_output_shapes = [self.lin_encoder_session.get_outputs()[i].shape for i in range(self.lin_encoder_session.get_outputs().__len__())]
        
        # Load ONNX model priv_encoder
        self.model_priv_encoder = f'{self.model_dir}/priv_encoder.onnx'
        self.priv_encoder_session = ort.InferenceSession(self.model_priv_encoder, sess_options=session_options, providers=cpu_providers)
        self.priv_encoder_input_names = [self.priv_encoder_session.get_inputs()[i].name for i in range(self.priv_encoder_session.get_inputs().__len__())]
        self.priv_encoder_output_names = [self.priv_encoder_session.get_outputs()[i].name for i in range(self.priv_encoder_session.get_outputs().__len__())]
        self.priv_encoder_input_shapes = [self.priv_encoder_session.get_inputs()[i].shape for i in range(self.priv_encoder_session.get_inputs().__len__())]
        self.priv_encoder_output_shapes = [self.priv_encoder_session.get_outputs()[i].shape for i in range(self.priv_encoder_session.get_outputs().__len__())]
        
        return True
      
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
        joint_positions = np.array(self.robot_state.q)
        joint_velocities = np.array(self.robot_state.dq)

        # Retrieve the last actions that were applied to the robot
        actions = np.array(self.last_actions)
        
        motion_phase = np.zeros(1)
        motion_phase[0] = self.motion_iter / self.motion_frames
        
        if motion_phase[0] >= 1.0:
            motion_phase[0] = 1.0

        # Create the observation vector by concatenating various state variables:
        # - Base angular velocity (scaled)
        # - Projected gravity vector
        # - Joint positions (difference from initial angles, scaled)
        # - Joint velocities (scaled)
        # - Last actions applied to the robot
        # - motion_phase
        obs = np.concatenate([
            base_ang_vel * self.ang_vel,  # Scaled base angular velocity
            projected_gravity,  # Projected gravity vector in body frame
            joint_positions * self.dof_pos,  # Scaled joint positions
            joint_velocities * self.dof_vel,  # Scaled joint velocities
            actions,  # Last actions taken by the robot
            motion_phase
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
    
    def compute_lin_encoder(self):
        """
        Computes the encoder output based on the proprioceptive history buffer.

        This method first concatenates the proprioceptive history buffer into a single input tensor.
        Then it converts the input tensor to the float32 data type. After that, it creates a dictionary
        of inputs for the encoder session and runs the encoder session to get the output. Finally,
        it flattens the output and stores it as the encoder output.
        """
        # Concatenate the proprioceptive history buffer into a single tensor and convert to float32
        input_tensor = np.concatenate([self.proprio_history_buffer], axis=0)
        input_tensor = input_tensor.astype(np.float32)

        # Create a dictionary of inputs for the encoder session
        inputs = {self.lin_encoder_input_names[0]: input_tensor}
      
        # Run the encoder session and get the output
        output = self.lin_encoder_session.run(self.lin_encoder_output_names, inputs)

        # Flatten the output and store it as the encoder output
        self.lin_encoder_out = np.array(output).flatten()
    
    def compute_priv_encoder(self):
        """
        Computes the encoder output based on the proprioceptive history buffer.

        This method first concatenates the proprioceptive history buffer into a single input tensor.
        Then it converts the input tensor to the float32 data type. After that, it creates a dictionary
        of inputs for the encoder session and runs the encoder session to get the output. Finally,
        it flattens the output and stores it as the encoder output.
        """
        # Concatenate the proprioceptive history buffer into a single tensor and convert to float32
        input_tensor = np.concatenate([self.proprio_history_buffer], axis=0)
        input_tensor = input_tensor.astype(np.float32)

        # Create a dictionary of inputs for the encoder session
        inputs = {self.priv_encoder_input_names[0]: input_tensor}

        # Run the encoder session and get the output
        output = self.priv_encoder_session.run(self.priv_encoder_output_names, inputs)

        # Flatten the output and store it as the encoder output
        self.priv_encoder_out = np.array(output).flatten()

    def compute_actions(self):
        """
        Computes the actions based on the current observations using the policy session.
        """
        # Concatenate observations into a single tensor and convert to float32
        command_filtered = np.zeros(3)
        input_tensor = np.concatenate([self.observations, command_filtered, self.lin_encoder_out, self.priv_encoder_out], axis=0)
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
        self.lin_encoder_out = np.zeros(self.encoder_output_size[0])
        self.priv_encoder_out = np.zeros(self.encoder_output_size[1])
        self.actions = np.zeros(self.actions_size)
        self.observations = np.zeros(self.observations_size)
        self.last_actions = np.zeros(self.actions_size)
        self.loop_count = 0  # loop iteration count
        self.motion_iter = 0
        self.imu_data = None
        self.robot_state = None
        self.is_first_rec_obs = True
        self.logger.info("MimicController started")

    def on_main(self):
        """Main control loop"""
        rate = Rate(self.update_rate)
        while self.running:
            self.robot_state = copy.deepcopy(self.get_robot_state())
            self.imu_data = copy.deepcopy(self.get_imu_data())
            
            # Rotate quaternion elements (w,x,y,z -> x,y,z,w)
            self.imu_data.quat = self.imu_data.quat[1:] + self.imu_data.quat[:1] 
            
            # Execute actions every 'decimation' iterations
            if self.loop_count % self.decimation == 0:
                self.compute_observation()
                self.compute_lin_encoder()
                self.compute_priv_encoder()
                self.compute_actions()
                
                # Clip the actions within predefined limits
                action_min = -self.clip_actions
                action_max = self.clip_actions
                self.actions = np.clip(self.actions, action_min, action_max)

                # Save the last action for reference
                self.last_actions = self.actions
                
                self.motion_iter += 1
            
            # Iterate over the joints and set commands based on actions
            joint_pos = np.array(self.robot_state.q)
            joint_vel = np.array(self.robot_state.dq)
            soft_torque_limit = 0.95
            motor_names = self.robot_state.motor_names
            for i in range(len(joint_pos)):
                # Compute the limits for the action based on joint position and velocity
                action_min = (joint_pos[i] + (self.kd[i] * joint_vel[i] - self.user_torque_limit[i] * soft_torque_limit) / self.kp[i])
                action_max = (joint_pos[i] + (self.kd[i] * joint_vel[i] + self.user_torque_limit[i] * soft_torque_limit) / self.kp[i])

                # Clip action within limits
                self.actions[i] = max(action_min / self.action_scale[i], min(action_max / self.action_scale[i], self.actions[i]))

                # Compute the desired joint position and set it
                pos_des = self.actions[i] * self.action_scale[i]
                
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
            
            # Maintain update rate
            rate.sleep()

    def on_stop(self):
        """Called when the ability stops"""
        self.logger.info("MimicController stopped")