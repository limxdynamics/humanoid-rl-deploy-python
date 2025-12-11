# English | [中文](README_cn.md)

# Deployment of Training Results

### 1. Create a Workspace

 You can create an RL deployment development workspace by following these steps:

- Open a Bash terminal.

- Create a new directory to store the workspace. For example, create a directory named "limx_ws" in the user's home directory:
  ```Bash
  mkdir -p ~/limx_ws
  ```
  
- Download the MuJoCo simulator:
  ```Bash
  cd ~/limx_ws
  git clone --recurse git@github.com:limxdynamics/humanoid-mujoco-sim.git
  ```
  
- Download the motion control algorithm:
  ```Bash
  cd ~/limx_ws
  git clone git@github.com:limxdynamics/humanoid-rl-deploy-python.git
  ```
  
- Set the robot model: If it is not set yet, follow these steps:
  - List available robot types with the Shell command `tree -L 1 humanoid-rl-deploy-python/controllers` ：
    
    ```
    cd ~/limx_ws
    tree -L 1 humanoid-rl-deploy-python/controllers
    humanoid-rl-deploy-python/controllers
    ├── HU_D03_03
    └── HU_D04_01
    
    ```
    
  - Taking `HU_D04_01` (replace with your actual robot type) as an example, set the robot model type:
    
    ```
    echo 'export ROBOT_TYPE=HU_D04_01' >> ~/.bashrc && source ~/.bashrc
    ```

### 2. Simulation Debugging

- Run the MuJoCo simulator (Python 3.8 or higher is recommended):

  - Open a Bash terminal.

  - Install the motion control development library:
    - For Linux x86_64 environment:
    
      ```bash
      cd ~/limx_ws
      pip install humanoid-mujoco-sim/limxsdk-lowlevel/python3/amd64/limxsdk-*-py3-none-any.whl
      ```
    
    - For Linux aarch64 environment:
    
      ```bash
      cd ~/limx_ws
      pip install humanoid-mujoco-sim/limxsdk-lowlevel/python3/aarch64/limxsdk-*-py3-none-any.whl
      ```
    
  - Run the MuJoCo simulator:
    
    ```bash
    cd ~/limx_ws
    python humanoid-mujoco-sim/simulator.py
    ```

- Run the algorithm:

  - Open a Bash terminal.

  - Run the algorithm:
    
    ```bash
    python humanoid-rl-deploy-python/main.py
    ```
    
    ![](doc/simulator.gif)
  
- Virtual Joystick: You can use a virtual joystick to control the robot during the simulation. Here are the specific steps:

  - Open a Bash terminal.

  - Run the virtual joystick:

    ```
    ~/limx_ws/humanoid-mujoco-sim/robot-joystick/robot-joystick
    ```
    
    ![](doc/robot-joystick.png)


  - Now you can use the virtual joystick to control the robot.
  
    | **Button** | **Mode**         | **Description**                                                    |
    | -------- | ---------------- | ----------------------------------------------------------- |
    | L1+Y     | Switch to Stand Mode   | If the robot cannot stand, click "Reset" in the MuJoCo interface to reset it. |
    | L1+A     | Switch to Greeting Mode |                                                             |
    | R1+B     | Switch to Walk Mode|  **You can train your model for this mode!**                                 |


- Deploy you own model for robot walking (Optional)
  - In the [humanoid-rl-isaaclab](https://github.com/limxdynamics/humanoid-rl-isaaclab) project, run the inference code and export the model, you will find a generated `policy.onnx` in the checkpoint folder:
    ``` sh
    python scripts/rsl_rl/play.py --task LimX-Oli-31dof-Velocity --checkpoint path-to-model
    ```
  
  - Add the `policy.onnx` to the folder 
    ``` sh
    limx_ws/humanoid-rl-deploy-python/controllers/HU_D04_01/walk_controller/policy/default
    ```

  - Follow the steps above to run the MuJoCo simulator and control the robot.

  - Make sure your model runs perfectly in the simulation before deploying it in the real robot!!!
  

### 3. Real Robot Debugging

- Set your computer's IP: Ensure your computer is connected to the robot's external network port. Set your computer's IP address to `10.192.1.200` and verify connectivity with the Shell command `ping 10.192.1.2`. Configure your development computer's IP as shown below:

  ![img](doc/ip.png)

- Robot preparation:

  - Hang the robot using the hooks on its left and right shoulders.
  - Power on the robot, then press the `right joystick` button on the remote control to activate the robot's motors.
  - Press `R1 + DOWN` on the remote control to switch to developer mode. In this mode, users can develop their own motion control algorithms. (This mode setting persists after rebooting. To exit developer mode, press `R1 + LEFT`.)

- Deploy and run on the real robot. In the Bash terminal, start the control algorithm with the following Shell command:

  ```bash
  python humanoid-rl-deploy-python/main.py 10.192.1.2
  ```
  
- Now you can press `L1 + △ (Y)` on the remote control to make the robot stand up.

- Press `L1 + X (A)` on the remote control to make the robot wave.

- Press `R1 + 〇 (B)` on the remote control to make the robot walk.
