# Parallel Joint Mapping Guide | [中文](parallel_joint_mapping_cn.md)

## Overview

LimX humanoid robots (HU-D03, HU-D04 series) use **parallel four-bar linkage mechanisms** in the **ankle** and **waist** joints. This results in two different naming conventions between the RL training/deployment code and the physical model (MJCF XML). This document explains the mapping relationship and important notes for secondary development.

## What Are Parallel Joints

The ankle and waist each use two motors (A, B) that work together through a linkage mechanism to achieve equivalent pitch/roll two-DOF motion.

- **Virtual joints (PR space, Pitch-Roll)**: Named by motion semantics (e.g., `ankle_pitch_joint`, `waist_roll_joint`), used in RL policies.
- **Physical motor joints (AB space, A-B Motor)**: Named by actual motor position (e.g., `A_achilles_joint`, `waist_A_joint`), used in MJCF XML and hardware.

## Joint Naming Reference Table

### Ankle

| Virtual Joint (PR Space) | Physical Motor Joint (AB Space) | Has Actuator in XML |
|:---:|:---:|:---:|
| `left_ankle_pitch_joint` | `left_A_achilles_joint` | ✅ (`ankle_A_left`) |
| `left_ankle_roll_joint` | `left_B_achilles_joint` | ✅ (`ankle_B_left`) |
| `right_ankle_pitch_joint` | `right_A_achilles_joint` | ✅ (`ankle_A_right`) |
| `right_ankle_roll_joint` | `right_B_achilles_joint` | ✅ (`ankle_B_right`) |

> **Note**: `ankle_pitch_joint` and `ankle_roll_joint` exist as kinematic joints in XML but have **no actuators**.

### Waist

| Virtual Joint (PR Space) | Physical Motor Joint (AB Space) | Has Actuator in XML |
|:---:|:---:|:---:|
| `waist_roll_joint` | `waist_A_joint` | ✅ (`body_waist_A`) |
| `waist_pitch_joint` | `waist_B_joint` | ✅ (`body_waist_B`) |

> **Note**: `waist_yaw_joint` is a standalone single-motor joint, not part of the parallel mechanism.

## SDK Conversion Mechanism

### Two Control Modes

When sending commands through the LimX SDK via `publishRobotCmd`, a **flag** can be configured per joint to control whether the SDK performs PR↔AB conversion. This corresponds to two control modes:

| Mode | Flag | Description | Use Case |
|:---:|:---:|:---|:---|
| **PR Mode (Pitch-Roll Mode)** | Conversion enabled (default) | SDK automatically converts PR space commands to AB space | Standard RL deployment, most dev scenarios |
| **AB Mode (A-B Motor Mode)** | Conversion disabled | User sends AB space commands directly, SDK passes through | Direct motor control, custom kinematics |

### PR Mode Flow (Default)

```
RL Policy Output (PR Space)
    ↓ action_scale scaling + default_angle offset
Deployment Code Computes Target Joint Positions (PR Space)
    ↓ publishRobotCmd()  [Flag: PR→AB conversion ON]
SDK Internal PR → AB Conversion
    ↓
Physical Motor Execution (AB Space)
```

### AB Mode Flow

```
User Computes Motor Commands (AB Space)
    ↓ publishRobotCmd()  [Flag: PR→AB conversion OFF]
SDK Direct Pass-through
    ↓
Physical Motor Execution (AB Space)
```

> ⚠️ **When using AB Mode**, users must perform forward/inverse kinematics of the parallel mechanism themselves, ensuring the A/B motor positions satisfy mechanical constraints.

## MJCF XML Structure

In MuJoCo simulation models, parallel mechanisms are implemented through `<equality>` constraints:

```xml
<!-- Ankle: rod endpoints constrained to ankle_roll_link -->
<connect name='A_achilles_connect_L' body1='left_ankle_roll_link'
         body2='left_A_achilles_rod_link' anchor='-0.035 0.033 0.0045'/>
<connect name='B_achilles_connect_L' body1='left_ankle_roll_link'
         body2='left_B_achilles_rod_link' anchor='-0.035 -0.033 0.0045'/>

<!-- Waist: rod endpoints constrained to waist_yaw_link -->
<connect name='waist_connect_A' body1='waist_yaw_link'
         body2='waist_A_rod_link' anchor='0.0639 0.06 0.0337'/>
<connect name='waist_connect_B' body1='waist_yaw_link'
         body2='waist_B_rod_link' anchor='0.0639 -0.06 0.0337'/>
```

MuJoCo's equality constraint solver automatically maintains the kinematics of the parallel mechanism during simulation.

## Deployment Parameter Configuration

Parameter arrays in `walk_param.yaml` and `mimic_param.yaml` follow this order (HU_D04_01, 31 DOF):

```
Index  0- 5: Left leg  (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
Index  6-11: Right leg (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
Index 12-14: Waist     (waist_yaw, waist_roll, waist_pitch)
Index 15-16: Head      (head_pitch, head_yaw)
Index 17-23: Left arm  (shoulder_pitch/roll/yaw, elbow, wrist_yaw/pitch/roll)
Index 24-30: Right arm (shoulder_pitch/roll/yaw, elbow, wrist_yaw/pitch/roll)
```

> In default PR Mode, all parameters (kp, kd, default_angle, action_scale) are defined in PR space.

## Notes for Secondary Development

1. **PR Mode (Recommended)**: Send commands via `publishRobotCmd` with the PR→AB conversion flag enabled. The SDK handles conversion automatically, keeping things consistent with the training environment.

2. **AB Mode (Advanced)**: Disable the conversion flag to send AB space commands directly. You must implement parallel kinematics conversion yourself. Suitable for advanced users requiring precise motor-level control.

3. **Custom Training Models**: Training environments in IsaacLab/IsaacGym use PR space joint definitions (e.g., `ankle_pitch_joint`). Trained policies output actions in PR space, so deploy with PR Mode.

4. **Direct MuJoCo Control**: When directly controlling actuators in MuJoCo simulation, note that XML actuators are bound to AB space joints — this is equivalent to AB Mode.

5. **Reading Joint States**: Joint positions and velocities obtained via `getRobotState()` are in the same coordinate space as the selected mode (PR Mode returns PR space values, AB Mode returns AB space values).

## FAQ

**Q: Why does walk_param.yaml reference `ankle_pitch_joint`, but XML actuators use `A_achilles_joint`?**

A: The deployment code operates in PR space by default, and the SDK converts via the flag setting. XML actuators directly drive physical motors (AB space), but in PR Mode users don't need to interact with them directly.

**Q: Do I need to implement PR↔AB conversion myself?**

A: It depends on your mode. In PR Mode, the SDK converts automatically — no action needed. In AB Mode, the SDK passes through directly, so you must implement conversion yourself.

**Q: Is the joint space consistent between training and deployment?**

A: Yes, when deploying in PR Mode. Both training and deployment use PR space consistently.

**Q: When should I use AB Mode?**

A: When you need direct motor control, custom parallel kinematics, or motor-level debugging. For standard RL deployment, use the default PR Mode.
