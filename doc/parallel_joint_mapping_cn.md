# 并联关节映射说明 | [English](parallel_joint_mapping_en.md)

## 概述

LimX 人形机器人（HU-D03、HU-D04 系列）的**踝部**和**腰部**采用了并联四连杆机构设计。这导致在 RL 训练/部署代码和物理模型（MJCF XML）之间存在两套不同的关节命名。本文档解释其映射关系及二次开发时的注意事项。

## 什么是并联关节

踝部和腰部各使用两个电机（A、B），通过连杆机构协同驱动，实现等效的 pitch/roll 两自由度运动。

- **虚拟关节（PR 空间，Pitch-Roll）**：以运动语义命名（如 `ankle_pitch_joint`、`waist_roll_joint`），在 RL 策略中使用。
- **物理电机关节（AB 空间，A-B Motor）**：以实际电机位置命名（如 `A_achilles_joint`、`waist_A_joint`），在 MJCF XML 和硬件中使用。

## 关节命名对照表

### 踝部（Ankle）

| 虚拟关节（PR 空间） | 物理电机关节（AB 空间） | XML 中有 actuator |
|:---:|:---:|:---:|
| `left_ankle_pitch_joint` | `left_A_achilles_joint` | ✅（`ankle_A_left`） |
| `left_ankle_roll_joint` | `left_B_achilles_joint` | ✅（`ankle_B_left`） |
| `right_ankle_pitch_joint` | `right_A_achilles_joint` | ✅（`ankle_A_right`） |
| `right_ankle_roll_joint` | `right_B_achilles_joint` | ✅（`ankle_B_right`） |

> **注意**：XML 中 `ankle_pitch_joint` 和 `ankle_roll_joint` 作为运动学关节存在，但**没有 actuator**。

### 腰部（Waist）

| 虚拟关节（PR 空间） | 物理电机关节（AB 空间） | XML 中有 actuator |
|:---:|:---:|:---:|
| `waist_roll_joint` | `waist_A_joint` | ✅（`body_waist_A`） |
| `waist_pitch_joint` | `waist_B_joint` | ✅（`body_waist_B`） |

> **注意**：`waist_yaw_joint` 是独立的单电机关节，不涉及并联机构。

## SDK 转换机制

### 两种控制模式

LimX SDK 通过 `publishRobotCmd` 接口下发指令时，支持通过**标志位**配置每个关节是否需要 SDK 进行 PR↔AB 的转换。对应两种控制模式：

| 模式 | 标志位 | 说明 | 适用场景 |
|:---:|:---:|:---|:---|
| **PR 模式（Pitch-Roll Mode）** | 转换开启（默认） | SDK 自动将 PR 空间的指令转换为 AB 空间 | 标准 RL 部署、大多数二开场景 |
| **AB 模式（A-B Motor Mode）** | 转换关闭 | 用户直接下发 AB 空间的指令，SDK 不做转换 | 需要直接控制电机、自行实现运动学的高级场景 |

### PR 模式流程（默认）

```
RL 策略输出 (PR 空间)
    ↓ action_scale 缩放 + default_angle 偏移
部署代码计算目标关节位置 (PR 空间)
    ↓ publishRobotCmd()  [标志位：PR→AB 转换开启]
SDK 内部 PR → AB 转换
    ↓
实际电机执行 (AB 空间)
```

### AB 模式流程

```
用户自行计算电机指令 (AB 空间)
    ↓ publishRobotCmd()  [标志位：PR→AB 转换关闭]
SDK 直接透传
    ↓
实际电机执行 (AB 空间)
```

> ⚠️ **使用 AB 模式时**，用户需自行完成并联机构的正/逆运动学计算，确保下发的 A/B 电机位置满足机构约束。

## MJCF XML 中的结构

在 MuJoCo 仿真模型里，并联机构通过 `<equality>` 约束来实现：

```xml
<!-- 踝部：连杆末端约束到 ankle_roll_link -->
<connect name='A_achilles_connect_L' body1='left_ankle_roll_link'
         body2='left_A_achilles_rod_link' anchor='-0.035 0.033 0.0045'/>
<connect name='B_achilles_connect_L' body1='left_ankle_roll_link'
         body2='left_B_achilles_rod_link' anchor='-0.035 -0.033 0.0045'/>

<!-- 腰部：连杆末端约束到 waist_yaw_link -->
<connect name='waist_connect_A' body1='waist_yaw_link'
         body2='waist_A_rod_link' anchor='0.0639 0.06 0.0337'/>
<connect name='waist_connect_B' body1='waist_yaw_link'
         body2='waist_B_rod_link' anchor='0.0639 -0.06 0.0337'/>
```

MuJoCo 的等式约束求解器在仿真中自动维护并联机构的运动学关系。

## 部署参数配置

`walk_param.yaml` 和 `mimic_param.yaml` 中的参数数组按以下顺序排列（以 HU_D04_01 为例，共 31 维）：

```
索引  0- 5: 左腿 (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
索引  6-11: 右腿 (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
索引 12-14: 腰部 (waist_yaw, waist_roll, waist_pitch)
索引 15-16: 头部 (head_pitch, head_yaw)
索引 17-23: 左臂 (shoulder_pitch/roll/yaw, elbow, wrist_yaw/pitch/roll)
索引 24-30: 右臂 (shoulder_pitch/roll/yaw, elbow, wrist_yaw/pitch/roll)
```

> 默认 PR 模式下，所有参数（kp、kd、default_angle、action_scale）均在 PR 空间中定义。

## 二开注意事项

1. **PR 模式（推荐）**：通过 `publishRobotCmd` 下发指令并开启 PR→AB 转换标志位，SDK 自动处理转换。用户只需关注 PR 空间，与训练环境保持一致。

2. **AB 模式（高级）**：关闭转换标志位后，SDK 直接透传 AB 空间指令。需要自行实现并联运动学转换。适合需要精确控制电机的高级用户。

3. **自定义训练模型**：IsaacLab/IsaacGym 中的训练环境使用 PR 空间的关节定义（如 `ankle_pitch_joint`），训练出的策略直接输出 PR 空间的动作。部署时使用 PR 模式即可。

4. **直接操作 MuJoCo**：如果在 MuJoCo 仿真中直接控制 actuator，需注意 XML 中 actuator 绑定的是 AB 空间的关节，等同于 AB 模式。

5. **读取关节状态**：通过 `getRobotState()` 获取的关节位置和速度，其坐标空间与下发时所选模式一致（PR 模式返回 PR 空间值，AB 模式返回 AB 空间值）。

## FAQ

**Q：为什么 walk_param.yaml 里写的是 `ankle_pitch_joint`，但 XML 里的 actuator 是 `A_achilles_joint`？**

A：因为部署代码默认在 PR 空间中工作，SDK 通过转换标志位自动完成 PR→AB 的转换。XML 中的 actuator 直接驱动物理电机（AB 空间），但在 PR 模式下用户不需要直接操作它们。

**Q：我需要自己实现 PR↔AB 的转换吗？**

A：取决于你使用的模式。PR 模式下 SDK 自动转换，不需要。AB 模式下 SDK 不做转换，需要用户自行实现。

**Q：训练时的关节空间和部署时一致吗？**

A：使用 PR 模式部署时是一致的。训练和部署都在 PR 空间中进行。

**Q：什么时候需要用 AB 模式？**

A：当你需要直接控制底层电机、自定义并联运动学、或者进行电机级别的调试时。一般的 RL 部署场景使用默认的 PR 模式即可。
