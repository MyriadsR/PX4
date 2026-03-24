## VTOL 控制框架文字描述
### 一、总体结构

系统核心为 **VTOL姿态控制器（VTOL Attitude Controller）**，它在不同飞行模式下（多旋翼模式、固定翼模式、过渡模式）协调姿态与速率控制，并生成最终的**舵面/电机控制量 ( $\mathbf{u}$ )**。
控制输出经过 **Mixer（混控器）**，将虚拟控制量转换为实际舵机、电机等执行机构的控制命令。

---

### 二、输入部分

VTOL姿态控制器接收来自两种位置控制器的虚拟姿态指令（setpoint）：

1. **MC Position Controller（多旋翼位置控制器）**
   输出：`mc_virtual_attitude_setpoint`，用于多旋翼模式下的姿态目标。
2. **FW Position Controller（固定翼位置控制器）**
   输出：`fw_virtual_attitude_setpoint`，用于固定翼模式下的姿态目标。

这些虚拟姿态指令提供飞行器在不同模式下的目标姿态信息。

---

### 三、姿态与速率控制层

VTOL姿态控制器根据当前飞行模式选择对应的控制路径：

#### 1. 姿态控制（Attitude Control）

* **MC Attitude Controller**：在多旋翼模式下生效，输入姿态指令 ( $\Psi_{sp}$ )，输出角速度指令 ( $\omega_{sp}$ )。
* **FW Attitude Controller**：在固定翼模式或过渡模式下生效，同样输出角速度指令 ( $\omega_{sp}$ )。

> *注：对于倾转旋翼（tailsitter）等机型，在过渡阶段也使用MC姿态控制器。*

#### 2. 速率控制（Rate Control）

* **MC Rates Controller**：接收MC姿态控制器生成的角速度指令 ( $\omega_{sp}$ )，输出虚拟的电机控制量 `actuator_controls_virtual_mc`。
* **FW Rates Controller**：接收FW姿态控制器生成的角速度指令 ( $\omega_{sp}$ )，输出虚拟的固定翼舵面控制量 `actuator_controls_virtual_fw`。

---

### 四、控制信号合成

VTOL姿态控制器将上述不同模式下的控制输出（`actuator_controls_virtual_mc` 与 `actuator_controls_virtual_fw`）进行融合，生成：

* `actuator_controls_0`
* `actuator_controls_1`

这些信号随后进入 **Mixer**，得到最终的执行机构控制量：
[
$\mathbf{u} = [u_1, u_2, \dots, u_n]^T$
]
用于驱动电机、舵面等。

---

### 五、符号说明

| 符号             | 含义                         |
| -------------- | -------------------------- |
| ( $\Psi$ )       | 姿态向量（Attitude vector）      |
| ( $\omega$ )     | 机体系角速度向量（Body rate vector） |
| ( $\mathbf{u}$ ) | 执行机构输出（Actuators output）   |
| ( $(x)_{sp}$ )   | 对变量 ( x ) 的设定值（Setpoint）   |
| MC             | Multicopter，多旋翼            |
| FW             | Fixed-wing，固定翼             |

---

### 六、控制逻辑总结

1. 位置控制器 → 生成虚拟姿态设定值。
2. 姿态控制器（MC/FW） → 生成角速度设定值。
3. 速率控制器（MC/FW） → 生成虚拟控制量。
4. VTOL姿态控制器 → 融合多旋翼与固定翼控制输出。
5. Mixer → 计算最终的执行机构命令。

## VTOL 控制框架相关代码
VTOL Attitude Controller 总览

  - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/vtol_att_control_main.cpp:68-105 构造函数根据 VT_TYPE 实例化尾座/倾转/标准控制器，并注册虚
    拟推力、力矩的订阅与发布。
  - 主循环 Run() 位于 vtol_att_control_main.cpp:317-535，按当前模式挑选多旋翼或固定翼的虚拟输入，调用具体机型的 update_*/fill_actuator_outputs()，最终
    发布 _vehicle_thrust_setpoint{0,1} 与 _vehicle_torque_setpoint{0,1}。

  输入：MC/FW 位置控制器产生虚拟姿态设定值

  - 多旋翼位置控制器 PX4_project/PX4-Autopilot/src/modules/mc_pos_control/MulticopterPositionControl.cpp:48-57 在 VTOL 模式下将姿态设定值发布到
    mc_virtual_attitude_setpoint，详细运行和发布在 MulticopterPositionControl.cpp:560-616。
  - 固定翼位置/纵向控制器 PX4_project/PX4-Autopilot/src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp:67-78 在 VTOL 模式下
    改发 fw_virtual_attitude_setpoint，具体姿态/推力设定值的生成与发布见 FwLateralLongitudinalControl.cpp:280-315。

  姿态控制层（Attitude Control）

  - 多旋翼姿态控制器 PX4_project/PX4-Autopilot/src/modules/mc_att_control/mc_att_control_main.cpp:56-200 读取 mc_virtual_attitude_setpoint，把姿态误差
    转成角速度设定值，并在 mc_att_control_main.cpp:342-365 发布 vehicle_rates_setpoint。
  - 固定翼姿态控制（VTOL 也使用）在 PX4_project/PX4-Autopilot/src/modules/fw_att_control/FixedwingAttitudeControl.cpp:201-316，把
    fw_virtual_attitude_setpoint 转为滚/俯仰速率需求；如为尾座机型，FixedwingAttitudeControl.cpp:217-253 做姿态坐标转换。

  速率控制层（Rate Control）

  - 多旋翼速率控制器 PX4_project/PX4-Autopilot/src/modules/mc_rate_control/MulticopterRateControl.cpp:50-265 订阅 vehicle_rates_setpoint，输出虚拟推
    力/力矩设定值，并在 MulticopterRateControl.cpp:258-264 发布 vehicle_thrust_setpoint_virtual_mc 与 vehicle_torque_setpoint_virtual_mc。
  - 固定翼速率控制器 PX4_project/PX4-Autopilot/src/modules/fw_rate_control/FixedwingRateControl.cpp:248-420 读取角速度设定值，处理尾座帧转换（同
    文件的 :248-356、:420-432），并把控制量写到 vehicle_thrust_setpoint / vehicle_torque_setpoint，其中在 VTOL 框架下被 vtol_att_control 订阅为
    vehicle_*_virtual_fw。

  控制信号融合与模式切换

  - VtolType 基类 PX4_project/PX4-Autopilot/src/modules/vtol_att_control/vtol_type.cpp:60-565 维护模式 (ROTARY_WING/FIXED_WING/TRANSITION_*) 以及过渡
    计时、Quad‐chute 安全条件。
  - 具体机型（如尾座）在 PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:59-365 中将 _vehicle_torque/
    thrust_setpoint_virtual_{mc,fw} 组合成矩阵 0/1 (_torque_setpoint_0 等)，并平滑混合推力、姿态。倾转与标准机型逻辑分别在 tiltrotor.cpp 与
    standard.cpp 同名段落实现。

  最终执行层（Mixer / Control Allocator）

  - 控制分配主模块 PX4_project/PX4-Autopilot/src/modules/control_allocator/ControlAllocator.cpp:381-417 订阅 vehicle_torque_setpoint（矩阵 0）及可
    选 vehicle_torque_setpoint1（矩阵 1），基于所选 ActuatorEffectiveness* 类生成 actuator_controls_*，此处 VTOL 选择逻辑见 ControlAllocator.cpp:220-
    235。
  - 各机型的效能矩阵定义在 PX4_project/PX4-Autopilot/src/modules/control_allocator/VehicleActuatorEffectiveness/
    ActuatorEffectiveness{Standard,Tiltrotor,Tailsitter}VTOL.cpp:35-120（对应 .hpp 描述），负责将矩阵 0/1 映射到真实的电机、舵面命令。
  - vtol_att_control 将组合后的推力/力矩发布到 orb，由控制分配器和后续 mixer 最终驱动执行机构，与 VTOLcontrol.md 中的 “Mixer” 匹配。

  这样 VTOLcontrol.md 中描述的每一层（位置 → 姿态 → 速率 → 融合 → Mixer）都在上述源码中有对应实现。
