# PX4 旋翼姿态控制学习笔记

> 本文结合技术报告 **《Nonlinear Quadrocopter Attitude Control》**（Dario Brescianini, Markus Hehn, Raffaello D'Andrea, ETH Zürich, 2013，本仓库 `2971/full.md`）梳理 PX4 中多旋翼姿态控制相关代码。报告是 PX4 多旋翼 **姿态环（外环）** 控制律的直接理论来源，源码里也明确引用了它。

---

## 0. 全局视角：旋翼控制的级联结构

报告第 2.2 节（Fig. 3）指出，四旋翼输出（位置/速度/姿态/角速率）多于输入（4 个电机推力），不能独立控制，因此采用 **时间尺度分离（time-scale separation）的级联控制**：外环慢、内环快，内环假设比外环快得多。

PX4 严格遵循这一思想，把旋翼控制拆成独立的 uORB 模块，逐层把"设定值"往下传：

```
位置/速度设定值
   │  mc_pos_control（位置环）
   ▼  → 输出：期望加速度 a_cmd + 期望偏航 ψ_cmd  →  vehicle_attitude_setpoint (q_d, thrust_body)
姿态设定值 q_d
   │  mc_att_control（姿态环，本报告对应模块）★
   ▼  → 输出：机体角速率设定值 Ω_cmd  →  vehicle_rates_setpoint
角速率设定值 Ω_cmd
   │  mc_rate_control（角速率环，PID）
   ▼  → 输出：归一化力矩 torque  →  vehicle_torque_setpoint / actuator_controls
力矩 + 推力
   │  control_allocator（控制分配）
   ▼  → 各电机 PWM/转速  →  actuator_motors
```

报告只讨论 **姿态环**（假设角速率 Ω 可被直接、无限快地控制）。在 PX4 里，这个"假设"由下面的 `mc_rate_control` 角速率 PID 环去逼近实现。

报告与代码的对应关系：

| 报告章节 | 内容 | PX4 代码位置 |
|---|---|---|
| §3.1 控制律 (式 23) | 四元数 P 控制律 Ω_cmd = (2/τ)·sgn(q_e0)·q_e,1:3 | `AttitudeControl::update()` |
| §3.2.1 简化姿态控制 | 只控制推力指向（roll/pitch 优先） | `update()` 中 `qd_red` 部分 |
| §3.2.2 完整姿态控制 | 推力指向 + 偏航 | 设定值 `q_d` 本身 |
| §3.2.3 混合简化/完整 | 偏航优先级因子 p | `update()` 中 `_yaw_w` 部分 |
| §3.2.4 限制最大倾角 | 倾角锥约束 | 手动模式 `generate_attitude_setpoint()` |
| 角速率可直接控制的假设 | —— | `rate_control.cpp`（PID 内环逼近该假设）|

核心文件：
- `src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp` ★ **报告控制律的实现**
- `src/modules/mc_att_control/AttitudeControl/AttitudeControl.hpp`
- `src/modules/mc_att_control/mc_att_control_main.cpp`（模块外壳：uORB 收发、手动模式设定值生成）
- `src/modules/mc_att_control/AttitudeControl/AttitudeControlMath.hpp`（VTOL 倾角修正）
- `src/lib/rate_control/rate_control.cpp`（角速率 PID 内环）

---

## 1. 姿态表示：为什么用单位四元数

报告 §2.1 比较了三种姿态表示：

- **欧拉角**（3 参数）：直观，但存在 **奇异点**（万向锁），且把"纯数学原因"造成的限制强加到物理运动上，**不适合全局/大机动姿态控制**。
- **旋转矩阵**（9 参数）：无奇异，但参数冗余多。
- **单位四元数**（4 参数）：报告式 (10) 定义 `q = [cos(α/2), k·sin(α/2)]`（k 为旋转轴，α 为绕轴转角，即 eigenaxis 旋转）。它是 **最小的、全局无奇异** 的参数化方式，且几何直观。

**关键陷阱——双重覆盖（§2.1.4）**：单位四元数空间 S³ 双重覆盖物理姿态空间 SO(3)，即一对反极四元数 `±q` 表示 **同一个物理姿态**。控制器若不处理这一点，会出现 **unwinding（卷绕）现象**——明明只需转一点点，飞机却绕一整圈。报告式 (22) 要求 `Ω_cmd(q) = Ω_cmd(-q)`，做法是当 `q_0 < 0` 时翻转 q 的符号（式 23 里的 `sgn(q_e,0)`）。

> 代码对应：`AttitudeControl.cpp:92` 的 `qe.canonical()` 和 `:79` 的 `qd_dyaw.canonicalize()` 就是在做"取标准半球（q_0 ≥ 0）"，对应报告式 (23) 的 `sgn(q_e,0)`。这一步是 **离散实现里避免 unwinding 的关键**，详见第 3 节。

PX4 中四元数运算由 `matrix` 库（`src/lib/matrix`）的 `Quatf` 类提供，与报告 §2.1.3 的定义（共轭、模、逆、乘法 Q(q)、向量旋转 q·p(r)·q̄）一一对应。

---

## 2. 姿态环控制律 `AttitudeControl::update()` 逐行精读

这是整篇报告最核心的落地。函数签名：

```cpp
matrix::Vector3f AttitudeControl::update(const Quatf &q) const;
```

输入：当前姿态估计 `q`（来自 EKF2，经 `vehicle_attitude` 话题）。
输出：机体系角速率设定值 `Ω_cmd`（rad/s），发布到 `vehicle_rates_setpoint`。

下面按报告的逻辑顺序拆解（`AttitudeControl.cpp:55-114`）。

### 2.1 简化姿态控制：优先 roll/pitch（报告 §3.2.1）

报告动机：四旋翼只能沿机体 z 轴 `e_z^B` 产生推力，所以**真正影响平移运动的是推力指向（roll/pitch），绕推力轴的旋转（yaw）不影响平移**。因此把任务拆成"简化姿态控制"（只控推力指向，且保证不诱发 yaw 旋转）和"完整姿态控制"（推力指向 + yaw）。

报告式 (45) 用当前 z 轴 `e_z^B` 和期望 z 轴 `e_cmd,z^B` 的叉乘构造 **简化误差四元数 `q_e,red`**，使旋转轴 `k ⊥ e_z^B`（最后一项恒为 0 → `Ω_cmd,z = 0`，不诱发偏航）。

代码：

```cpp
const Vector3f e_z   = q.dcm_z();      // 当前机体 z 轴（推力方向），即 e_z^B
const Vector3f e_z_d = qd.dcm_z();     // 期望机体 z 轴，即 e_cmd,z^B
Quatf qd_red(e_z, e_z_d);              // 两向量间的最短旋转 → 对应报告式 (45) 的 q_e,red
```

`Quatf(v1, v2)` 构造的是"把 v1 旋到 v2 的最短旋转四元数"，正是报告式 (45)（绕 `e_z^B × e_cmd,z^B` 轴转 α，α 由式 46 给出）。

把这个"误差旋转"作用到当前姿态上，得到世界系下的简化期望姿态（报告式 47 `q_cmd,red = q · q_e,red`，注意代码用右乘是因为两个 z 向量都在世界系表示）：

```cpp
} else {
    qd_red *= q;   // q_cmd,red = q_e,red(world) * q
}
```

**退化情形处理**（代码 `:64-69`）：当机体推力方向与期望方向几乎完全相反（夹角 ≈180°）时，叉乘退化、旋转轴不确定。此时直接退回完整期望姿态 `qd_red = qd`，注释说明这在数学上仍然安全稳定。这正是报告 §3.2.1 没细谈、但实现必须兜住的数值边界。

### 2.2 混合简化与完整姿态控制：偏航降权（报告 §3.2.3）

报告核心洞察：**偏航 Ω_z 的动态远慢于 roll/pitch**——roll/pitch 靠差动推力直接产生力矩，而 yaw 只能靠桨叶气动阻力反扭矩，慢得多。若三轴用同一增益 `1/τ`，要么 roll/pitch 太慢，要么 yaw 严重超调，还浪费有限的控制量（`‖Ω_cmd‖ ≤ 2/τ`）在不关键的 yaw 上。

解决办法（报告式 53、54）：定义 `q_mix = q_cmd,red⁻¹ · q_cmd,full`，它恒为绕 z 轴的纯偏航旋转 `[cos(α_mix/2),0,0,sin(α_mix/2)]`，然后用因子 `p∈[0,1]` 只修正 **p 比例** 的偏航误差，让期望姿态落在简化与完整之间：

```
q_cmd = q_cmd,red · [cos(p·α_mix/2), 0, 0, sin(p·α_mix/2)]
```

代码实现（`:76-85`）：

```cpp
// 从 qd_red 到完整设定值 qd 之间的"剩余偏航" → 即报告的 q_mix
Quatf qd_dyaw = qd_red.inversed() * qd;
qd_dyaw.canonicalize();
// 数值保护：限制 acosf/asinf 的定义域
qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

// 用偏航权重 _yaw_w（即报告的 p）缩放偏航角，再重新组合期望姿态
qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f,
                    sinf(_yaw_w * asinf(qd_dyaw(3))));
```

- `qd_dyaw` 形如 `[cos(α_mix/2),0,0,sin(α_mix/2)]`，与报告式 (53) 完全一致。
- `_yaw_w` 就是报告里的优先级因子 `p`，由参数 `MC_YAW_WEIGHT` 设定（见第 5 节）。
- 即使只修正 p 比例的偏航误差，报告证明 `t→∞` 时偏航仍收敛到目标（只是慢），同时把宝贵控制量留给 roll/pitch。

> 增益补偿：见 `setProportionalGain()`（`:44-53`），当 `_yaw_w` 给 yaw 降权后，会把 yaw 的比例增益 `_proportional_gain(2) /= _yaw_w` 放大回去，抵消降权对输出尺度的影响。报告式 (64) `p = τ/τ_yaw` 给出了 p 的物理含义：让小偏航误差时控制器表现得像时间常数 τ_yaw。

### 2.3 四元数误差反馈律（报告 §3.1，式 23）

这是报告的"主定理"——全局渐近稳定的控制律：

```
Ω_cmd(q) = (2/τ)·sgn(q_e,0)·q_e,1:3,   q_e := q⁻¹·q_cmd
```

代码（`:87-95`）：

```cpp
// qe 是从当前姿态 q 到期望 qd 的旋转误差
const Quatf qe = q.inversed() * qd;

// 用 sin(α/2) 缩放的旋转轴作为姿态误差（四元数轴角定义），
// 同时通过 canonical() 处理反极四元数歧义（即报告的 sgn(q_e,0)）
const Vector3f eq = 2.f * qe.canonical().imag();

// 角速率设定值 = 误差 ⊙ 比例增益
Vector3f rate_setpoint = eq.emult(_proportional_gain);
```

逐项对应报告式 (23)：

| 报告 | 代码 | 说明 |
|---|---|---|
| `q_e = q⁻¹·q_cmd` | `qe = q.inversed() * qd` | 误差四元数 |
| `q_e,1:3`（向量部）| `qe.canonical().imag()` | 取虚部 |
| `2·(...)` | `2.f * ...` | 由 `sin(α/2)→α` 的线性化系数（小角时 `q_e,1:3 ≈ k·α/2`，乘 2 得旋转向量 `k·α`）|
| `sgn(q_e,0)` | `.canonical()` | 强制 `q_0 ≥ 0`，取最短旋转、防 unwinding |
| `1/τ` | `_proportional_gain`（每轴）| 时间常数 τ 的倒数即比例增益 P |

**为什么是"P 控制"？** 报告 Remark 4（式 32-33）证明：把控制律代回动力学，在平衡点附近系统简化为 **解耦的一阶系统**，时间常数恰为 τ。所以这个看似简单的比例增益，本质上让每个姿态分量像一阶惯性环节一样以时间常数 τ 收敛——增益 `P = 1/τ`。模块说明里也写明"The controller has a P loop for angular error"（`mc_att_control_main.cpp:447`）。

**输出限幅**（报告 Remark 3：`‖Ω_cmd‖ ≤ 2/τ`，对有饱和的系统是优势）：

```cpp
// 限制角速率（来自 MC_*RATE_MAX 参数）
for (int i = 0; i < 3; i++) {
    rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
}
```

### 2.4 偏航角速率前馈

报告假设直接产生 Ω_cmd；PX4 额外加了 **偏航速率前馈**（如位置环要求飞机以某偏航角速度转动时）：

```cpp
if (std::isfinite(_yawspeed_setpoint)) {
    // 前馈是世界 z 轴的旋转，需转到机体系再叠加：
    // q.inversed().dcm_z() 取出"世界 z 轴在机体系的表示"
    rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
}
```

这是工程实现对报告纯反馈律的扩展，提升跟踪性能（前馈 + 反馈）。

---

## 3. 鲁棒性与离散实现（报告 §3.1.1）

报告坦承：式 (23) 虽全局渐近稳定，但 **对任意小的测量噪声不鲁棒**。在不连续点 `q_0 = 0`（与目标差 180° 的姿态集合 M）附近，可构造噪声让系统卡在不连续面上不收敛（式 34-42）。

报告给出两种解法：

1. **带迟滞记忆状态的离散控制器**（文献 [8]）：记住旋转方向，在 `|π-α| ≤ β_0` 时不更新方向，避免被小噪声反复翻转。
2. **离散时间实现天然带迟滞**（报告关键工程结论）：离散控制器两次更新之间输出恒定，旋转方向在该周期内不会被改变，因此 **离散实现本身就等效于带迟滞的连续控制器**，无需显式迟滞状态。

> 这正是 PX4 的做法：`AttitudeControl::update()` 在固定的工作队列周期（姿态更新触发）被调用，输出在两次之间保持。配合 `canonical()` 取最短旋转，离散执行就规避了不连续面附近的不鲁棒问题——无需额外的迟滞记忆变量。报告 §3.3 的"启发式（heuristic）"（针对大偏航角速率时主动选择长方向旋转，式 60-63）在当前 PX4 主线姿态环中 **未实现**，因为它仅对接近陀螺量程的极大角速率有收益，且 3D 情形稳定性未证明（报告 §3.3 Remark 3 也未给证明）。

---

## 4. 模块外壳 `mc_att_control_main.cpp`：数据流与手动设定值生成

`AttitudeControl` 是纯算法类（不依赖 uORB，便于单元测试）。`MulticopterAttitudeControl` 是它的运行外壳，负责调度和 uORB 通信。

### 4.1 调度与数据流（`Run()`，`:208-397`）

- 运行在工作队列 `nav_and_controllers`，由 `vehicle_attitude`（姿态估计）更新 **回调触发**（`init()` 里 `registerCallback()`），即"每来一帧姿态就跑一次控制"——天然实现 §3.1.1 所说的离散周期执行。
- `dt` 被约束在 `[0.2ms, 20ms]`（`:250`），防止异常时间步破坏控制。
- 输入话题：`vehicle_attitude`(q)、`vehicle_attitude_setpoint`(q_d, thrust)、`manual_control_setpoint`、`vehicle_control_mode`、`vehicle_status`、`vehicle_land_detected`、`hover_thrust_estimate`、`autotune_attitude_control_status`。
- 输出话题：`vehicle_rates_setpoint`（roll/pitch/yaw 角速率 + thrust_body）。
- 设定值注入算法类（`:319`）：
  ```cpp
  _attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d),
                                        vehicle_attitude_setpoint.yaw_sp_move_rate);
  ```
- 核心调用（`:345`）：`Vector3f rates_sp = _attitude_control.update(q);`
- 自整定叠加（`:350-358`）：整定期间把激励角速率叠加到 `rates_sp` 上。

### 4.2 EKF 偏航/航向重置处理（`:326-343`）

当 EKF 重置偏航（`quat_reset_counter` 变化）时，用 `delta_q_reset` 同步调整偏航设定值与姿态设定值（`adaptAttitudeSetpoint()`），避免航向跳变导致姿态环误判出巨大误差而猛打舵。这是工程鲁棒性细节，报告未涉及。

### 4.3 手动/自稳模式的姿态设定值生成（`generate_attitude_setpoint()`，`:140-206`）

当处于手动/自稳模式（无位置/高度/速度控制）时，姿态设定值不来自位置环，而由摇杆直接生成，**对应报告 §3.2.4 的最大倾角限制思想**：

- 把 roll/pitch 摇杆映射为 **倾角（tilt angle）+ 最大倾斜方向**，而非直接两个角度：
  ```cpp
  Vector2f v = Vector2f(roll * _man_tilt_max, -pitch * _man_tilt_max);
  float v_norm = v.norm();          // v 的模 = 倾角
  if (v_norm > _man_tilt_max) {     // 限制到配置的最大倾角（报告 §3.2.4 的 α_max）
      v *= _man_tilt_max / v_norm;
  }
  Quatf q_sp_rp = AxisAnglef(v(0), v(1), 0.f);  // 纯 roll/pitch 倾斜四元数
  ```
  这样飞机朝摇杆指向倾斜、倾角易于限幅、摇杆输入线性——与报告"把推力指向约束在半锥角 α_max 的锥内"（式 56-59）目标一致。
- 偏航设定值由 `_stick_yaw.generateYawSetpoint()` 从偏航摇杆积分得到。
- 油门经 `throttle_curve()` 映射（支持悬停推力估计 HTE 居中），写入 `thrust_body[2]`。
- 组合：`Quatf q_sp = q_sp_yaw * q_sp_rp`，发布为 `vehicle_attitude_setpoint`。

### 4.4 VTOL 倾角偏航误差修正（`AttitudeControlMath.hpp`）

`correctTiltSetpointForYawError()` 仅 VTOL 用：存在大偏航误差时，纯倾斜设定值若直接与偏航设定值组合，倾斜方向会和机头朝向不对齐、不符合用户直觉。该函数求解 `q_yaw·q_tilt_ne = q_sp_yaw·q_sp_rp_compensated`，修正倾斜四元数，使其经偏航设定值旋转后倾斜方向与机体对齐。这是报告之外的 VTOL 工程扩展。

---

## 5. 关键参数与时间常数 τ 的对应

报告的物理量 ↔ PX4 参数（`mc_att_control_params.yaml`）：

| 报告量 | PX4 参数 | 代码字段 | 含义 |
|---|---|---|---|
| `1/τ_roll` | `MC_ROLL_P` | `_proportional_gain(0)` | roll 比例增益（≈ 1/时间常数）|
| `1/τ_pitch` | `MC_PITCH_P` | `_proportional_gain(1)` | pitch 比例增益 |
| `1/τ_yaw` | `MC_YAW_P` | `_proportional_gain(2)` | yaw 比例增益 |
| `p`（式 54/64）| `MC_YAW_WEIGHT` | `_yaw_w` | 偏航优先级因子 [0,1] |
| `Ω_i,max`（Remark 3 限幅）| `MC_ROLLRATE_MAX` / `MC_PITCHRATE_MAX` / `MC_YAWRATE_MAX` | `_rate_limit` | 各轴角速率上限 |

注入点：`parameters_updated()`（`mc_att_control_main.cpp:91-110`）。报告 §4 实验给出的经验值（roll/pitch τ∈[0.08s,0.15s]、yaw τ∈[0.2s,0.4s]、`p=τ/τ_yaw`）正是 PX4 默认增益调参的理论依据——yaw 增益（1/τ_yaw）天然比 roll/pitch 小，并通过 `MC_YAW_WEIGHT` 实现报告式 (54) 的混合降权。

---

## 6. 内环：角速率 PID `rate_control.cpp`（逼近"Ω 可直接控制"假设）

报告把"角速率 Ω 可被直接、无限快地控制"当作 **假设**（§2.2、§4.1）；PX4 用 `mc_rate_control` 的 PID 内环去 **实现** 这个假设——内环越快，报告的姿态环理论越成立（时间尺度分离）。

控制律（`rate_control.cpp:71-86`）：

```cpp
Vector3f rate_error = rate_sp - rate;   // 角速率误差（rate_sp 来自姿态环 update() 的输出）
Vector3f torque = _gain_p.emult(rate_error) + _rate_int
                  - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);
```

- **P**：角速率误差比例项。
- **I**：积分项 `_rate_int`，消除稳态误差。
- **D**：作用在 **角加速度** 上（`-K_d·angular_accel`），而非误差微分——避免对设定值阶跃求微分产生尖峰。
- **FF**：对设定值的前馈。

工程细节：
- **抗饱和**（`updateIntegral()`，`:88-118`）：当控制分配器报告某轴饱和（`_control_allocator_saturation_*`）时，停止朝饱和方向继续积分（条件积分），防积分卷绕。
- **大误差降 I 增益**（`:101-111`）：误差越大，`i_factor` 越小（400°时显著），抑制翻转后"回弹（bounce-back）"。

内环输出力矩 `torque` 与推力一起送入 `control_allocator`（控制分配），最终解算各电机转速——这一层把"力矩/推力"映射到具体的旋翼，完成从控制律到实际旋翼输出的最后一跳。

---

## 7. 上游：位置环如何把"期望加速度 + 期望偏航"变成"期望姿态 + 推力"（报告 §3.2）

前面第 2 节的姿态环把 **期望姿态 `q_d`** 当输入。这个 `q_d`（以及推力 `thrust_body`）从哪来？答案是 **位置环 `mc_pos_control`**，它正是报告 §3.2"Desired Attitude"——"把期望加速度 `a_cmd` 和期望偏航 `ψ_cmd` 转成目标朝向 `q_cmd`"——的实现。

涉及文件：
- `src/modules/mc_pos_control/PositionControl/PositionControl.cpp`（位置/速度/加速度级联）
- `src/modules/mc_pos_control/PositionControl/ControlMath.cpp`（加速度/推力 → 姿态的几何变换）★

### 7.1 从位置误差到期望加速度（级联 P–PID）

报告假设位置环已给出期望加速度 `a_cmd`，没展开它怎么来。PX4 的实现是一个 **P（位置）→ PID（速度）→ 加速度** 的级联：

```cpp
// _positionControl(): 位置 P 控制 → 速度设定值
Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);   // P

// _velocityControl(): 速度 PID（含前馈、抗饱和）→ 加速度设定值 _acc_sp
Vector3f vel_error = _vel_sp - _vel;
Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);
ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);
_accelerationControl();   // ← 加速度转推力，见 7.2
```

得到的 `_acc_sp` 就是报告里的 `a_cmd`（外环输出），是后续一切的起点。

### 7.2 加速度 → 推力向量（报告式 43、44）`_accelerationControl()`

报告 §3.2.1 的两条关键式：

```
e_cmd,z^B = a_cmd / ‖a_cmd‖        (式 43)  —— 推力指向 = 期望加速度方向（单位向量）
coll_cmd  = ‖a_cmd‖                (式 44)  —— 集体推力大小 = 期望加速度模长
```

代码（`PositionControl.cpp:207-225`）：

```cpp
void PositionControl::_accelerationControl()
{
    // 竖直方向先补上重力对应的比力（飞机要先"扛住"重力）
    float z_specific_force = -CONSTANTS_ONE_G;
    if (!_decouple_horizontal_and_vertical_acceleration) {
        z_specific_force += _acc_sp(2);
    }

    // 推力方向 = 期望比力方向（含重力补偿），归一化 → 对应报告式 (43) 的 e_cmd,z^B
    Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();

    // 限制最大倾角（报告 §3.2.4：把推力方向约束在半锥角 _lim_tilt 的锥内）
    ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);

    // 把加速度换算成归一化推力：用悬停推力 _hover_thrust 标定 "1g ↔ 悬停油门"
    const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
    // 投影到计划的机体姿态上（推力大小，对应报告式 44 的 coll_cmd）
    const float cos_ned_body = Vector3f(0, 0, 1).dot(body_z);
    const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
    _thr_sp = body_z * collective_thrust;   // 推力向量 = 方向 × 大小
}
```

与报告的对应及工程差异：

| 报告 | 代码 | 说明 |
|---|---|---|
| `e_cmd,z^B = a_cmd/‖a_cmd‖`（式 43）| `body_z = (-acc).normalized()` | 推力指向 = 期望比力方向。注意符号：NED 下重力 `+z`，推力沿机体 `-z`，故取负 |
| `coll_cmd = ‖a_cmd‖`（式 44）| `collective_thrust` | 报告用物理量纲推力；PX4 用 **悬停推力 `_hover_thrust`** 把加速度换算成 **归一化油门 [0,1]**，更贴近实际执行 |
| 重力补偿 | `z_specific_force = -g (+acc_z)` | 报告把重力补偿留给位置环上游；PX4 在此显式加入 |
| §3.2.4 最大倾角 `α_max` | `limitTilt(..., _lim_tilt)` | 见 7.3 |

> 悬停推力 `_hover_thrust` 由 `mc_hover_thrust_estimator` 在线估计（话题 `hover_thrust_estimate`），保证"1 个 g 的竖直加速度 ↔ 悬停油门"标定随载重/电量自适应。这是把报告里抽象的"推力大小"落到归一化电机指令的关键工程桥梁。

### 7.3 最大倾角限制 `limitTilt()`（报告 §3.2.4）

报告式 (56-59)：定义倾角 `α_tilt` 为 `e_z^I` 与 `e_cmd,z^B` 的夹角，若超过 `α_max` 就把推力方向限制在半锥角 `α_max` 的锥内。代码（`ControlMath.cpp:53-68`）用向量分解实现同一几何：

```cpp
void limitTilt(Vector3f &body_unit, const Vector3f &world_unit, const float max_angle)
{
    const float dot_product_unit = body_unit.dot(world_unit);
    float angle = acosf(dot_product_unit);          // 当前倾角 α_tilt（式 56）
    angle = math::min(angle, max_angle);            // 限制到 α_max
    Vector3f rejection = body_unit - (dot_product_unit * world_unit);  // 水平分量方向
    if (rejection.norm_squared() < FLT_EPSILON) { rejection(0) = 1.f; }// 平行退化兜底
    body_unit = cosf(angle) * world_unit + sinf(angle) * rejection.unit();  // 锥面上重建方向
}
```

报告指出（也适用于此）：限制倾角 **不能阻止翻转**——外力矩仍可能把飞机推到极端姿态，恢复时最短旋转可能仍要翻一圈。

### 7.4 推力方向 + 偏航 → 完整期望姿态 `q_d`（报告 §3.2.2）`thrustToAttitude` / `bodyzToAttitude`

有了推力向量 `_thr_sp` 和偏航设定值 `_yaw_sp`，最后构造期望四元数。报告 §3.2.2 的思路：**偏航 `ψ_cmd` + 推力指向 `e_cmd,z^B` 唯一确定完整姿态**（roll/pitch 可由两者反解，式 48-52，得到 `q_cmd,full`）。

PX4 用更直接的"构造正交基 → 旋转矩阵 → 四元数"实现同一结果（`ControlMath.cpp:47-114`）：

```cpp
void thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
    bodyzToAttitude(-thr_sp, yaw_sp, att_sp);          // 推力方向 + yaw → q_d
    att_sp.thrust_body[2] = -thr_sp.length();          // 推力大小写入 thrust_body[2]
}

void bodyzToAttitude(Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
    if (body_z.norm_squared() < FLT_EPSILON) { body_z(2) = 1.f; }  // 零向量兜底
    body_z.normalize();                                            // 期望 z 轴 = 推力指向

    // 期望偏航在 XY 平面的方向，再转 90°，作为构造正交基的辅助向量 y_C
    const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};

    // body_x ⊥ body_z 且与期望偏航对齐：body_x = y_C × body_z
    Vector3f body_x = y_C % body_z;
    if (body_z(2) < 0.f) { body_x = -body_x; }         // 倒扣时保持机头朝前
    // ...（推力近水平的退化处理）
    body_x.normalize();
    const Vector3f body_y = body_z % body_x;           // 右手系补全 body_y

    Dcmf R_sp;                                          // 用三个机体轴组装旋转矩阵
    for (int i = 0; i < 3; i++) {
        R_sp(i, 0) = body_x(i); R_sp(i, 1) = body_y(i); R_sp(i, 2) = body_z(i);
    }
    const Quatf q_sp{R_sp};                             // 旋转矩阵 → 四元数
    q_sp.copyTo(att_sp.q_d);                            // 写入期望姿态 q_d
}
```

几何含义：先固定期望 z 轴（推力指向，决定 roll/pitch），再用期望偏航把 x 轴在水平面内"拧"到目标航向，最后右手系补全——等价于报告把 `e_cmd,z^B` 在偏航 `ψ_cmd` 旋转的中间坐标系里投影出 `θ_cmd`、`φ_cmd`（式 48-51），构造 `q_cmd,full`（式 52）。

### 7.5 衔接姿态环：位置环给 full，姿态环再做 reduced/mixing

一个很重要的分工关系：

- **位置环输出的 `q_d` 是报告的 `q_cmd,full`**（推力指向 + 完整偏航，§3.2.2）。
- **姿态环 `AttitudeControl::update()`（第 2 节）再从 `q_d` 里拆出简化姿态 `qd_red` 并按 `_yaw_w` 做混合**（§3.2.1 + §3.2.3）。

也就是说，报告 §3.2 的"reduced / full / mixing"三件事被拆到两个模块里：**位置环负责构造 full，姿态环负责降权偏航**。这正是为什么姿态环里要重新分解出 `qd_red` 和 `qd_dyaw`——它收到的本就是 full 设定值。

输出汇总（`getAttitudeSetpoint()`，`PositionControl.cpp:269-273`）：

```cpp
ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);  // → q_d + thrust_body
attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;                   // 偏航速率前馈（喂给 2.4 节）
```

这三样（`q_d`、`thrust_body`、`yaw_sp_move_rate`）打包进 `vehicle_attitude_setpoint` 话题，正是第 2 节姿态环 `setAttitudeSetpoint()` 的输入——级联闭合。

---

## 8. 单元测试

算法类可脱离飞控独立测试（GoogleTest）：

- `AttitudeControl/AttitudeControlTest.cpp` — 姿态环控制律测试
- `AttitudeControl/AttitudeControlMathTest.cpp` — VTOL 倾角修正测试
- `rate_control/rate_control_test.cpp` — 角速率 PID 测试
- `PositionControl/ControlMathTest.cpp` — 加速度/推力 → 姿态几何变换测试
- `PositionControl/PositionControlTest.cpp` — 位置环级联测试

运行：

```bash
make tests TESTFILTER=AttitudeControl
# 或构建后单跑：
cd build/px4_sitl_test && ctest -R AttitudeControl -V
```

---

## 9. 小结：报告 → 代码 的一句话映射

1. **用四元数而非欧拉角**（§2.1）→ `Quatf`，全局无奇异、防万向锁。
2. **加速度 → 推力指向与大小**（§3.2.1，式 43/44）→ 位置环 `_accelerationControl()`：`body_z` + `collective_thrust`。
3. **推力指向 + 偏航 → 完整期望姿态 q_cmd,full**（§3.2.2）→ `bodyzToAttitude()` 构造正交基 → `q_d`。
4. **简化姿态控制**（§3.2.1，优先 roll/pitch）→ 姿态环 `qd_red`（两 z 轴间最短旋转）。
5. **混合简化/完整、偏航降权**（§3.2.3，因子 p）→ `_yaw_w` 缩放 `qd_dyaw`。
6. **四元数 P 控制律 + 防 unwinding**（§3.1，式 23）→ `eq = 2·qe.canonical().imag()`，`P = 1/τ`。
7. **离散执行天然带迟滞、规避噪声不鲁棒**（§3.1.1）→ 工作队列周期调用 + `canonical()`。
8. **最大倾角限制**（§3.2.4）→ 位置环 `limitTilt()`；手动模式按"倾角 + 方向"映射并限幅。
9. **角速率可直接控制的假设**（§2.2）→ 由 `mc_rate_control` PID 内环逼近。

完整链路：**位置环**（位置/速度/加速度级联）把期望加速度 + 期望偏航转成期望姿态 `q_d` 与推力 → **姿态环**（报告主体，四元数 P 律）输出角速率设定值 → **角速率环**（PID）输出力矩 → **控制分配** 把力矩映射到各旋翼。这就是 PX4 旋翼控制从期望加速度到电机指令的完整理论—代码链路。

---
---

# 第二部分：固定翼位置控制器（Fixed-Wing Position Controller）

> 旋翼和固定翼是 **两套完全不同** 的控制哲学。旋翼能原地悬停、可独立控制推力大小与方向（见第一部分）；固定翼 **必须保持前飞空速** 才有升力，且高度与空速强耦合（拉杆爬升会掉速、推油门会加速也会抬头）。因此固定翼位置控制不能照搬旋翼的"加速度→推力向量"思路，而要用 **能量** 的视角统一处理高度与空速——这就是 TECS。本部分结合代码讲解 PX4 的固定翼位置控制。

## 10. 总览：固定翼控制的双环分工与新架构

### 10.1 旋翼 vs 固定翼 控制对比

| 维度 | 多旋翼 | 固定翼 |
|---|---|---|
| 升力来源 | 旋翼推力，可悬停 | 机翼升力，必须有前飞空速 |
| 推力方向 | 可任意指向（靠姿态）| 基本固定沿机头 |
| 高度控制 | 直接调推力大小 | 靠俯仰 + 油门 **协同**（能量分配）|
| 空速控制 | 无"空速"概念 | 与高度强耦合，需解耦 |
| 横向机动 | 任意平移 | 只能靠 **滚转转弯**（协调转弯）|
| 纵向算法 | 加速度→推力（报告 §3.2）| **TECS** 总能量控制 |
| 横侧向算法 | —— | **NPFG** 非线性路径跟踪制导（取代旧 L1）|

### 10.2 PX4 固定翼控制链路（2025 重构后的新架构）

> 注意：旧版 PX4 把制导 + 控制合在一个 `fw_pos_control_l1` 模块里。当前代码已 **拆分** 成"制导层"和"控制层"两个独立模块，通过 uORB 解耦：

```
任务/航点 (position_setpoint_triplet)、手动设定值
   │  fw_mode_manager（FixedWingModeManager，制导层 = 旧 fw_pos_control 的继承者）★
   │  · 模式状态机：AUTO / TAKEOFF / LANDING / 手动高度/位置 / 8字盘旋 / VTOL过渡
   │  · 横侧向制导（航点跟踪、盘旋）→ 期望航迹方向 / 横向加速度
   │  · 纵向制导 → 期望高度 / 爬升率 / 空速
   ▼  发布两个话题：
   │     fixed_wing_lateral_setpoint      (course / airspeed_direction / lateral_acceleration)
   │     fixed_wing_longitudinal_setpoint (altitude / height_rate / equivalent_airspeed / pitch_direct / throttle_direct)
   │
   │  fw_lateral_longitudinal_control（FwLateralLongitudinalControl，控制层）★
   │  · 横侧向：NPFG → 横向加速度 → roll = atan(a_lat/g)
   │  · 纵向：TECS → pitch + throttle
   ▼  发布：vehicle_attitude_setpoint (q_d 由 roll/pitch/yaw 组成, thrust_body[0]=油门)
姿态设定值
   │  fw_att_control（姿态环：roll/pitch/yaw 角 → 角速率设定值）
   ▼
   │  fw_rate_control（角速率环 PID → 力矩）
   ▼
   │  control_allocator（控制分配 → 副翼/升降舵/方向舵/油门）
   ▼  舵机 + 电机
```

涉及核心文件：
- `src/modules/fw_mode_manager/FixedWingModeManager.cpp`（制导层，2837 行，含起降/盘旋逻辑）
- `src/modules/fw_lateral_longitudinal_control/FwLateralLongitudinalControl.cpp`（控制层，本部分重点）★
- `src/lib/tecs/TECS.cpp`（总能量控制系统）★
- `src/lib/npfg/`（NPFG 非线性路径跟踪制导：`CourseToAirspeedRefMapper`、`AirspeedDirectionController`、`DirectionalGuidance`）

**这种拆分的好处**：制导逻辑（"我要去哪"）与控制逻辑（"怎么用舵面到那"）彻底解耦；横侧向与纵向也通过独立话题解耦，便于 VTOL 复用、便于外部直接注入 `lateral_acceleration` / `pitch_direct` / `throttle_direct` 等底层设定值。

---

## 11. 控制层主循环 `FwLateralLongitudinalControl::Run()`

模块运行在工作队列，由 `vehicle_local_position` 更新 **回调触发**（`Run()`，`FwLateralLongitudinalControl.cpp:142`）。`control_interval` 被约束在 `[1ms, 100ms]`（`:146`）。

是否运行的判据（`:190-196`）：处于位置/速度/加速度/高度/爬升率任一受控模式，且机型为固定翼或处于 VTOL 过渡。

主循环把工作分成 **纵向** 和 **横侧向** 两段，最后组装成姿态设定值发布。

### 11.1 纵向段：调用 TECS 得到 pitch / throttle（`:200-229`）

```cpp
// 读取制导层给的纵向设定值（高度/爬升率/空速/直接俯仰/直接油门）
if (_fw_longitudinal_ctrl_sub.updated()) {
    _fw_longitudinal_ctrl_sub.copy(&_long_control_sp);
}

// 空速设定值做风、失速、变化率适配
const float airspeed_sp_eas = adapt_airspeed_setpoint(control_interval,
        _long_control_sp.equivalent_airspeed, _min_airspeed_from_guidance,
        _lateral_control_state.wind_speed.length());

// 若同时给了高度和爬升率，则把高度设为 NAN（优先跟踪爬升率）
const float altitude_sp = PX4_ISFINITE(_long_control_sp.height_rate) ? NAN : _long_control_sp.altitude;

// 核心：TECS 计算
current_flight_phase = tecs_update_pitch_throttle(control_interval, altitude_sp, airspeed_sp_eas,
        _long_configuration.pitch_min, _long_configuration.pitch_max,
        _long_configuration.throttle_min, _long_configuration.throttle_max,
        _long_configuration.sink_rate_target, _long_configuration.climb_rate_target,
        _long_configuration.disable_underspeed_protection, _long_control_sp.height_rate, now);

// 取 TECS 结果（除非制导层用 pitch_direct/throttle_direct 直接指定，如起飞滑跑）
pitch_sp    = PX4_ISFINITE(_long_control_sp.pitch_direct)    ? _long_control_sp.pitch_direct    : _tecs.get_pitch_setpoint();
throttle_sp = PX4_ISFINITE(_long_control_sp.throttle_direct) ? _long_control_sp.throttle_direct : _tecs.get_throttle_setpoint();
```

TECS 的内部机理见第 12 节。

### 11.2 横侧向段：NPFG → 横向加速度 → 滚转角（`:231-287`）

固定翼 **靠滚转转弯**，所以横侧向控制的终点是一个滚转角设定值。链条是"航迹方向 → 横向加速度 → 滚转角"：

```cpp
float airspeed_direction_sp{NAN};
float lateral_accel_sp{NAN};
// 空速向量 = 地速 − 风速（关键：固定翼真正"感受"到的是空速，不是地速）
const Vector2f airspeed_vector = _lateral_control_state.ground_speed - _lateral_control_state.wind_speed;

if (PX4_ISFINITE(_lat_control_sp.course) && !PX4_ISFINITE(_lat_control_sp.airspeed_direction)) {
    // (A) 制导层给的是"航迹方向（course，地速方向）"，需做风修正转成"空速方向"
    airspeed_direction_sp = _course_to_airspeed.mapCourseSetpointToHeadingSetpoint(
            _lat_control_sp.course, _lateral_control_state.wind_speed, airspeed_sp_eas);
    // 顺带算出当前航迹所需的最小空速（逆风时需要更大空速才能维持地速方向），下一周期喂给纵向
    const float max_true_airspeed = _performance_model.getMaximumCalibratedAirspeed() * _long_control_state.eas2tas;
    _min_airspeed_from_guidance = _course_to_airspeed.getMinAirspeedForCurrentBearing(
            _lat_control_sp.course, _lateral_control_state.wind_speed,
            max_true_airspeed, _param_fw_gnd_spd_min.get()) / _long_control_state.eas2tas;

} else if (PX4_ISFINITE(_lat_control_sp.airspeed_direction)) {
    // (B) 制导层直接给空速方向
    airspeed_direction_sp = _lat_control_sp.airspeed_direction;
    _min_airspeed_from_guidance = 0.f;
}

// 由"期望空速方向 vs 当前空速方向"算横向加速度指令（NPFG 的方向控制器，类比 L1）
if (PX4_ISFINITE(airspeed_direction_sp)) {
    const float heading = atan2f(airspeed_vector(1), airspeed_vector(0));
    lateral_accel_sp = _airspeed_direction_control.controlHeading(airspeed_direction_sp, heading,
                       airspeed_vector.norm());
}

// 制导层也可直接叠加横向加速度（如直接的盘旋向心加速度）
if (PX4_ISFINITE(_lat_control_sp.lateral_acceleration)) {
    lateral_accel_sp = PX4_ISFINITE(lateral_accel_sp) ? lateral_accel_sp + _lat_control_sp.lateral_acceleration
                       : _lat_control_sp.lateral_acceleration;
}

// 按制导质量因子缩放（速度/风估计不确定时降权，见 11.3），再限幅
lateral_accel_sp = getCorrectedLateralAccelSetpoint(lateral_accel_sp, now);
lateral_accel_sp = math::constrain(lateral_accel_sp, -_lateral_configuration.lateral_accel_max,
                                   _lateral_configuration.lateral_accel_max);

// 横向加速度 → 滚转角（协调转弯）
roll_sp = mapLateralAccelerationToRollAngle(lateral_accel_sp);
```

**协调转弯公式**（`mapLateralAccelerationToRollAngle`，`:794-796`）：

```cpp
float FwLateralLongitudinalControl::mapLateralAccelerationToRollAngle(float lateral_acceleration_sp) const {
    return atanf(lateral_acceleration_sp / CONSTANTS_ONE_G);
}
```

物理含义：协调转弯（无侧滑）时，机翼升力的水平分量提供向心加速度 `a_lat`，竖直分量平衡重力 `g`。两者之比 `a_lat/g = tan(φ)`，故所需滚转角 `φ = atan(a_lat/g)`。这是固定翼"用滚转实现转弯"的最核心一行。

**为什么强调"空速方向"而非"地速方向"**：飞机机翼感受的是相对空气的运动（空速向量 = 地速 − 风）。横向制导本质是控制 **空速向量的指向**；而任务航点要求的是 **地速方向（course）** 对齐航线。有风时两者不同（需要"侧偏修正/蟹行"），`CourseToAirspeedRefMapper` 就负责把期望 course 换算成应当对准的空速方向，并算出维持该 course 所需的最小空速。

### 11.3 制导质量降权 `getCorrectedLateralAccelSetpoint()`（`:757-792`）

NPFG 依赖速度与风的估计。当估计不可靠时，盲目打大滚转很危险。该函数用 `getGuidanceQualityFactor()` 算一个 `_can_run_factor ∈ [0,1]`，把横向加速度指令按比例缩小：

```cpp
_can_run_factor = math::constrain(getGuidanceQualityFactor(_local_pos, _wind_valid), 0.f, 1.f);
// ...（若降权超过阈值持续 2 秒，向用户告警 "Roll command reduced due to uncertain velocity/wind estimates!"）
return _can_run_factor * lateral_accel_sp;
```

这是把"算法置信度"显式纳入控制输出的工程安全设计，旋翼部分没有的对应物。

### 11.4 组装并发布姿态设定值（`:296-318`）

```cpp
float roll_body  = PX4_ISFINITE(roll_sp)  ? roll_sp  : 0.0f;
float pitch_body = PX4_ISFINITE(pitch_sp) ? pitch_sp : 0.0f;
float yaw_body   = _yaw;                                   // 固定翼不直接控偏航，随姿态
const float thrust_body_x = PX4_ISFINITE(throttle_sp) ? throttle_sp : 0.0f;

if (_control_mode_sub.get().flag_control_manual_enabled) {   // 手动模式额外限幅
    roll_body  = constrain(roll_body,  -radians(_param_fw_r_lim.get()), radians(_param_fw_r_lim.get()));
    pitch_body = constrain(pitch_body,  radians(_param_fw_p_lim_min.get()), radians(_param_fw_p_lim_max.get()));
}

roll_body = _roll_slew_rate.update(roll_body, control_interval);   // 滚转变化率限制，避免猛打

_att_sp.timestamp = now;
const Quatf q(Eulerf(roll_body, pitch_body, yaw_body));    // 欧拉角 → 四元数
q.copyTo(_att_sp.q_d);
_att_sp.thrust_body[0] = thrust_body_x;                    // 注意：固定翼推力沿机体 X（前向）！
_attitude_sp_pub.publish(_att_sp);
```

与旋翼的关键差异：
- 推力写 **`thrust_body[0]`（机体 X，前向）**，而旋翼写 `thrust_body[2]`（机体 −Z，上向）。
- 姿态设定值由 `(roll, pitch, yaw)` **欧拉角** 构造——因为固定翼姿态围绕协调飞行的小范围，欧拉角直观且不会触及奇异；这与旋翼用四元数全局表示形成对比（但最终都存成 `q_d` 交给姿态环）。

---

## 12. 纵向核心：TECS 总能量控制系统（`src/lib/tecs/TECS.cpp`）

TECS = **Total Energy Control System**。它解决固定翼最棘手的问题：**高度与空速强耦合**。

### 12.1 核心思想：用"能量"解耦高度与空速

飞机的机械能分为两部分（用"比能"，即单位质量能量）：
- **比势能** `SPE = g·h`（高度）
- **比动能** `SKE = ½·V²`（空速）

对时间求导得 **比能速率**（代码 `_calcSpecificEnergyRates`，`:342-357`）：

```cpp
// 设定值
spe_rate.setpoint = altitude_rate_setpoint * g;          // 势能速率 = 期望爬升率 × g
ske_rate.setpoint = tas_setpoint * tas_rate_setpoint;    // 动能速率 = V · V̇
// 估计值
spe_rate.estimate = altitude_rate * g;                   // 当前爬升率 × g
ske_rate.estimate = tas * tas_rate;                      // 当前 V · V̇
```

两个关键组合量：

| 量 | 定义 | 物理含义 | 由谁控制 |
|---|---|---|---|
| **总能量速率 STE_rate** | `spe_rate + ske_rate` | 总机械能变化（要爬升 **或** 加速都需更多能量）| **油门**（发动机是唯一能量来源）|
| **能量平衡速率 SEB_rate** | `spe_rate·w_spe − ske_rate·w_ske` | 能量在"高度"与"空速"间的 **分配** | **俯仰**（低头把势能换动能=加速，抬头反之）|

直觉：
- **油门管总能量**——油门大，飞机总能量上升，可同时用于爬升和加速。
- **俯仰管能量分配**——俯仰不增加总能量，只在高度和空速之间"搬运"。低头：高度↓空速↑；抬头：高度↑空速↓。

这正是 TECS 的精髓：把两个耦合的物理量（h, V）变换到两个解耦的控制通道（油门 ↔ 总能量，俯仰 ↔ 能量分配）。

### 12.2 速度/高度优先级权重 `speed_weight`（`_updateSpeedAltitudeWeights`，`:382-403`）

`SEB_rate` 里的权重决定俯仰更看重高度还是空速：

```cpp
float pitch_speed_weight = constrain(param.pitch_speed_weight, 0.0f, 2.0f);
// 失速或快速下降时，把权重拉向 2（全力保空速）
const float max_ratio = fmaxf(_ratio_undersped, param.fast_descend);
pitch_speed_weight = max_ratio * 2.0f + (1.0f - max_ratio) * pitch_speed_weight;
if (!flag.airspeed_enabled) { pitch_speed_weight = 0.0f; }   // 无空速传感器时只能靠俯仰保高度

weight.spe_weighting = constrain(2.0f - pitch_speed_weight, 0.f, 2.f);  // 高度权重
weight.ske_weighting = constrain(pitch_speed_weight,        0.f, 2.f);  // 空速权重
```

- `pitch_speed_weight = 0`：俯仰 100% 管高度（无空速测量时必须如此）。
- `= 1`：高度与空速等权。
- `= 2`：俯仰 100% 管空速——**失速保护** 的关键：一旦检测到欠速（`_detectUnderspeed`，`:359-380`），权重渐变到 2，俯仰会主动低头换取空速，优先级高于保持高度。参数 `FW_T_SPDWEIGHT`。

### 12.3 油门控制：总能量速率 → 油门（`_calcThrottleControl`，`:505-540`）

```cpp
// STE rate 估计先过低通滤波
const float STE_rate_estimate_raw = spe_rate.estimate + ske_rate.estimate;
_ste_rate_estimate_filter.update(STE_rate_estimate_raw);

ControlValues ste_rate = _calcThrottleControlSteRate(limit, specific_energy_rates, param);
// ste_rate.setpoint = spe_rate.setpoint + ske_rate.setpoint，并叠加转弯诱导阻力补偿：
//   ste_rate.setpoint += load_factor_correction * (load_factor − 1)
//   —— 转弯时 1/cos(滚转角) 使载荷因子上升，诱导阻力增大，需提前加油门（前馈）

_calcThrottleControlUpdate(...);   // 积分项（含抗饱和、欠速时停止积分）
throttle_setpoint = _calcThrottleControlOutput(...);   // PI：误差比例 + 前馈 + 积分
// 油门变化率限制 + 限幅 [throttle_min, throttle_max]
```

油门通道本质是对 **STE_rate 误差** 的 PI 控制 + 前馈（含转弯阻力补偿）。增益尺度 `STE_rate_to_throttle = 1/(STE_rate_max − STE_rate_min)` 把能量速率归一化到油门量程（`:565`）。

### 12.4 俯仰控制：能量平衡速率 → 俯仰角（`_calcPitchControl`，`:405-503`）

```cpp
ControlValues seb_rate = _calcPitchControlSebRate(weight, specific_energy_rates);
// seb_rate.setpoint = spe_rate.setpoint·w_spe − ske_rate.setpoint·w_ske   （能量分配目标）

_calcPitchControlUpdate(...);   // 俯仰积分项（含俯仰饱和时的抗饱和）

// 把 SEB rate 误差换算成俯仰角：
float SEB_rate_correction = error(seb_rate) * pitch_damping_gain + seb_rate_ff * seb_rate.setpoint;
const float climb_angle_to_SEB_rate = airspeed * g;          // 爬升角 ↔ SEB rate 的换算系数
pitch_setpoint = SEB_rate_correction / climb_angle_to_SEB_rate + _pitch_integ_state;

// 用竖直加速度上限换算出俯仰变化率上限，限制俯仰增量；再限幅 [pitch_min, pitch_max]
const float pitch_increment = dt * vert_accel_limit / max(tas, FLT_EPSILON);
_pitch_setpoint = constrain(pitch_setpoint, _pitch_setpoint - pitch_increment, _pitch_setpoint + pitch_increment);
_pitch_setpoint = constrain(_pitch_setpoint, pitch_min, pitch_max);
```

要点：
- 俯仰角由"**爬升角与 SEB rate 成正比**"的假设反推（`climb_angle_to_SEB_rate = V·g`），即假定迎角偏置恒定、爬升角紧跟俯仰角。
- 含阻尼项（`FW_T_PTCH_DAMP`）、前馈项（`FW_T_SEB_R_FF`）、积分项（`FW_T_I_GAIN_PIT`）。
- 通过竖直加速度上限（`FW_T_VERT_ACC`）限制俯仰变化率，保证乘坐/结构载荷舒适。

### 12.5 高度→爬升率外环 与 限幅（`_calcAltitudeControlOutput`，`:331-340`）

进入能量计算前，高度误差先转成爬升率设定值（比例 + 前馈）：

```cpp
altitude_rate_output = (altitude_setpoint − altitude) * altitude_error_gain
                       + altitude_setpoint_gain_ff * altitude_rate_setpoint;
altitude_rate_output = constrain(altitude_rate_output, -max_sink_rate, max_climb_rate);
```

总能量速率上下限由性能模型给出（`_calculateTotalEnergyRateLimit`，`:298-304`）：`STE_rate_max = max_climb_rate · g`，`STE_rate_min = −min_sink_rate · g`，从而把油门/俯仰需求约束在飞机性能包线内。

### 12.6 控制层对 TECS 的封装 `tecs_update_pitch_throttle()`（`:372-414`）

控制层在调用 `_tecs.update()` 前做了若干适配：
- VTOL 处于旋翼模式/过渡时 **不运行 TECS**（`:381-386`）。
- 计算配平油门 `throttle_trim_compensated`（随空速、空气密度变化）。
- 俯仰设定值偏置 `FW_PSP_OFF`（安装角补偿）。
- 把高度、空速、爬升率、性能限制等一并喂入 `_tecs.update(...)`，随后发布 `tecs_status` 供日志/调参。

---

## 13. 制导层 `FixedWingModeManager`（旧 fw_pos_control 的继承者）

制导层负责"去哪"，是一个 **模式状态机**（`FW_POSCTRL_MODE_*`），根据 `vehicle_control_mode` 和 `position_setpoint_triplet` 切换子控制器（`update_control_mode`，`:378+`）：

- `AUTO` / `AUTO_PATH`：常规航点跟踪（`control_auto_position`），用 `navigateWaypoints()` 做 NPFG 航迹制导，盘旋用 `navigateLoiter()`。
- `AUTO_TAKEOFF` / `AUTO_TAKEOFF_NO_NAV`：起飞（滑跑 `runway_takeoff/` 或弹射/手抛 `launchdetection/`），此阶段常用 `pitch_direct` / `throttle_direct` 绕过 TECS。
- `AUTO_LANDING_STRAIGHT` / `AUTO_LANDING_CIRCULAR`：直线/盘旋进近着陆。
- `MANUAL_ALTITUDE` / `MANUAL_POSITION`：手动定高/定位。
- `TRANSITION_TO_HOVER_*`：VTOL 切回旋翼。
- 特技：8 字盘旋（`figure_eight/`）。

输出（`FixedWingModeManager.hpp:200-201`）：

```cpp
uORB::PublicationData<fixed_wing_lateral_setpoint_s>      _lateral_ctrl_sp_pub{ORB_ID(fixed_wing_lateral_setpoint)};
uORB::PublicationData<fixed_wing_longitudinal_setpoint_s> _longitudinal_ctrl_sp_pub{ORB_ID(fixed_wing_longitudinal_setpoint)};
```

即制导层把所有飞行阶段的复杂逻辑都归一化成两个简单话题（横侧向 course/横向加速度 + 纵向高度/空速），交给第 11 节的控制层统一执行。这正是新架构解耦的价值：无论起飞、巡航还是着陆，控制层代码不变。

---

## 14. 固定翼关键参数速查

| 参数 | 含义 | 对应代码 |
|---|---|---|
| `FW_T_SPDWEIGHT` | 速度/高度优先级权重（0=只保高度，2=只保速度）| `pitch_speed_weight` |
| `FW_T_CLMB_R_SP` / `FW_T_SINK_R_SP` | 目标爬升率 / 下沉率 | `climb_rate_target` / `sink_rate_target` |
| `FW_T_PTCH_DAMP` / `FW_T_I_GAIN_PIT` | TECS 俯仰阻尼 / 积分增益 | `_calcPitchControl*` |
| `FW_T_THR_DAMPING` / `FW_T_THR_INTEG` | TECS 油门阻尼 / 积分增益 | `_calcThrottleControl*` |
| `FW_T_SEB_R_FF` | 能量平衡速率前馈 | `seb_rate_ff` |
| `FW_T_RLL2THR` | 转弯（滚转）→ 油门 前馈补偿 | `load_factor_correction` |
| `FW_T_VERT_ACC` | 竖直加速度上限（限俯仰变化率）| `pitch_increment` |
| `FW_THR_MIN/MAX/SLEW_MAX` | 油门下限/上限/变化率 | 油门限幅 |
| `FW_P_LIM_MIN/MAX` / `FW_R_LIM` | 俯仰/滚转角限制 | 姿态设定值限幅 |
| `NPFG_PERIOD` / `NPFG_DAMPING` | NPFG 周期 / 阻尼（横向制导带宽与超调）| `setPGainFromPeriodAndDamping` |
| `FW_GND_SPD_MIN` | 最小地速（逆风下保航迹）| `getMinAirspeedForCurrentBearing` |
| `FW_USE_AIRSPD` | 是否有空速传感器 | `airspeed_enabled`（影响 speed_weight）|

---

## 15. 固定翼小结：一句话映射

1. **能量解耦高度与空速**（TECS 核心）→ 油门管总能量 `STE_rate`，俯仰管能量分配 `SEB_rate`。
2. **总能量速率 → 油门**（PI + 转弯阻力前馈）→ `_calcThrottleControl()`。
3. **能量平衡速率 → 俯仰**（爬升角 ∝ SEB rate）→ `_calcPitchControl()`。
4. **失速保护**（欠速时 speed_weight→2，俯仰优先保速）→ `_detectUnderspeed` + `_updateSpeedAltitudeWeights`。
5. **横侧向 NPFG**（路径跟踪取代 L1）→ 期望航迹方向 → 横向加速度。
6. **协调转弯**（固定翼靠滚转转弯）→ `roll = atan(a_lat/g)`。
7. **风修正**（控空速方向而非地速方向）→ `CourseToAirspeedRefMapper`。
8. **制导/控制解耦**（新架构）→ `fw_mode_manager`（去哪）+ `fw_lateral_longitudinal_control`（怎么飞）。

完整链路：**制导层 `fw_mode_manager`**（模式状态机：起飞/巡航/着陆/盘旋）发布横侧向 + 纵向设定值 → **控制层 `fw_lateral_longitudinal_control`**（NPFG→滚转、TECS→俯仰/油门）发布 `vehicle_attitude_setpoint` → **`fw_att_control`**（姿态→角速率）→ **`fw_rate_control`**（角速率→力矩）→ **`control_allocator`**（→ 副翼/升降舵/方向舵/油门）。这就是 PX4 固定翼从航点到舵面的完整控制链路。

---

# 第三部分：固定翼姿态控制器（Fixed-Wing Attitude Controller）

> 第二部分的位置控制层输出了 `vehicle_attitude_setpoint`（期望姿态 `q_d` + 油门）。本部分讲下游两环：**姿态环 `fw_att_control`**（姿态角 → 角速率设定值）和 **角速率环 `fw_rate_control`**（角速率 → 舵面力矩）。固定翼姿态控制有两个旋翼完全没有的核心特性——**偏航误差消除**（无直接偏航操纵权）与 **空速缩放**（舵效随速度平方变化），下面重点剖析。

涉及文件：
- `src/modules/fw_att_control/FixedwingAttitudeControl.cpp`（姿态环，含 `computeAttitudeError`、协调转弯前馈）★
- `src/modules/fw_att_control/FixedwingAttitudeControl.hpp`（`computeAttitudeError` 内联函数）★
- `src/modules/fw_rate_control/FixedwingRateControl.cpp`（角速率环，含空速缩放、配平、增益压缩）★
- `src/lib/rate_control/rate_control.cpp`（与旋翼 **共用** 的 PID 库，见第一部分第 6 节）

## 16. 姿态环 `fw_att_control`：姿态角 → 角速率设定值

模块由 `vehicle_attitude` 触发（或 20ms 超时兜底，`FixedwingAttitudeControl.cpp:183`），`dt` 约束在 `[2ms, 40ms]`。输入 `vehicle_attitude_setpoint`，输出 `vehicle_rates_setpoint`。核心控制律是 **比例（P）控制 + 协调转弯前馈**。

### 16.1 比例增益：roll/pitch 用时间常数，yaw 固定为 1（`:75-79`）

```cpp
_proportional_gain = matrix::Vector3f(1.0f / math::max(0.01f, _param_fw_r_tc.get()),   // roll: 1/FW_R_TC
                                      1.0f / math::max(0.01f, _param_fw_p_tc.get()),   // pitch: 1/FW_P_TC
                                      1.0f);                                            // yaw: 固定 1.0
```

- roll/pitch 增益 = **时间常数的倒数**（`FW_R_TC`、`FW_P_TC`），含义与旋翼姿态环一致（P = 1/τ，让姿态角以时间常数 τ 一阶收敛，见第一部分 §2.3）。
- yaw 增益固定为 1.0——因为固定翼偏航不靠"对准偏航设定值"来控制，而是靠下面的偏航误差消除 + 协调转弯前馈（见 16.2/16.3）。

### 16.2 ★关键：偏航误差消除 `computeAttitudeError()`（`FixedwingAttitudeControl.hpp:80-90`）

这是固定翼与旋翼最本质的区别。**固定翼没有直接的偏航操纵权**——它不能像旋翼那样原地改变机头朝向，偏航只能作为滚转转弯的 **副产品**（协调转弯）。因此姿态误差里若包含偏航误差，会让控制器试图用方向舵硬拗偏航，导致侧滑、效率低下。

解决办法：计算姿态误差前，先 **绕世界 z 轴旋转期望姿态一个 `yaw_offset`，把偏航误差恰好抵消掉**，只剩 roll/pitch 误差需要控制：

```cpp
inline Vector3f computeAttitudeError(const Quatf &q_current, const Quatf &q_sp)
{
    // 计算抵消偏航误差所需的偏航偏置量（公式推导见 formula_derrivation.py）
    const float yaw_offset = -2.f * (q_current(0)*q_sp(3) - q_current(1)*q_sp(2) + q_current(2)*q_sp(1) - q_current(3)*q_sp(0)) /
                              (q_current(0)*q_sp(0) - q_current(1)*q_sp(1) - q_current(2)*q_sp(2) + q_current(3)*q_sp(3));

    const Quatf q_yaw_offset = Quatf(1.f, 0.f, 0.f, yaw_offset / 2.f).normalized();
    // 用偏航偏置"转正"期望姿态后，再算误差四元数（canonical 取最短旋转）
    const Quatf q_err = (q_current.inversed() * q_yaw_offset * q_sp).canonical();
    return 2.f * q_err.imag();   // sin(α/2) 缩放的旋转向量误差（同旋翼的 2·imag 约定）
}
```

要点：
- `yaw_offset` 把期望姿态绕 z 轴旋转，使其偏航与当前偏航对齐 → 误差里 **不再含偏航分量**。
- 误差的提取方式 `2·q_err.imag()` 与旋翼姿态环（第一部分 §2.3）完全一致——同样是四元数误差的虚部，同样用 `canonical()` 防 unwinding。差别只在于多了"消偏航"这一步。
- 推导脚本 `formula_derrivation.py` 就放在模块目录里。

### 16.3 协调转弯前馈（Turn Coordination，`:307-319`）

偏航虽不直接控制，但协调转弯（滚转后机头要随转弯方向偏转、且抬头补偿升力损失）需要 **偏航率与俯仰率前馈**：

```cpp
const float V = math::max(get_airspeed_constrained(), 0.1f);
const float q1 = 2.f * (q_current(0)*q_current(1) + q_current(2)*q_current(3)); // ≈ 2·sin(roll)·cos(pitch)
const float yawrate_ff  = CONSTANTS_ONE_G * q1 / V;                              // 协调转弯偏航角速率
const float pitchrate_ff = q1 * yawrate_ff / (1.f - 2.f*q_current(1)*q_current(1) - 2.f*q_current(2)*q_current(2));

// 极端姿态（倾角 70°~75°）渐变关闭前馈，避免数值发散
const float cos_tilt = 1.f - 2.f * (q_current(1)*q_current(1) + q_current(2)*q_current(2));
const float tilt = acosf(math::constrain(cos_tilt, -1.f, 1.f));
const float ff_scale = math::interpolate(tilt, radians(70.f), radians(75.f), 1.f, 0.f);

body_rates_setpoint(1) += ff_scale * pitchrate_ff;   // 俯仰率前馈
body_rates_setpoint(2) += ff_scale * yawrate_ff;     // 偏航率前馈
```

物理含义：协调转弯时偏航角速率 `ψ̇ = g·sin(φ)/V`（与第二部分协调转弯 `roll=atan(a_lat/g)` 是同一物理关系的角速率形式）。注意 `yawrate_ff` **反比于空速 V**——同样的滚转角，速度越慢转弯越快。前馈让飞机在进入转弯的瞬间就主动给出协调的偏航/俯仰率，而不必等姿态误差累积，大幅提升转弯品质。

### 16.4 角速率设定值组装、限幅与发布（`:304-361`）

```cpp
Vector3f body_rates_setpoint = _proportional_gain.emult(att_err);   // P 控制
// ... 叠加 16.3 的协调转弯前馈 ...

// 三轴角速率限幅（FW_R_RMAX / FW_P_RMAX_POS/NEG / FW_Y_RMAX）
body_rates_setpoint(0) = constrain(..., -FW_R_RMAX, FW_R_RMAX);
body_rates_setpoint(1) = constrain(..., -FW_P_RMAX_NEG, FW_P_RMAX_POS);
body_rates_setpoint(2) = constrain(..., -FW_Y_RMAX, FW_Y_RMAX);

// 自整定激励叠加；手动模式下叠加偏航杆输入（协调转弯时蹬舵）
if (_vcontrol_mode.flag_control_manual_enabled) {
    body_rates_setpoint(2) += constrain(_manual_control_setpoint.yaw * MAN_YR_MAX, ...);
}

// Tailsitter VTOL：从固定翼系变换到悬停（机体）系
if (_vehicle_status.is_vtol_tailsitter) {
    body_rates_setpoint = Vector3f(body_rates_setpoint(2), body_rates_setpoint(1), -body_rates_setpoint(0));
}

_rates_sp.roll/pitch/yaw = body_rates_setpoint;
_rate_sp_pub.publish(_rates_sp);   // → vehicle_rates_setpoint
```

此外模块还内嵌一个 **前轮转向控制器** `fw_wheel_controller`（`_wheel_ctrl`，PI+FF），用于地面滑跑阶段的方向控制（`:366-399`），与空中姿态控制独立。

## 17. 角速率环 `fw_rate_control`：角速率 → 舵面力矩

输入 `vehicle_rates_setpoint`，输出 `vehicle_torque_setpoint`（三轴归一化力矩 → 副翼/升降舵/方向舵）+ `vehicle_thrust_setpoint`（油门）。它 **复用与旋翼相同的 `RateControl` PID 库**（第一部分 §6），但叠加了固定翼特有的 **空速缩放** 和 **配平**。

### 17.1 ★关键：空速缩放（Airspeed Scaling，`:194-201`）

固定翼最重要的工程特性：**气动操纵力正比于动压，即正比于速度平方 `V²`**。同样的舵偏角，速度越快力矩越大。为了让控制器增益在整个速度包线内保持一致的响应，PX4 引入空速缩放：

```cpp
if (_param_fw_arsp_scale_en.get()) {
    const float min_airspeed = math::max(_param_fw_airspd_stall.get(), 0.1f);
    const float airspeed_constrained = math::max(airspeed, min_airspeed);
    _airspeed_scaling = _param_fw_airspd_trim.get() / airspeed_constrained;   // = V_trim / V
} else {
    _airspeed_scaling = 1.0f;
}
```

应用到力矩输出（`:386-388`）：

```cpp
const Vector3f angular_acceleration_setpoint = _rate_control.update(rates, body_rates_setpoint, angular_accel, dt, _landed);
// 角加速度需求 → 舵面力矩，乘以 scaling²（因为力矩 ∝ V²，低速时要按 (V_trim/V)² 放大舵偏补偿）
Vector3f control_u = _gain_compression.getGains().emult(angular_acceleration_setpoint * _airspeed_scaling * _airspeed_scaling);
```

- `_airspeed_scaling = V_trim / V`：低于配平空速时 >1，高于时 <1。
- 力矩乘 `scaling²`：因为产生同样力矩所需的归一化舵量 ∝ 1/V²。低速时放大、高速时缩小，使闭环响应与速度 **解耦**。
- 前馈增益则除以 scaling（`:382`，`scaled_gain_ff = gain_ff / _airspeed_scaling`），同理补偿。
- 参数 `FW_ARSP_SCALE_EN` 开关，`FW_AIRSPD_TRIM` 为配平空速基准。

### 17.2 配平（Trim）与舵面交联

```cpp
// 配平随空速插值（不同速度下保持平飞所需的舵面中立偏置），并乘 scaling²
trim *= _airspeed_scaling * _airspeed_scaling;                                 // :349
// ...
matrix::constrain(control_u + trim, -1.f, 1.f).copyTo(_vehicle_torque_setpoint.xyz);  // 力矩 = PID输出 + 配平，限幅[-1,1]
```

**滚转→偏航前馈**（`:440-441`，协调转弯的舵面级补偿）：

```cpp
// 副翼滚转转弯时，方向舵按比例配合，抑制不利偏航（adverse yaw）
_vehicle_torque_setpoint.xyz[2] = constrain(xyz[2] + _param_fw_rll_to_yaw_ff.get() * xyz[0], -1.f, 1.f);
```

这与 16.3 的偏航率前馈是 **不同层级** 的协调转弯补偿：16.3 在角速率设定值层，17.2 在舵面力矩层（`FW_RLL_TO_YAW_FF`）。

### 17.3 PID 本体、增益压缩与电池补偿

- `_rate_control` 用 `setPidGains(rate_p, rate_i, rate_d)`（`:83`）——和旋翼同一个库（误差比例 + 积分 + 角加速度阻尼 + 前馈，见第一部分 §6），积分含控制分配饱和的抗饱和。
- `_gain_compression`（增益压缩，`src/lib/rate_control/gain_compression.cpp`）：在接近饱和时压缩增益，防止舵面饱和引发的振荡。
- 电池补偿（`:411-422`）：`FW_BAT_SCALE_EN` 开启时按电压衰减放大油门，保持推力随电量稳定。
- Acro 模式偏航特例（`:392-397`）：`FW_ACRO_YAW_EN` 未设时偏航直接由摇杆控制、不做角速率闭环。

### 17.4 输出去向

```cpp
_vehicle_torque_setpoint.xyz   // 三轴力矩（roll→副翼, pitch→升降舵, yaw→方向舵）
_vehicle_thrust_setpoint.xyz[0] // 油门（前向，从 rates_sp.thrust_body[0] 透传 + 电池补偿）
```

两者发布后交给 **`control_allocator`**，由控制效率矩阵映射到具体舵机与电机的 PWM 输出——这是固定翼控制链路的最后一跳（与旋翼共用控制分配框架，但效率矩阵不同：固定翼是副翼/升降舵/方向舵/油门，旋翼是各电机转速）。

## 18. 固定翼姿态控制关键参数速查

| 参数 | 含义 | 对应代码 |
|---|---|---|
| `FW_R_TC` / `FW_P_TC` | roll/pitch 姿态时间常数（增益=1/TC）| `_proportional_gain` |
| `FW_RR_P/I/D/FF` 等 | 角速率 PID 与前馈增益（roll，pitch/yaw 同理）| `_rate_control.setPidGains` |
| `FW_ARSP_SCALE_EN` | 是否启用空速缩放 | `_airspeed_scaling` |
| `FW_AIRSPD_TRIM/STALL/MAX` | 配平/失速/最大空速（缩放基准与限幅）| `updateAirspeed()` |
| `FW_R_RMAX` / `FW_P_RMAX_POS/NEG` / `FW_Y_RMAX` | 三轴角速率上限 | 角速率设定值限幅 |
| `FW_RLL_TO_YAW_FF` | 滚转→偏航 舵面前馈（抑制不利偏航）| `xyz[2] += ff * xyz[0]` |
| `FW_DTRIM_*_VMAX` | 配平随速度插值 | `trim` 插值 |
| `FW_BAT_SCALE_EN` | 油门电池电压补偿 | `_battery_scale` |
| `FW_ACRO_YAW_EN` | Acro 模式是否角速率控偏航 | 偏航特例 |
| `FW_MAN_R/P/Y_MAX`、`MAN_YR_MAX` | 手动模式角度/角速率上限 | 手动设定值 |

## 19. 固定翼姿态控制小结：一句话映射

1. **姿态环用 P 控制**（roll/pitch 增益 = 1/时间常数）→ `_proportional_gain.emult(att_err)`。
2. **★偏航误差消除**（固定翼无直接偏航操纵权，只控 roll/pitch）→ `computeAttitudeError()` 的 `yaw_offset`。
3. **协调转弯前馈①（角速率层）**（`ψ̇ = g·sin φ/V`）→ `yawrate_ff` / `pitchrate_ff`，极端姿态渐隐。
4. **角速率环复用旋翼 PID 库** → `_rate_control.update()`。
5. **★空速缩放**（舵效 ∝ V²）→ 力矩 × `(V_trim/V)²`，前馈 ÷ scaling。
6. **协调转弯前馈②（舵面层）**（抑制不利偏航）→ `FW_RLL_TO_YAW_FF`。
7. **配平随速度插值 + 电池补偿** → `trim` / `_battery_scale`。
8. **输出力矩 + 油门 → 控制分配**（→ 副翼/升降舵/方向舵/油门）。

完整链路（接第二部分）：`vehicle_attitude_setpoint` → **`fw_att_control`**（消偏航 + P + 协调转弯前馈 → 角速率设定值）→ **`fw_rate_control`**（PID + 空速缩放 + 配平 → 力矩 + 油门）→ **`control_allocator`** → 副翼/升降舵/方向舵/油门。至此，固定翼从航点到每一片舵面的完整控制链路全部打通。

---

## 附：PX4 控制器学习全景

| 部分 | 机型/环节 | 核心算法 | 关键模块 |
|---|---|---|---|
| 第一部分 | 多旋翼全链路 | 四元数姿态控制（ETH 报告）+ 加速度→推力 | `mc_pos_control` / `mc_att_control` / `mc_rate_control` |
| 第二部分 | 固定翼位置控制 | TECS 总能量 + NPFG 路径跟踪 + 协调转弯 | `fw_mode_manager` / `fw_lateral_longitudinal_control` |
| 第三部分 | 固定翼姿态控制 | 消偏航 P 控制 + 空速缩放 PID | `fw_att_control` / `fw_rate_control` |

三部分共同的设计哲学：**uORB 解耦的级联控制** + **设定值逐层下传**（位置→姿态→角速率→力矩→分配）。旋翼与固定翼共享 `RateControl` PID 库和 `control_allocator` 框架，但上层控制律因物理本质不同而分道扬镳——这正是 PX4 多机型统一架构的精妙之处。
