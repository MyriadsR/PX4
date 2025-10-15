# PX4 尾座（Tailsitter）控制框架

本文件概述 PX4 中尾座 VTOL（Tailsitter）的控制结构、状态机、过渡姿态生成、推力/力矩混合、固定翼控制中的坐标变换、控制分配以及关键参数与数据流。文末附带相关源码文件与行号引用，便于快速跳转阅读。

## 总体结构

- 入口与基类
  - `PX4-Autopilot/src/modules/vtol_att_control/vtol_att_control_main.cpp`：VTOL 控制主循环（虚拟 MC/FW 控制器接口）。
  - `PX4-Autopilot/src/modules/vtol_att_control/vtol_type.h`：通用 VtolType 基类（公共状态/参数/过渡管理）。
- 尾座机型实现
  - `PX4-Autopilot/src/modules/vtol_att_control/tailsitter.h:76`：尾座专用状态枚举 `vtol_mode`。
  - `PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp`：状态机、过渡姿态、推力混合、输出。

## 状态机与模式切换

- 主状态机：`PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:59`
  - 固定翼系统故障→立即回到 MC。
  - 用户请求 MC：
    - FW_MODE → TRANSITION_BACK，达到俯仰阈值（或超时）→ MC_MODE。
  - 用户请求 FW：
    - MC_MODE → TRANSITION_FRONT_P1，完成条件满足→ FW_MODE。
  - 模式映射到通用 VTOL 模式：MC→ROTARY_WING，FW→FIXED_WING，TRANSITION_* → TRANSITION_TO_FW/MC。
- 前向过渡完成判据：`PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:337`
  - 俯仰角达到阈值，且（如果有空速）达到 `VT_ARSP_TRANS`；无空速则仅姿态阈值。
  - 姿态阈值常量见 `PX4-Autopilot/src/modules/vtol_att_control/tailsitter.h:23` 与 `:26`。

## 过渡姿态轨迹与坐标处理

- 过渡状态更新：`PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:155`
  - 依赖“虚拟”MC/FW 姿态设定的新鲜度（<1s），否则保持上次设定。
  - 进入过渡时计算旋转轴 `_trans_rot_axis` 与起始姿态 `_q_trans_start`，生成目标四元数 `_q_trans_sp`：
    - 前向过渡：围绕与机体 x 轴相关的轴旋转，使机头由竖直上指逐步过渡至水平飞行。
    - 回退过渡：以机体朝向与重力方向叉乘的轴，平滑回到竖直悬停姿态。
  - 以参数化角速度推进旋转（`VT_F_TRANS_DUR`/`VT_B_TRANS_DUR`，下限 0.1s），并写入 `_v_att_sp->q_d`。
  - 回退开始短时间内对推力进行平滑混合，确保电机不断油。

## 推力/力矩混合与输出

- 前向过渡后推力混合：`PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:359`
  - MC 推力为负 z，FW 推力为正 x，进行轴向与幅值平滑过渡。
- 回退过渡初期推力混合：`PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:365`
  - 将 MC 推力与上次 FW 推力短时混合，避免空档。
- 输出填充：`PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:261`
  - matrix 0（电机）：来自 MC 或 FW 虚拟推力/力矩，根据模式短时保持/混合。
  - matrix 1（舵面）：默认使用 FW 虚拟力矩；若 `VT_ELEV_MC_LOCK=1` 且纯 MC 模式则锁定舵面。
  - 差动推力（以电机代替部分舵效）：由 `VT_FW_DIFTHR_EN` 与 `VT_FW_DIFTHR_S_*` 控制，仅在 FW_MODE 下生效。
- TECS 等待期（前向过渡完成后）：`PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:246`
  - 沿机体 x 轴保留 MC 推力一段时间，直到 TECS 接管。

## 固定翼控制中的尾座特殊旋转

- 固定翼速率控制对尾座进行帧变换：`PX4-Autopilot/src/modules/fw_rate_control/FixedwingRateControl.cpp:248`
  - 读取机体角速率后将 roll/yaw 对调并取符号，映射“悬停接口”到“固定翼控制器”参考系。
- 期望速率同样做逆变换：`PX4-Autopilot/src/modules/fw_rate_control/FixedwingRateControl.cpp:314`
- 控制输出再旋回到机体系：`PX4-Autopilot/src/modules/fw_rate_control/FixedwingRateControl.cpp:420`

## 控制分配与效能矩阵（尾座）

- 类定义与接口：`PX4-Autopilot/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessTailsitterVTOL.hpp:35`
  - 两套矩阵：matrix 0（电机，顺序去饱和）与 matrix 1（舵面，伪逆）。
  - 由 `_mc_rotors` 与 `_control_surfaces` 组合效能，根据飞行阶段调整归一化与参与轴。
- 控制分配入口选择：`PX4-Autopilot/src/modules/control_allocator/ControlAllocator.cpp:228`
  - 尾座类型映射到 `ActuatorEffectivenessTailsitterVTOL`。

## 关键参数（与尾座逻辑强相关）

- 机型选择：`VT_TYPE=0`
- 过渡时间与空速阈值：`VT_F_TRANS_DUR`、`VT_B_TRANS_DUR`、`VT_ARSP_BLEND`、`VT_ARSP_TRANS`、`VT_TRANS_TIMEOUT`、`VT_TRANS_MIN_TM`
- 过渡推力：`VT_F_TRANS_THR`
- 舵面锁定：`VT_ELEV_MC_LOCK`
- 差动推力：`VT_FW_DIFTHR_EN`、`VT_FW_DIFTHR_S_R`、`VT_FW_DIFTHR_S_P`、`VT_FW_DIFTHR_S_Y`
- 安全门（Quad-chute）：`VT_FW_MIN_ALT`、`VT_QC_ALT_LOSS`、`VT_QC_T_ALT_LOSS`、`VT_FW_QC_P`、`VT_FW_QC_R`、`VT_FW_QC_HMAX`
- 参数定义文件：`PX4-Autopilot/src/modules/vtol_att_control/vtol_att_control_params.c:35`

## 数据流总结

- 上层（Navigator/Commander）根据任务与模式请求 FW 或 MC，Commander 发出过渡命令。
- vtol_att_control 按 `VT_TYPE` 实例化 Tailsitter，循环执行：
  - 依据请求与测量更新 `vtol_mode`（MC、前向/回退过渡、FW）。
  - 生成过渡姿态 `_q_trans_sp` 写入 `_v_att_sp`。
  - 从“虚拟”MC/FW 控制器获取推力/力矩设定，按模式进行混合与短时保持。
  - 经控制分配将控制需求映射到电机与舵面两套矩阵。
- 固定翼速率控制对尾座进行帧变换，确保控制环在正确参考系工作。

## 源码参考（路径:行号）

- 尾座头文件与常量
  - `PX4-Autopilot/src/modules/vtol_att_control/tailsitter.h:23`
  - `PX4-Autopilot/src/modules/vtol_att_control/tailsitter.h:26`
  - `PX4-Autopilot/src/modules/vtol_att_control/tailsitter.h:76`
- 尾座实现（核心函数）
  - `PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:59` update_vtol_state
  - `PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:155` update_transition_state
  - `PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:246` waiting_on_tecs
  - `PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:252` update_fw_state
  - `PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:261` fill_actuator_outputs
  - `PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:337` isFrontTransitionCompletedBase
  - `PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:359` blendThrottleAfterFrontTransition
  - `PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:365` blendThrottleBeginningBackTransition
- VTOL 基类与参数
  - `PX4-Autopilot/src/modules/vtol_att_control/vtol_type.h:55` 通用 VTOL 模式枚举
  - `PX4-Autopilot/src/modules/vtol_att_control/vtol_att_control_params.c:35` 参数定义
- 固定翼速率控制中的尾座变换
  - `PX4-Autopilot/src/modules/fw_rate_control/FixedwingRateControl.cpp:248`
  - `PX4-Autopilot/src/modules/fw_rate_control/FixedwingRateControl.cpp:314`
  - `PX4-Autopilot/src/modules/fw_rate_control/FixedwingRateControl.cpp:420`
- 控制分配（尾座）
  - `PX4-Autopilot/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessTailsitterVTOL.hpp:35`
  - `PX4-Autopilot/src/modules/control_allocator/ControlAllocator.cpp:228`

