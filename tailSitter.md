  - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp 实现尾座（Tailsitter）VTOL 类型在 VTOL 姿态控制中的机型特定逻辑：
      - 模式状态机（MC/前向过渡/回退过渡/FW）
      - 过渡时姿态轨迹生成（四元数、旋转轴、速率）
      - 推力/力矩的混合和平滑
      - 将“虚拟”多旋翼/固定翼控制输出合成为两套输出矩阵（电机与舵面）

  关键常量与状态（见头文件）

  - 过渡判据（俯仰阈值）与混合时长:
      - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.h:23 PITCH_THRESHOLD_AUTO_TRANSITION_TO_FW = -60°
      - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.h:26 PITCH_THRESHOLD_AUTO_TRANSITION_TO_MC = -15°
      - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.h:29 B_TRANS_THRUST_BLENDING_DURATION = 0.5s
  - 尾座状态枚举与内部变量:
      - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.h:76 vtol_mode{MC_MODE, TRANSITION_FRONT_P1, TRANSITION_BACK, FW_MODE}
      - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.h:83 _vtol_mode 当前机型内部模式
      - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.h:100 _q_trans_start/_q_trans_sp/_trans_rot_axis 过渡起始/目标姿态与旋转轴

  模式状态机 update_vtol_state()

  - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:59
      - 固定翼系统故障时立即回到多旋翼：fixed_wing_system_failure → MC_MODE
      - 用户请求 MC：
          - FW_MODE → resetTransitionStates() → TRANSITION_BACK
          - 在回退过程中，按俯仰阈值（-15°）或超时切 MC：pitch >= -15° || t>VT_B_TRANS_DUR → MC_MODE（:94-115）
      - 用户请求 FW：
          - MC_MODE → resetTransitionStates() → TRANSITION_FRONT_P1（:117-142）
          - 前向过渡完成（见完成判据）后 → FW_MODE（:131-140）
      - 将尾座内部模式映射到通用 VTOL 模式，便于上层统一处理：ROTARY_WING/FIXED_WING/TRANSITION_*（:146-168）

  前/后过渡的姿态生成 update_transition_state()

  - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:155
  - 前置条件与初始化
      - 先调用基类更新过渡计时/记录最后一次 MC 推力等（VtolType::update_transition_state()）
      - 要求“虚拟”MC/FW姿态设定值必须是近期（<1s），否则保持上一姿态（:163-171）
      - 首次进入过渡时，计算旋转轴与起始姿态（:173-206）
          - 回退（TRANSITION_BACK）：
              - 当前姿态四元数 _q_trans_start = q(v_att)，取机体-z方向 z = -R_body.z
              - 旋转轴 axis = z × (0,0,-1)；将初始姿态组合为“机翼水平、给定 yaw”的姿态（如果 FW 虚拟姿态新鲜则用其 pitch，否则 0），再旋转到“多旋翼框
                架”（乘以 Rotation Y(-90°)）（:178-204）
          - 前向（TRANSITION_FRONT_P1）：
              - 以“机翼水平”的初始姿态（roll=0，pitch=mc虚拟设定的 pitch，yaw=mc虚拟设定 yaw）
              - 旋转轴由机体 x 与重力方向确定：x = R_body*[1,0,0]; axis = -x × (0,0,-1)（:206-214）
  - 轨迹推进与停止条件
      - 规范化四元数，计算倾斜角 tilt = acos(cos_tilt)（:218-224）
      - 前向过渡：
          - 过渡角速度 ω = 90° / max(VT_F_TRANS_DUR, 0.1)，按时间推进：q_sp = R(axis, t*ω) * q_start
          - 达到“距离 90° 还差 FW_PSP_OFF 的余度”时停止进一步抬头（:226-235）
      - 回退过渡：
          - 过渡角速度 ω = 90° / max(VT_B_TRANS_DUR, 0.1)，tilt>0则按轴回转（:237-246）
      - 设定推力/时间戳：_v_att_sp->thrust_body[2] = mc_virtual.thrust_z，回退过渡开始时做推力混合（:248-256，:258-265）

  推力/力矩输出合成 fill_actuator_outputs()

  - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:261
  - 目标：生成两套输出矩阵的推力/力矩设定
      - 矩阵0（电机，MC矩阵）：_thrust_setpoint_0/_torque_setpoint_0
      - 矩阵1（舵面，FW矩阵）：_thrust_setpoint_1/_torque_setpoint_1
  - 固定翼模式下（FW_MODE）
      - 将固定翼推力 x 映射为电机 z 推力：thrust0.z = -fw_virtual.thrust.x（:284-289）
      - 按差动推力位掩码将 FW 虚拟力矩映射为电机差动扭矩（yaw/pitch/roll 三轴独立启用与缩放）（:291-305）
      - 为切入 FW 后短时（<50ms）尚未有 FW 推力的窗口保持 MC 推力，避免电机停转（:307-315）
  - 其他模式（MC 或过渡）
      - 默认用 MC 虚拟推力/力矩（:317-327）
      - 回退过渡开始短时（<50ms）若 MC 控制尚未发布推力，用“上次 FW 推力”维持（:319-323）
  - 舵面输出（矩阵1）
      - 若 VT_ELEV_MC_LOCK=0 或当前非 MC_MODE，则从 FW 虚拟力矩直通到舵面矩阵（:329-336）
  - 说明
      - MC 推力定义为负 z（机体坐标），FW 推力沿正 x，函数内部转换了轴向与符号（注释见 blendThrottleAfterFrontTransition）

  TECS 接管等待 waiting_on_tecs()

  - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:246
  - 在前向过渡刚完成、TECS 尚未运行期间，将 MC 推力沿机体系 x 方向保留一段时间（避免推力空档）

  前向过渡完成判据 isFrontTransitionCompletedBase()

  - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:337
  - 逻辑：俯仰角达到阈值（≤ -60°），且（若有空速）空速≥VT_ARSP_TRANS；否则（无空速）仅姿态判据即完成

  推力混合（前/后过渡）

  - 前向过渡完成后混合（平滑从 MC 推力到 FW 推力的过渡）
      - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:359
        thrust_body[0] = scale*FW + (1-scale)*(-last_mc_thrust)
  - 回退过渡初期混合（避免 MC 接管前推力空档）
      - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/tailsitter.cpp:365
        thrust_body[2] = scale*MC + (1-scale)*(-last_fw_thrust)

  与基类/上层模块的配合点

  - 计时与 Quad-chute、安全判据、TECS 接入、最后一次 MC/FW 推力的记录等由基类 VtolType 处理：
      - 过渡计时与最后推力更新：PX4_project/PX4-Autopilot/src/modules/vtol_att_control/vtol_type.cpp:152-165
      - 前向过渡最小时间/超时/空速条件：vtol_type.cpp:167-193, 327-342
      - TECS 接管与前向过渡后推力混合窗口：vtol_type.cpp:121-147
  - 上层主循环将尾座内部模式映射为通用 VTOL 模式并驱动工作流：
      - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/vtol_att_control_main.cpp:440-485
        在 TRANSITION_*/ROTARY_WING/FIXED_WING 状态下分别调用 update_transition_state()、update_mc_state()、update_fw_state() 并发布姿态设定

  参数与调参要点（相关使用处）

  - 前向/后退过渡时长：VT_F_TRANS_DUR（:228-236）、VT_B_TRANS_DUR（:237-246）
  - 前向过渡俯仰偏置：FW_PSP_OFF（限制接近 90° 的终点，:230-235）
  - 俯仰阈值（完成/回退）：tailsitter.h:23,26
  - 回退推力混合时长：tailsitter.h:29（在 update_transition_state() 中使用）
  - 差动推力使能与缩放：VT_FW_DIFTHR_EN 与 VT_FW_DIFTHR_S_*（:291-305）
  - 舵面在悬停锁定：VT_ELEV_MC_LOCK（:329-336）

  坐标系与帧转换注意

  - MC 推力定义：机体坐标 -z 方向（负值表示上推）
  - FW 推力定义：机体坐标 +x 方向（正值）
  - 回退/前向过渡的旋转轴依赖机体方向与重力方向叉乘；回退初始化时将姿态转到“多旋翼框架”以便 MC 控制器接手（:199-204）
  - 固定翼控制中还有尾座专用的速率/输出帧变换（位于固定翼速率控制器），与本文件的轴/符号约定一致

  典型流程概览

  - 前向过渡（MC→FW）：
      - MC_MODE → TRANSITION_FRONT_P1 → 更新四元数姿态轨迹（按轴匀速旋转）→ 俯仰≤-60° 且（若有空速）≥VT_ARSP_TRANS → FW_MODE → TECS 接管前短时保持与推
        力混合
  - 回退过渡（FW→MC）：
      - FW_MODE → TRANSITION_BACK → 逆向轴匀速旋转 → 俯仰≥-15° 或超时 → MC_MODE
      - 开始短时保持上次 FW 推力，平滑到 MC 推力