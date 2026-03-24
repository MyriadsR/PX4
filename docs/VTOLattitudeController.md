
• 核心模块

  - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/vtol_att_control_main.cpp:68 定义 VtolAttitudeControl，在构造函数里根据 VT_TYPE 参数实例化
    具体机型控制器（尾座/倾转/标准），并注册推力、力矩、扰流板等话题发布者。
  - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/vtol_att_control_main.cpp:317 是主循环 Run()，通过订阅 vehicle_torque_setpoint_virtual_*
    和 vehicle_thrust_setpoint_virtual_* 接收多旋翼与固定翼控制器的“虚拟”输出，按当前模式决定是否运行控制逻辑。

  状态管理

  - vtol_att_control_main.cpp:114、:131、:160 分别轮询飞行器状态、动作请求与 MAVLink 指令，更新 _transition_command、_immediate_transition 等模式切换
    标志。
  - vtol_att_control_main.cpp:438 每次循环调用 _vtol_type->update_vtol_state()，具体决策由机型实现；随后根据模式调用 update_transition_state()、
    update_mc_state() 或 update_fw_state() 并同步姿态设定点。

  输出与安全

  - vtol_att_control_main.cpp:487 在机型层完成 fill_actuator_outputs() 后发布双矩阵推力/力矩设定点；针对自动模式，还在 vtol_att_control_main.cpp:510
    处理襟翼和扰流板指令。
  - vtol_att_control_main.cpp:214 实现 Quad-chute 安全栈，遇到高度、姿态或超时等触发条件时强制回到多旋翼模式；具体检查逻辑封装在
    VtolType::check_quadchute_condition() 等函数中。

  类型适配层

  - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/vtol_type.cpp:60 的 VtolType 基类持有姿态、推力、位置等共享指针，并提供
    update_mc_state()、update_fw_state() 等通用实现，同时管理 TECS 接入、过渡计时和 Quad-chute 判据。
  - vtol_type.cpp:152、:171、:330 展示前向过渡完成判据、过渡状态复位、超时判断等共性逻辑；机型差异集中在 standard.cpp、tiltrotor.cpp、tailsitter.cpp
    的重写函数里（如不同的推力混合和姿态轨迹）。

  参数接口

  - PX4_project/PX4-Autopilot/src/modules/vtol_att_control/vtol_att_control_params.c:35 列出全部 VTOL 姿态控制参数（过渡时间、空速阈值、差动推力、
    Quad-chute 限制等），主循环通过 parameters_update() 与 VtolType::parameters_update() 保持实时同步。