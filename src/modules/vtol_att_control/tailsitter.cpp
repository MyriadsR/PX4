/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
* @file tailsitter.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#include "tailsitter.h"
#include "vtol_att_control_main.h"

using namespace matrix;

Tailsitter::Tailsitter(VtolAttitudeControl *attc) :
	VtolType(attc)
{
}

void
Tailsitter::parameters_update()
{
	VtolType::updateParams();

}

void Tailsitter::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/

	/*
	使用双向开关进行模式切换
	前向过渡：飞行器在多旋翼控制模式下开始倾斜，获得前向速度，当速度和俯仰角足够时，进入固定翼模式
	反向过渡：俯仰角再次由多旋翼模式控制，当达到足够的俯仰角时，完全切换到多旋翼模式*/

	// 当检测到固定翼系统故障时，立即切换到多旋翼(MC)模式以确保安全
	if (_vtol_vehicle_status->fixed_wing_system_failure) {
		// Failsafe event, switch to MC mode immediately
		if (_vtol_mode != vtol_mode::MC_MODE) {
			_transition_start_timestamp = hrt_absolute_time();
		}

		_vtol_mode = vtol_mode::MC_MODE;

	// 多旋翼模式请求处理： 当用户请求切换到多旋翼模式时，根据当前状态进行不同的处理
	} else if (!_attc->is_fixed_wing_requested()) {

		switch (_vtol_mode) { // user switchig to MC mode
		case vtol_mode::MC_MODE:
			break;
		
		// 如果当前处于固定翼(FW)模式，开始反向过渡(TRANSITION_BACK)
		case vtol_mode::FW_MODE:
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_BACK;
			break;
		
		// 如果当前处于前向过渡(TRANSITION_FRONT_P1)模式，直接切换到MC模式（故障安全）
		case vtol_mode::TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_mode = vtol_mode::MC_MODE;
			break;
		
		// 如果当前处于反向过渡(TRANSITION_BACK)模式，检查是否达到切换到MC模式的条件（俯仰角或时间阈值）
		case vtol_mode::TRANSITION_BACK:
			const float pitch = Eulerf(Quatf(_v_att->q)).theta();

			// check if we have reached pitch angle to switch to MC mode
			if (pitch >= PITCH_THRESHOLD_AUTO_TRANSITION_TO_MC || _time_since_trans_start > _param_vt_b_trans_dur.get()) {
				_vtol_mode = vtol_mode::MC_MODE;
			}

			break;
		}

	} else {  // user switchig to FW mode

		switch (_vtol_mode) {
		case vtol_mode::MC_MODE:
			// initialise a front transition
			_vtol_mode = vtol_mode::TRANSITION_FRONT_P1;
			resetTransitionStates();
			break;

		case vtol_mode::FW_MODE:
			break;

		case vtol_mode::TRANSITION_FRONT_P1: {
				// 检查前向过渡是否完成
				if (isFrontTransitionCompleted()) {
					_vtol_mode = vtol_mode::FW_MODE;
					_trans_finished_ts = hrt_absolute_time();
				}

				break;
			}

		case vtol_mode::TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_mode = vtol_mode::FW_MODE;
			_trans_finished_ts = hrt_absolute_time();
			break;
		}
	}

	// map tailsitter specific control phases to simple control modes
	// 将尾座式特定控制阶段映射到通用控制模式
	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:
		_common_vtol_mode = mode::ROTARY_WING;
		_flag_was_in_trans_mode = false;
		break;

	case vtol_mode::FW_MODE:
		_common_vtol_mode = mode::FIXED_WING;
		_flag_was_in_trans_mode = false;
		break;

	case vtol_mode::TRANSITION_FRONT_P1:
		_common_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_BACK:
		_common_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

// 此方法负责处理过渡状态下的姿态控制
/*
这个函数的主要目的是在VTOL过渡期间计算和更新飞行器的姿态设定点，确保过渡过程平稳、安全。
它会根据当前的过渡模式（前向或反向）计算合适的姿态，并生成相应的控制指令*/
void Tailsitter::update_transition_state()
{
	VtolType::update_transition_state();

	const hrt_abstime now = hrt_absolute_time();

	// we need the incoming (virtual) mc attitude setpoints to be recent, otherwise return (means the previous setpoint stays active)
	if (_mc_virtual_att_sp->timestamp < (now - 1_s)) {
		return;
	}

	if (!_flag_was_in_trans_mode) {
		_flag_was_in_trans_mode = true;

		if (_vtol_mode == vtol_mode::TRANSITION_BACK) {
			// calculate rotation axis for transition.
			_q_trans_start = Quatf(_v_att->q);	// 获取当前姿态四元数
			Vector3f z = -_q_trans_start.dcm_z();	// 获取当前z轴方向（指向机体坐标系下方）
			_trans_rot_axis = z.cross(Vector3f(0.f, 0.f, -1.f));	// 计算旋转轴（当前z轴与垂直向下方向的叉积）

			// as heading setpoint we choose the heading given by the direction the vehicle points
			// 选择航向设定点：使用飞行器当前指向的方向作为航向
			// atan2f(y, x) 计算出 z 向量在水平面的投影方向（即飞行器机头指向的航向）
			const float yaw_sp = atan2f(z(1), z(0));

			// the intial attitude setpoint for a backtransition is a combination of the current fw pitch setpoint,
			// the yaw setpoint and zero roll since we want wings level transition.
			// If for some reason the fw attitude setpoint is not recent then don't use it and assume 0 pitch
			// 反向过渡的初始姿态设定点是当前固定翼俯仰设定点、航向设定点和零滚转的组合
			if (_fw_virtual_att_sp->timestamp > (now - 1_s)) {
				const float pitch_body = Eulerf(Quatf(_fw_virtual_att_sp->q_d)).theta();
				_q_trans_start = Eulerf(0.f, pitch_body, yaw_sp);

			} else {
				_q_trans_start = Eulerf(0.f, 0.f, yaw_sp);
			}

			// attitude during transitions are controlled by mc attitude control so rotate the desired attitude to the
			// multirotor frame
			// 过渡期间的姿态由多旋翼姿态控制，因此将期望姿态旋转到多旋翼坐标系
			// 尾座式飞行器在固定翼模式下，机体 x 轴朝前（飞行方向）
			// 在多旋翼模式下，机体 z 轴朝下（推力方向）
			// 两者相差 90° 俯仰旋转
			_q_trans_start = _q_trans_start * Quatf(Eulerf(0, -M_PI_2_F, 0));

		} else if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {
			// initial attitude setpoint for the transition should be with wings level
			// 设置初始姿态：保持当前俯仰和航向，但将滚转设为0（机翼水平）
			const Eulerf setpoint_euler(Quatf(_mc_virtual_att_sp->q_d));
			_q_trans_start = Eulerf(0.f, setpoint_euler.theta(), setpoint_euler.psi());
			
			// 计算旋转轴：通过当前姿态的x轴与垂直方向的叉积得到
			Vector3f x = Dcmf(Quatf(_v_att->q)) * Vector3f(1.f, 0.f, 0.f);
			_trans_rot_axis = -x.cross(Vector3f(0.f, 0.f, -1.f));
		}

		// 将初始化的姿态设定为过渡姿态的起始点
		_q_trans_sp = _q_trans_start;
	}

	// ensure input quaternions are exactly normalized because acosf(1.00001) == NaN
	_q_trans_sp.normalize();

	// tilt angle (zero if vehicle nose points up (hover))
	// 计算当前倾斜角度：通过四元数计算倾斜角的余弦值，然后取反余弦得到实际倾斜角度
	// 倾斜角度为0表示飞行器机头指向上方（悬停状态）
	const float cos_tilt = math::constrain(_q_trans_sp(0) * _q_trans_sp(0) - _q_trans_sp(1) * _q_trans_sp(1) -
					       _q_trans_sp(2) * _q_trans_sp(2) + _q_trans_sp(3) * _q_trans_sp(3), -1.f, 1.f);
	const float tilt = acosf(cos_tilt);

	if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {

		// calculate pitching rate - and constrain to at least 0.1s transition time
		// _param_vt_f_trans_dur.get() 是用户设定的前向过渡时长（秒）
		// 若参数设为 2.0 秒：trans_pitch_rate = π/2 / 2.0 = 0.785 弧度/秒 ≈ 45°/秒
		const float trans_pitch_rate = M_PI_2_F / math::max(_param_vt_f_trans_dur.get(), 0.1f);

		// 当倾斜角度小于阈值时，根据时间和旋转轴计算新的姿态设定点
		// _time_since_trans_start：过渡开始后经过的时间（秒）乘以俯仰速率 → 得到累计旋转角度（弧度）
		// 四元数乘法 q_delta * q_start 表示：在起始姿态基础上，叠加增量旋转
		if (tilt < M_PI_2_F - math::radians(_param_fw_psp_off.get())) {
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
						       _time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
		}

	} else if (_vtol_mode == vtol_mode::TRANSITION_BACK) {

		// calculate pitching rate - and constrain to at least 0.1s transition time
		const float trans_pitch_rate = M_PI_2_F / math::max(_param_vt_b_trans_dur.get(), 0.1f);

		// 当倾斜角度大于0.01弧度（约0.57度）时，计算新的姿态设定点
		if (tilt > 0.01f) {
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
						       _time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
		}
	}

	// 设置垂直方向（z轴）的推力为多旋翼虚拟推力设定点
	_v_att_sp->thrust_body[2] = _mc_virtual_att_sp->thrust_body[2];

	/*
	_time_since_trans_start是过渡开始后经过的时间
	progress是过渡进度，从0（开始）到1（完成）
	随着反向过渡的进行，progress逐渐增加，多旋翼推力的权重逐渐增加，固定翼推力的权重逐渐减少*/
	if (_vtol_mode == vtol_mode::TRANSITION_BACK) {
		const float progress = math::constrain(_time_since_trans_start / B_TRANS_THRUST_BLENDING_DURATION, 0.f, 1.f);
		blendThrottleBeginningBackTransition(progress);
	}

	_v_att_sp->timestamp = hrt_absolute_time();

	const Eulerf euler_sp(_q_trans_sp);
	_q_trans_sp.copyTo(_v_att_sp->q_d);
}

void Tailsitter::waiting_on_tecs()
{
	// copy the last trust value from the front transition
	_v_att_sp->thrust_body[0] = -_last_thr_in_mc;
}

void Tailsitter::update_fw_state()
{
	VtolType::update_fw_state();

}

/**
* Write data to actuator output topic.
*/
void Tailsitter::fill_actuator_outputs()
{
	// 初始化扭矩设定点0（多旋翼控制）
	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _vehicle_torque_setpoint_virtual_mc->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	// 初始化扭矩设定点1（固定翼控制）
	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_torque_setpoint_1->timestamp_sample = _vehicle_torque_setpoint_virtual_fw->timestamp_sample;
	_torque_setpoint_1->xyz[0] = 0.f;
	_torque_setpoint_1->xyz[1] = 0.f;
	_torque_setpoint_1->xyz[2] = 0.f;

	// 初始化推力设定点0（多旋翼控制）
	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _vehicle_thrust_setpoint_virtual_mc->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	// 初始化推力设定点1（固定翼控制）
	_thrust_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp_sample = _vehicle_thrust_setpoint_virtual_fw->timestamp_sample;
	_thrust_setpoint_1->xyz[0] = 0.f;
	_thrust_setpoint_1->xyz[1] = 0.f;
	_thrust_setpoint_1->xyz[2] = 0.f;

	// Motors
	if (_vtol_mode == vtol_mode::FW_MODE) {
		// 将固定翼的前向推力转换为多旋翼z轴相反方向推力
		_thrust_setpoint_0->xyz[2] = -_vehicle_thrust_setpoint_virtual_fw->xyz[0];

		/* allow differential thrust if enabled */
		// 如果启用了差分推力，则设置相应的扭矩
		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::YAW_BIT)) {
			_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0] * _param_vt_fw_difthr_s_y.get();
		}

		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::PITCH_BIT)) {
			_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1] * _param_vt_fw_difthr_s_p.get();
		}

		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::ROLL_BIT)) {
			_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2] * _param_vt_fw_difthr_s_r.get();
		}

		// for the short period after switching to FW where there is no thrust published yet from the FW controller,
		// keep publishing the last MC thrust to keep the motors running
		// 在刚切换到固定翼模式时，固定翼控制器可能还没有发布推力值
    	// 此时保持最后的多旋翼推力值，以保持电机运转
		if (hrt_elapsed_time(&_trans_finished_ts) < 50_ms) {
			_thrust_setpoint_0->xyz[2] = _last_thr_in_mc;
			_torque_setpoint_0->xyz[0] = 0.f;
			_torque_setpoint_0->xyz[1] = 0.f;
			_torque_setpoint_0->xyz[2] = 0.f;
		}

	} 
	// 多旋翼模式和过渡模式
	else {
		_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2];

		// for the short period after starting the backtransition where there is no thrust published yet from the MC controller,
		// keep publishing the last FW thrust to keep the motors running
		// 在刚开始反向过渡时，多旋翼控制器可能还没有发布推力值
    	// 此时保持最后的固定翼推力值，以保持电机运转
		if (_vtol_mode != vtol_mode::TRANSITION_FRONT_P1 && hrt_elapsed_time(&_transition_start_timestamp) < 50_ms) {
			_thrust_setpoint_0->xyz[2] = -_last_thr_in_fw_mode;
		}

		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0];
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1];
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2];
	}

	// Control surfaces
	if (!_param_vt_elev_mc_lock.get() || _vtol_mode != vtol_mode::MC_MODE) {
		_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
		_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		_torque_setpoint_1->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2];
	}
}

/*
判断前向过渡是否完成，主要考虑两个条件：
- 俯仰角是否达到阈值（-60度）
- 如果有空速传感器，空速是否达到过渡阈值*/
bool Tailsitter::isFrontTransitionCompletedBase()
{
	const bool airspeed_triggers_transition = PX4_ISFINITE(_attc->get_calibrated_airspeed());

	bool transition_to_fw = false;
	const float pitch = Eulerf(Quatf(_v_att->q)).theta();

	if (pitch <= PITCH_THRESHOLD_AUTO_TRANSITION_TO_FW) {
		if (airspeed_triggers_transition) {
			transition_to_fw = _attc->get_calibrated_airspeed() >= _param_vt_arsp_trans.get() ;

		} else {
			transition_to_fw = true;
		}
	}

	return transition_to_fw;
}

void Tailsitter::blendThrottleAfterFrontTransition(float scale)
{
	// note: MC throttle is negative (as in negative z), while FW throttle is positive (positive x)
	_v_att_sp->thrust_body[0] = scale * _v_att_sp->thrust_body[0] + (1.f - scale) * (-_last_thr_in_mc);
}

/*
- 当scale为0时，完全使用固定翼模式的推力
- 当scale为1时，完全使用多旋翼模式的推力
_v_att_sp->thrust_body[2]是姿态设定点结构体中的z轴推力分量，
在机体坐标系中，z轴通常指向下方，因此正值表示向下推力
前部分表示当前多旋翼模式的推力贡献, 后部分是保存的固定翼模式下的最后推力值*/
void Tailsitter::blendThrottleBeginningBackTransition(float scale)
{
	_v_att_sp->thrust_body[2] = scale * _v_att_sp->thrust_body[2] + (1.f - scale) * (-_last_thr_in_fw_mode);
}
