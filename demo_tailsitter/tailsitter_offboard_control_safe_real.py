#!/usr/bin/env python3
"""
Tailsitter VTOL Offboard Control (Real-Flight Safer Edition)

主要增强：
1) 连接参数化（支持真机链路地址）
2) 包线检查（高度/半径/速度/姿态）
3) 关键阶段超时自动 RTL
4) 异常或用户中断触发急停流程（优先 RTL）

注意：该脚本是“更安全”而非“绝对安全”。真机前请先系留测试/HITL。
"""

import argparse
import asyncio
import math
import sys
from dataclasses import dataclass
from typing import Optional

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


@dataclass
class MissionConfig:
    system_address: str

    # Mission geometry
    takeoff_alt_m: float
    prep_n_m: float
    prep_alt_m: float
    fw_leg_n_m: float
    fw_leg_e_m: float
    fw_alt_m: float
    home_return_alt_m: float

    # Timeouts / thresholds
    waypoint_timeout_s: float
    transition_timeout_s: float
    rtl_wait_s: float
    waypoint_tolerance_m: float
    stable_hits_required: int

    # Safety envelope
    max_radius_m: float
    max_alt_m: float
    max_speed_m_s: float
    max_tilt_deg: float


async def wait_until_reached_ned(
    drone: System,
    target_n: float,
    target_e: float,
    target_d: float,
    timeout_s: float,
    tolerance_m: float,
    stable_hits_required: int,
    abort_event: asyncio.Event,
) -> bool:
    """等待无人机到达指定 NED 目标点（超时/急停可中断）。"""
    start_time = asyncio.get_event_loop().time()
    last_print_time = 0.0
    stable_hits = 0

    async for pv_ned in drone.telemetry.position_velocity_ned():
        if abort_event.is_set():
            return False

        pos = pv_ned.position
        dn = target_n - pos.north_m
        de = target_e - pos.east_m
        dd = target_d - pos.down_m
        distance = math.sqrt(dn * dn + de * de + dd * dd)

        now = asyncio.get_event_loop().time()
        if now - last_print_time >= 0.2:
            print(
                f"[到点检测] 当前NED=({pos.north_m:.1f}, {pos.east_m:.1f}, {pos.down_m:.1f}) "
                f"目标=({target_n:.1f}, {target_e:.1f}, {target_d:.1f}) 距离={distance:.1f}m"
            )
            last_print_time = now

        if distance <= tolerance_m:
            stable_hits += 1
            if stable_hits >= stable_hits_required:
                print(
                    f"-- 已到达目标点 (误差 {distance:.1f}m <= {tolerance_m:.1f}m, "
                    f"连续命中 {stable_hits_required} 次)"
                )
                return True
        else:
            stable_hits = 0

        if now - start_time > timeout_s:
            print(f"-- 到点等待超时 ({timeout_s:.0f}s)")
            return False


async def wait_for_vtol_state(
    drone: System,
    expected_token: str,
    timeout_s: float,
    abort_event: asyncio.Event,
) -> bool:
    """等待 VTOL 状态包含指定 token（如 FIXED_WING / MULTICOPTER）。"""
    start_time = asyncio.get_event_loop().time()

    async for vtol_state in drone.telemetry.vtol_state():
        if abort_event.is_set():
            return False

        s = str(vtol_state)
        print(f"当前VTOL状态: {s}")

        if expected_token in s:
            return True

        if asyncio.get_event_loop().time() - start_time > timeout_s:
            print(f"-- VTOL状态等待超时 ({timeout_s:.0f}s)")
            return False


async def safe_rtl(drone: System, reason: str, abort_event: asyncio.Event) -> None:
    """统一急停处置：停止 Offboard 后执行 RTL。"""
    if abort_event.is_set():
        # 已在急停流程中，避免重复执行
        return

    abort_event.set()
    print(f"\n[急停] {reason}")

    try:
        await drone.offboard.stop()
        print("-- 已停止 Offboard")
    except Exception as e:
        print(f"-- 停止 Offboard 失败(可忽略): {e}")

    try:
        await drone.action.return_to_launch()
        print("-- 已触发 RTL")
    except Exception as e:
        print(f"-- RTL 触发失败: {e}")


async def monitor_position_envelope(
    drone: System,
    cfg: MissionConfig,
    abort_event: asyncio.Event,
    abort_reason_holder: dict,
) -> None:
    """监控位置/速度包线。超限后设置 abort_event。"""
    async for pv_ned in drone.telemetry.position_velocity_ned():
        if abort_event.is_set():
            return

        pos = pv_ned.position
        vel = pv_ned.velocity

        radius = math.hypot(pos.north_m, pos.east_m)
        alt_m = -pos.down_m
        speed_m_s = math.sqrt(
            vel.north_m_s * vel.north_m_s +
            vel.east_m_s * vel.east_m_s +
            vel.down_m_s * vel.down_m_s
        )

        reason = None
        if radius > cfg.max_radius_m:
            reason = f"超出水平包线: 半径 {radius:.1f}m > {cfg.max_radius_m:.1f}m"
        elif alt_m > cfg.max_alt_m:
            reason = f"超出高度包线: 高度 {alt_m:.1f}m > {cfg.max_alt_m:.1f}m"
        elif speed_m_s > cfg.max_speed_m_s:
            reason = f"超出速度包线: 速度 {speed_m_s:.1f}m/s > {cfg.max_speed_m_s:.1f}m/s"

        if reason:
            abort_reason_holder["reason"] = reason
            abort_event.set()
            return


async def monitor_attitude_envelope(
    drone: System,
    cfg: MissionConfig,
    abort_event: asyncio.Event,
    abort_reason_holder: dict,
) -> None:
    """监控姿态包线（横滚/俯仰角）。"""
    async for euler in drone.telemetry.attitude_euler():
        if abort_event.is_set():
            return

        if abs(euler.roll_deg) > cfg.max_tilt_deg or abs(euler.pitch_deg) > cfg.max_tilt_deg:
            abort_reason_holder["reason"] = (
                "超出姿态包线: "
                f"roll={euler.roll_deg:.1f}deg, pitch={euler.pitch_deg:.1f}deg, "
                f"limit={cfg.max_tilt_deg:.1f}deg"
            )
            abort_event.set()
            return


async def guarded_setpoint_and_wait(
    drone: System,
    cfg: MissionConfig,
    abort_event: asyncio.Event,
    n: float,
    e: float,
    d: float,
    yaw_deg: float,
    timeout_s: Optional[float] = None,
    tolerance_m: Optional[float] = None,
) -> bool:
    """发送目标点并等待到达；期间若触发急停则返回 False。"""
    if abort_event.is_set():
        return False

    await drone.offboard.set_position_ned(PositionNedYaw(n, e, d, yaw_deg))

    ok = await wait_until_reached_ned(
        drone,
        target_n=n,
        target_e=e,
        target_d=d,
        timeout_s=timeout_s if timeout_s is not None else cfg.waypoint_timeout_s,
        tolerance_m=tolerance_m if tolerance_m is not None else cfg.waypoint_tolerance_m,
        stable_hits_required=cfg.stable_hits_required,
        abort_event=abort_event,
    )
    return ok and (not abort_event.is_set())


async def run(cfg: MissionConfig) -> int:
    drone = System()
    abort_event = asyncio.Event()
    abort_reason_holder = {"reason": "未提供原因"}
    monitor_tasks = []

    try:
        print(f"连接飞控: {cfg.system_address}")
        await drone.connect(system_address=cfg.system_address)

        print("等待无人机连接...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("-- 已连接到无人机")
                break

        print("等待位置估计就绪...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- 位置估计就绪")
                break

        # 提高关键遥测频率，减小控制与感知延迟
        try:
            await drone.telemetry.set_rate_position_velocity_ned(30.0)
            await drone.telemetry.set_rate_attitude_euler(30.0)
            print("-- 已设置位置/姿态遥测频率: 30Hz")
        except Exception as e:
            print(f"-- 设置遥测频率失败，使用默认频率: {e}")

        # 启动包线监控
        monitor_tasks.append(asyncio.create_task(
            monitor_position_envelope(drone, cfg, abort_event, abort_reason_holder)
        ))
        monitor_tasks.append(asyncio.create_task(
            monitor_attitude_envelope(drone, cfg, abort_event, abort_reason_holder)
        ))

        print("设置初始 Offboard 设定点...")
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        print("解锁无人机...")
        await drone.action.arm()
        print("-- 无人机已解锁")

        print("启动 Offboard 模式...")
        try:
            await drone.offboard.start()
            print("-- Offboard 模式已启动")
        except OffboardError as e:
            print(f"启动 Offboard 失败: {e}")
            await drone.action.disarm()
            return 1

        # Stage 1: Takeoff
        print("\n[阶段1] 垂直起飞")
        ok = await guarded_setpoint_and_wait(
            drone,
            cfg,
            abort_event,
            n=0.0,
            e=0.0,
            d=-cfg.takeoff_alt_m,
            yaw_deg=0.0,
            timeout_s=45.0,
            tolerance_m=2.0,
        )
        if not ok:
            await safe_rtl(drone, "起飞阶段失败或被急停", abort_event)
            await asyncio.sleep(cfg.rtl_wait_s)
            return 1

        # Stage 2: Prep for transition
        print("\n[阶段2] 前飞加速准备过渡")
        ok = await guarded_setpoint_and_wait(
            drone,
            cfg,
            abort_event,
            n=cfg.prep_n_m,
            e=0.0,
            d=-cfg.prep_alt_m,
            yaw_deg=0.0,
            timeout_s=40.0,
            tolerance_m=4.0,
        )
        if not ok:
            await safe_rtl(drone, "过渡前加速阶段失败或被急停", abort_event)
            await asyncio.sleep(cfg.rtl_wait_s)
            return 1

        # Stage 3: Transition to FW
        print("\n[阶段3] 过渡到固定翼")
        await drone.action.transition_to_fixedwing()
        fw_ok = await wait_for_vtol_state(
            drone,
            expected_token="FIXED_WING",
            timeout_s=cfg.transition_timeout_s,
            abort_event=abort_event,
        )
        if not fw_ok:
            await safe_rtl(drone, "固定翼过渡超时/失败", abort_event)
            await asyncio.sleep(cfg.rtl_wait_s)
            return 1

        # Stage 4: FW rectangle
        print("\n[阶段4] 固定翼航线")
        waypoints = [
            (cfg.fw_leg_n_m, 0.0, -cfg.fw_alt_m, 0.0, "航点1"),
            (cfg.fw_leg_n_m, cfg.fw_leg_e_m, -cfg.fw_alt_m, 90.0, "航点2"),
            (0.0, cfg.fw_leg_e_m, -cfg.fw_alt_m, 180.0, "航点3"),
            (0.0, 30.0, -cfg.fw_alt_m, 270.0, "航点4"),
        ]

        for n, e, d, yaw, name in waypoints:
            print(f"-- 飞向{name}")
            ok = await guarded_setpoint_and_wait(
                drone,
                cfg,
                abort_event,
                n=n,
                e=e,
                d=d,
                yaw_deg=yaw,
                timeout_s=cfg.waypoint_timeout_s,
                tolerance_m=max(cfg.waypoint_tolerance_m, 8.0),
            )
            if not ok:
                await safe_rtl(drone, f"{name} 到点超时/失败", abort_event)
                await asyncio.sleep(cfg.rtl_wait_s)
                return 1

        # Stage 5: Transition back to MC
        print("\n[阶段5] 过渡回多旋翼")
        await drone.action.transition_to_multicopter()
        mc_ok = await wait_for_vtol_state(
            drone,
            expected_token="MULTICOPTER",
            timeout_s=cfg.transition_timeout_s,
            abort_event=abort_event,
        )
        if not mc_ok:
            await safe_rtl(drone, "多旋翼过渡超时/失败", abort_event)
            await asyncio.sleep(cfg.rtl_wait_s)
            return 1

        # Stage 6: Return above home
        print("\n[阶段6] 返回 Home 上方")
        ok = await guarded_setpoint_and_wait(
            drone,
            cfg,
            abort_event,
            n=0.0,
            e=0.0,
            d=-cfg.home_return_alt_m,
            yaw_deg=270.0,
            timeout_s=60.0,
            tolerance_m=2.5,
        )
        if not ok:
            await safe_rtl(drone, "返航点到点失败", abort_event)
            await asyncio.sleep(cfg.rtl_wait_s)
            return 1

        # Stage 7: Land
        print("\n[阶段7] 停止 Offboard 并降落")
        try:
            await drone.offboard.stop()
            print("-- Offboard 已停止")
        except Exception as e:
            print(f"-- Offboard 停止失败(可继续降落): {e}")

        await drone.action.land()
        print("等待降落完成...")
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                break

        await asyncio.sleep(2)
        try:
            await drone.action.disarm()
        except Exception:
            pass

        print("\n任务完成")
        return 0

    except KeyboardInterrupt:
        # 用户中断视为急停指令
        await safe_rtl(drone, "收到用户中断 (Ctrl+C)", abort_event)
        await asyncio.sleep(cfg.rtl_wait_s)
        return 2
    except Exception as e:
        await safe_rtl(drone, f"任务异常: {e}", abort_event)
        await asyncio.sleep(cfg.rtl_wait_s)
        return 3
    finally:
        for t in monitor_tasks:
            t.cancel()
        await asyncio.gather(*monitor_tasks, return_exceptions=True)


def parse_args() -> MissionConfig:
    parser = argparse.ArgumentParser(description="Tailsitter 真机安全版 Offboard 控制")

    parser.add_argument("--system-address", default="udp://:14540", help="MAVSDK 连接地址")

    parser.add_argument("--takeoff-alt", type=float, default=20.0, help="起飞高度(m)")
    parser.add_argument("--prep-n", type=float, default=20.0, help="过渡前北向距离(m)")
    parser.add_argument("--prep-alt", type=float, default=30.0, help="过渡前高度(m)")
    parser.add_argument("--fw-leg-n", type=float, default=120.0, help="固定翼北向航段长度(m)")
    parser.add_argument("--fw-leg-e", type=float, default=120.0, help="固定翼东向航段长度(m)")
    parser.add_argument("--fw-alt", type=float, default=35.0, help="固定翼高度(m)")
    parser.add_argument("--home-return-alt", type=float, default=20.0, help="返回 Home 上方高度(m)")

    parser.add_argument("--waypoint-timeout", type=float, default=90.0, help="航点超时(s)")
    parser.add_argument("--transition-timeout", type=float, default=20.0, help="过渡等待超时(s)")
    parser.add_argument("--rtl-wait", type=float, default=12.0, help="触发 RTL 后等待时间(s)")
    parser.add_argument("--waypoint-tolerance", type=float, default=6.0, help="到点容差(m)")
    parser.add_argument("--stable-hits", type=int, default=3, help="连续命中次数")

    parser.add_argument("--max-radius", type=float, default=450.0, help="最大水平半径(m)")
    parser.add_argument("--max-alt", type=float, default=80.0, help="最大高度(m)")
    parser.add_argument("--max-speed", type=float, default=35.0, help="最大速度(m/s)")
    parser.add_argument("--max-tilt", type=float, default=65.0, help="最大横滚/俯仰角(deg)")

    args = parser.parse_args()

    return MissionConfig(
        system_address=args.system_address,
        takeoff_alt_m=args.takeoff_alt,
        prep_n_m=args.prep_n,
        prep_alt_m=args.prep_alt,
        fw_leg_n_m=args.fw_leg_n,
        fw_leg_e_m=args.fw_leg_e,
        fw_alt_m=args.fw_alt,
        home_return_alt_m=args.home_return_alt,
        waypoint_timeout_s=args.waypoint_timeout,
        transition_timeout_s=args.transition_timeout,
        rtl_wait_s=args.rtl_wait,
        waypoint_tolerance_m=args.waypoint_tolerance,
        stable_hits_required=args.stable_hits,
        max_radius_m=args.max_radius,
        max_alt_m=args.max_alt,
        max_speed_m_s=args.max_speed,
        max_tilt_deg=args.max_tilt,
    )


if __name__ == "__main__":
    config = parse_args()
    code = asyncio.run(run(config))
    sys.exit(code)
