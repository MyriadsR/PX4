#!/usr/bin/env python3
"""
Tailsitter VTOL Offboard Control Script
使用Offboard模式手动控制Tailsitter完成完整飞行流程
包括：垂直起飞 -> 前向过渡到固定翼 -> 固定翼飞行 -> 反向过渡 -> 降落

使用MAVSDK进行控制
"""

import asyncio
import sys
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw)


async def run():
    """主函数：连接无人机并执行Offboard控制"""

    # 创建System对象并连接到PX4 SITL
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("等待无人机连接...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- 已连接到无人机!")
            break

    # 等待位置估计就绪
    print("等待位置估计...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- 位置估计就绪")
            break

    # 设置初始位置设定点（NED坐标系：North, East, Down）
    print("\n设置初始Offboard模式...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    # 解锁无人机
    print("解锁无人机...")
    await drone.action.arm()
    print("-- 无人机已解锁")

    # 启动Offboard模式
    print("启动Offboard模式...")
    try:
        await drone.offboard.start()
        print("-- Offboard模式已启动")
    except OffboardError as error:
        print(f"启动Offboard模式失败: {error}")
        await drone.action.disarm()
        return

    # =============== 阶段1: 垂直起飞 ===============
    print("\n[阶段1] 垂直起飞到20米...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -20.0, 0.0))

    # 等待到达目标高度
    await asyncio.sleep(10)
    print("-- 到达起飞高度")

    # =============== 阶段2: 准备过渡 - 加速前飞 ===============
    print("\n[阶段2] 加速前飞，准备过渡到固定翼模式...")
    # 向前（北）移动50米，保持高度20米
    await drone.offboard.set_position_ned(PositionNedYaw(50.0, 0.0, -20.0, 0.0))
    await asyncio.sleep(8)

    # =============== 阶段3: 切换到固定翼模式 ===============
    print("\n[阶段3] 请求切换到固定翼模式...")
    # 使用VTOL过渡命令
    try:
        await drone.action.transition_to_fixedwing()
        print("-- 已发送固定翼过渡命令")
    except Exception as e:
        print(f"过渡命令发送失败: {e}")

    # 监控VTOL状态
    print("监控VTOL状态...")
    transition_complete = False
    timeout = 15  # 15秒超时
    start_time = asyncio.get_event_loop().time()

    while not transition_complete:
        async for vtol_state in drone.telemetry.vtol_state():
            print(f"当前VTOL状态: {vtol_state}")
            if "FIXED_WING" in str(vtol_state) or "FW" in str(vtol_state):
                transition_complete = True
                print("-- 已完成过渡到固定翼模式")
                break
            if asyncio.get_event_loop().time() - start_time > timeout:
                print("-- 过渡超时，继续下一步")
                transition_complete = True
                break
            await asyncio.sleep(1)
            break

    # =============== 阶段4: 固定翼模式飞行 ===============
    print("\n[阶段4] 固定翼模式飞行 - 执行矩形航线...")

    # 航点1: 向前（北）飞到100米
    print("-- 飞向航点1 (北100米)")
    await drone.offboard.set_position_ned(PositionNedYaw(100.0, 0.0, -25.0, 0.0))
    await asyncio.sleep(8)

    # 航点2: 向右（东）转弯
    print("-- 飞向航点2 (东100米)")
    await drone.offboard.set_position_ned(PositionNedYaw(100.0, 100.0, -25.0, 90.0))
    await asyncio.sleep(10)

    # 航点3: 向后（南）飞
    print("-- 飞向航点3 (返回)")
    await drone.offboard.set_position_ned(PositionNedYaw(50.0, 100.0, -25.0, 180.0))
    await asyncio.sleep(10)

    # 航点4: 返回起点附近
    print("-- 飞向航点4 (接近home)")
    await drone.offboard.set_position_ned(PositionNedYaw(20.0, 50.0, -22.0, 270.0))
    await asyncio.sleep(10)

    # =============== 阶段5: 反向过渡回多旋翼模式 ===============
    print("\n[阶段5] 请求切换回多旋翼模式...")
    try:
        await drone.action.transition_to_multicopter()
        print("-- 已发送多旋翼过渡命令")
    except Exception as e:
        print(f"过渡命令发送失败: {e}")

    # 监控过渡状态
    print("监控反向过渡...")
    transition_complete = False
    start_time = asyncio.get_event_loop().time()

    while not transition_complete:
        async for vtol_state in drone.telemetry.vtol_state():
            print(f"当前VTOL状态: {vtol_state}")
            if "MULTICOPTER" in str(vtol_state) or "MC" in str(vtol_state):
                transition_complete = True
                print("-- 已完成过渡到多旋翼模式")
                break
            if asyncio.get_event_loop().time() - start_time > timeout:
                print("-- 过渡超时，继续下一步")
                transition_complete = True
                break
            await asyncio.sleep(1)
            break

    # =============== 阶段6: 返回home点上方 ===============
    print("\n[阶段6] 返回home点上方...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -15.0, 0.0))
    await asyncio.sleep(10)
    print("-- 已到达home点上方")

    # =============== 阶段7: 停止Offboard并降落 ===============
    print("\n[阶段7] 停止Offboard模式并执行降落...")
    try:
        await drone.offboard.stop()
        print("-- Offboard模式已停止")
    except OffboardError as error:
        print(f"停止Offboard模式失败: {error}")

    # 执行降落
    print("执行降落...")
    await drone.action.land()

    # 等待降落完成
    print("等待降落完成...")
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("-- 降落完成")
            break

    # 等待一会儿确保完全着陆
    await asyncio.sleep(2)

    # 上锁
    print("\n任务完成，上锁无人机...")
    await drone.action.disarm()
    print("-- 无人机已上锁\n")
    print("=== Tailsitter VTOL 任务完成 ===")


async def print_status_text(drone):
    """打印状态文本"""
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"[状态] {status_text.text}")
    except asyncio.CancelledError:
        return


async def print_position(drone):
    """打印位置信息"""
    try:
        async for position in drone.telemetry.position():
            print(f"[位置] NED估计: N={position.latitude_deg:.6f}, E={position.longitude_deg:.6f}, "
                  f"高度={position.relative_altitude_m:.1f}m")
            await asyncio.sleep(5)
    except asyncio.CancelledError:
        return


if __name__ == "__main__":
    """程序入口"""
    try:
        # 运行主函数
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\n程序被用户中断")
        sys.exit(0)
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
