#!/bin/bash
# Tailsitter VTOL 快速启动脚本

echo "================================"
echo "Tailsitter VTOL 任务启动脚本"
echo "================================"
echo ""

# 设置颜色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查PX4目录
if [ ! -d "PX4-Autopilot" ]; then
    echo -e "${RED}错误: 找不到PX4-Autopilot目录${NC}"
    echo "请确保在正确的目录下运行此脚本"
    exit 1
fi

# 检查Python脚本
if [ ! -f "tailsitter_waypoint_mission.py" ]; then
    echo -e "${RED}错误: 找不到 tailsitter_waypoint_mission.py${NC}"
    exit 1
fi

# 检查MAVSDK是否安装
echo "检查依赖..."
python3 -c "import mavsdk" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}MAVSDK未安装，正在安装...${NC}"
    pip3 install mavsdk
    if [ $? -ne 0 ]; then
        echo -e "${RED}安装失败，请手动安装: pip3 install mavsdk${NC}"
        exit 1
    fi
fi
echo -e "${GREEN}✓ 依赖检查完成${NC}"
echo ""

# 提示用户选择
echo "请选择运行模式:"
echo "1) 航点任务模式 (推荐)"
echo "2) Offboard控制模式"
echo "3) 仅启动仿真（不运行任务）"
read -p "输入选项 (1/2/3): " choice

case $choice in
    1)
        SCRIPT="tailsitter_waypoint_mission.py"
        MODE="航点任务"
        ;;
    2)
        SCRIPT="tailsitter_offboard_control.py"
        MODE="Offboard控制"
        ;;
    3)
        SCRIPT=""
        MODE="仅仿真"
        ;;
    *)
        echo -e "${RED}无效选项${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}启动模式: $MODE${NC}"
echo ""

# 启动PX4 SITL和Gazebo
echo "=========================================="
echo "步骤 1: 启动 PX4 SITL 和 Gazebo"
echo "=========================================="
echo ""
echo -e "${YELLOW}注意: Gazebo窗口将在新终端中打开${NC}"
echo -e "${YELLOW}等待Gazebo完全启动（约10-15秒）...${NC}"
echo ""

cd PX4-Autopilot

# 使用gnome-terminal启动（如果可用）
if command -v gnome-terminal &> /dev/null; then
    gnome-terminal -- bash -c "make px4_sitl gz_quadtailsitter; exec bash" &
elif command -v xterm &> /dev/null; then
    xterm -e "make px4_sitl gz_quadtailsitter; bash" &
else
    # 后台启动
    make px4_sitl gz_quadtailsitter &
fi

PX4_PID=$!
cd ..

# 等待PX4启动
echo "等待PX4启动..."
sleep 15

# 检查PX4是否正在运行
if ! ps -p $PX4_PID > /dev/null 2>&1; then
    # 尝试查找px4进程
    if pgrep -x "px4" > /dev/null; then
        echo -e "${GREEN}✓ PX4已启动${NC}"
    else
        echo -e "${RED}错误: PX4未能启动${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✓ PX4已启动${NC}"
fi

# 如果选择了任务模式，运行Python脚本
if [ ! -z "$SCRIPT" ]; then
    echo ""
    echo "=========================================="
    echo "步骤 2: 运行任务脚本"
    echo "=========================================="
    echo ""
    sleep 5  # 额外等待以确保MAVLink连接就绪

    echo -e "${GREEN}执行 $SCRIPT...${NC}"
    echo ""
    python3 "$SCRIPT"

    if [ $? -eq 0 ]; then
        echo ""
        echo -e "${GREEN}=========================================="
        echo "任务执行完成！"
        echo -e "==========================================${NC}"
    else
        echo ""
        echo -e "${RED}任务执行失败${NC}"
    fi
else
    echo ""
    echo -e "${GREEN}=========================================="
    echo "仿真已启动"
    echo "你可以在Gazebo中观察无人机"
    echo "或手动运行任务脚本"
    echo -e "==========================================${NC}"
    echo ""
    echo "按Ctrl+C退出..."
    wait
fi

echo ""
echo "完成！"
