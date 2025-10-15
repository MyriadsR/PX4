# Tailsitter VTOL 自主飞行任务指南

本指南将帮助你运行Tailsitter尾座垂起固定翼无人机的完整自主飞行任务，包括垂直起飞、过渡到固定翼模式、固定翼飞行、过渡回多旋翼模式和降落。

## 目录结构

```
PX4_project/
├── PX4-Autopilot/           # PX4主代码库
├── tailsitter_waypoint_mission.py      # 航点任务脚本（推荐）
├── tailsitter_offboard_control.py      # Offboard手动控制脚本
├── TAILSITTER_MISSION_GUIDE.md         # 本文档
└── requirements.txt         # Python依赖（需创建）
```

## 前置条件

### 1. 安装MAVSDK-Python

```bash
# 安装MAVSDK Python库
pip3 install mavsdk

# 或使用清华镜像加速
pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple mavsdk
```

### 2. 验证PX4安装

确保你已经成功编译了PX4并能够运行Gazebo仿真：

```bash
cd PX4-Autopilot
make px4_sitl gz_quadtailsitter
```

如果能看到Gazebo中出现Tailsitter无人机，说明环境配置正确。

## 飞行任务说明

### 任务流程

1. **垂直起飞** - 无人机以多旋翼模式垂直起飞到20米高度
2. **前向过渡** - 开始加速前飞，俯仰角逐渐降低，过渡到固定翼模式
3. **固定翼巡航** - 以固定翼模式飞行矩形航线（约200m x 200m）
4. **反向过渡** - 减速并俯仰角抬升，过渡回多旋翼模式
5. **悬停定位** - 以多旋翼模式悬停在降落点上方
6. **垂直降落** - 垂直下降并着陆

### VTOL模式转换原理

#### 前向过渡（MC → FW）
- **触发条件**：命令切换到固定翼模式或速度达到阈值
- **过渡过程**：
  - 俯仰角从垂直（~90°）逐渐降低到固定翼角度（~-60°）
  - 多旋翼推力逐渐转换为前向推力
  - 空速逐渐增加到巡航速度（默认15 m/s）
- **完成条件**：俯仰角 ≤ -60° 且空速达标
- **参数**：
  - `VT_F_TRANS_DUR`: 前向过渡持续时间（1.5秒）
  - `VT_ARSP_TRANS`: 过渡空速阈值（15 m/s）

#### 反向过渡（FW → MC）
- **触发条件**：命令切换到多旋翼模式
- **过渡过程**：
  - 俯仰角从固定翼角度逐渐抬升到垂直
  - 前向推力转换为垂直推力
  - 空速逐渐降低
- **完成条件**：俯仰角 ≥ -15°
- **参数**：
  - `VT_B_TRANS_DUR`: 反向过渡持续时间（5秒）

## 运行方法

### 方法1: 使用航点任务脚本（推荐）

这个方法使用PX4的Mission模式，更加稳定可靠。

#### 步骤1: 启动PX4 SITL和Gazebo

在终端1中运行：

```bash
cd PX4-Autopilot
make px4_sitl gz_quadtailsitter
```

等待Gazebo启动并看到Tailsitter模型出现。

#### 步骤2: 运行任务脚本

在终端2中运行：

```bash
cd /home/zr/PX4_ws/PX4_project
python3 tailsitter_waypoint_mission.py
```

你将看到如下输出：

```
等待无人机连接...
-- 已连接到无人机!
等待全局位置估计...
-- 全局位置估计就绪
-- Home位置: Lat=47.397742, Lon=8.545594, Alt=488.0m

创建Tailsitter VTOL任务航点...
上传任务到无人机...
-- 任务上传完成

解锁无人机...
-- 无人机已解锁

开始执行任务...
-- 任务已开始

监控任务执行...
任务进度: 1/7
VTOL状态: VtolState.TRANSITION_TO_FW
位置: Lat=47.397742, Lon=8.545594, 相对高度=20.5m, 绝对高度=508.5m
...
```

### 方法2: 使用Offboard控制脚本

这个方法直接通过Offboard模式控制，更加灵活但需要持续发送命令。

```bash
# 终端1: 启动仿真
cd PX4-Autopilot
make px4_sitl gz_quadtailsitter

# 终端2: 运行控制脚本
cd /home/zr/PX4_ws/PX4_project
python3 tailsitter_offboard_control.py
```

## Gazebo可视化

在Gazebo中你可以观察到：

1. **起飞阶段**：无人机保持垂直姿态，螺旋桨向下推，垂直上升
2. **前向过渡**：无人机开始前倾，从垂直姿态逐渐过渡到水平飞行姿态
3. **固定翼飞行**：无人机以水平姿态高速飞行，类似飞机
4. **反向过渡**：无人机减速并抬头，从水平姿态恢复到垂直姿态
5. **降落**：保持垂直姿态下降

### Gazebo视角控制

- **旋转视角**：按住鼠标中键拖动
- **平移视角**：按住Shift + 鼠标中键拖动
- **缩放**：滚动鼠标滚轮
- **跟随无人机**：右键点击无人机 → "Follow"

## 关键参数说明

### VTOL过渡参数

在 `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4018_gz_quadtailsitter` 中配置：

```bash
# 前向过渡参数
param set-default VT_F_TRANS_DUR 1.5   # 前向过渡持续时间（秒）
param set-default VT_ARSP_TRANS 15     # 过渡空速阈值（m/s）

# 反向过渡参数
param set-default VT_B_TRANS_DUR 5     # 反向过渡持续时间（秒）

# VTOL类型
param set-default VT_TYPE 0            # 0=Tailsitter

# 差分推力（用于固定翼模式的姿态控制）
param set-default VT_FW_DIFTHR_EN 7    # 启用所有轴的差分推力
param set-default VT_FW_DIFTHR_S_Y 1   # 偏航差分推力比例
```

### 飞行控制参数

```bash
# 固定翼参数
param set-default FW_AIRSPD_MIN 14     # 最小空速（m/s）
param set-default FW_AIRSPD_TRIM 18    # 巡航空速（m/s）
param set-default FW_AIRSPD_MAX 22     # 最大空速（m/s）
param set-default FW_PSP_OFF 2         # 俯仰设定偏移（度）

# 多旋翼参数
param set-default MC_ROLL_P 3          # 滚转比例增益
param set-default MC_PITCH_P 3         # 俯仰比例增益
```

## 调试和问题排查

### 查看日志

PX4的日志文件保存在：
```
PX4-Autopilot/build/px4_sitl_default/logs/
```

可以使用FlightPlot或在线工具查看：https://logs.px4.io

### 常见问题

#### 1. 连接失败

**问题**：脚本显示"等待无人机连接..."但一直无法连接

**解决**：
- 确保PX4 SITL正在运行
- 检查端口14540是否被占用：`netstat -an | grep 14540`
- 确保没有其他地面站软件（QGC）占用连接

#### 2. 无人机不起飞

**问题**：解锁后无人机不起飞或立即上锁

**解决**：
- 检查EKF状态：在PX4控制台输入 `commander status`
- 确保GPS/位置估计正常：`listener vehicle_local_position`
- 查看failsafe状态

#### 3. 过渡失败

**问题**：无人机无法从多旋翼过渡到固定翼

**解决**：
- 检查空速是否达到阈值（15 m/s）
- 增加前飞速度或降低 `VT_ARSP_TRANS` 参数
- 在PX4控制台查看过渡状态：`listener vtol_vehicle_status`

#### 4. 固定翼模式飞行不稳定

**问题**：固定翼模式下姿态抖动或失控

**解决**：
- 调整固定翼PID参数（FW_RR_P, FW_PR_P等）
- 检查差分推力设置
- 增加巡航空速

## 修改任务航点

编辑 `tailsitter_waypoint_mission.py` 中的航点定义：

```python
# 航点格式
mission_items.append(MissionItem(
    latitude,           # 纬度（度）
    longitude,          # 经度（度）
    altitude,           # 相对高度（米）
    speed,              # 飞行速度（m/s）
    is_fly_through,     # False=悬停, True=飞过
    float('nan'),       # 接受半径（默认）
    float('nan'),       # 通过半径（默认）
    float('nan'),       # 盘旋方向（默认）
    float('nan'),       # 俯仰角（默认）
    float('nan'),       # 航向（默认）
    MissionItem.CameraAction.NONE,  # 相机动作
    loiter_time,        # 悬停时间（秒）
    float('nan'),       # 相机间隔
    float('nan'),       # 相机距离
    float('nan'),       # 悬停速度
))
```

### 坐标转换

大约的转换关系（在赤道附近）：
- 纬度增加 0.00001 ≈ 向北 1.11 米
- 经度增加 0.00001 ≈ 向东 1.11 米（随纬度变化）

## 进阶：自定义VTOL参数

如果要调整过渡行为，可以修改参数文件或在运行时通过MAVLink设置：

```python
# 在脚本中设置参数示例
await drone.param.set_param_float("VT_F_TRANS_DUR", 2.0)  # 延长前向过渡时间
await drone.param.set_param_float("VT_ARSP_TRANS", 12.0)  # 降低过渡空速阈值
```

## 参考资料

- PX4用户指南 - VTOL: https://docs.px4.io/main/en/frames_vtol/
- MAVSDK Python文档: https://mavsdk-python-docs.s3.amazonaws.com/
- PX4开发指南: https://docs.px4.io/main/en/development/
- Tailsitter原理: https://docs.px4.io/main/en/frames_vtol/vtol_tailsitter.html

## 联系和支持

如有问题，可以：
1. 查看PX4文档
2. 在PX4 Discuss论坛提问：https://discuss.px4.io/
3. 查看PX4 GitHub Issues

---

**祝飞行愉快！** 🚁✈️
