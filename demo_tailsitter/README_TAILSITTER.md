# Tailsitter VTOL 自主飞行项目

这是一个基于PX4的Tailsitter尾座垂起固定翼无人机自主飞行控制项目，实现了完整的垂直起飞、模式转换、固定翼飞行和垂直降落流程。

## 🎯 项目目标

实现Tailsitter VTOL无人机的完整自主飞行任务：
1. ✅ 垂直起飞（多旋翼模式）
2. ✅ 前向过渡（多旋翼 → 固定翼）
3. ✅ 固定翼巡航飞行
4. ✅ 反向过渡（固定翼 → 多旋翼）
5. ✅ 垂直降落（多旋翼模式）

全程在Gazebo仿真环境中可视化展示。

## 📁 项目文件

```
PX4_project/
├── PX4-Autopilot/                          # PX4固件源码
├── tailsitter_waypoint_mission.py          # 航点任务脚本（推荐使用）
├── tailsitter_offboard_control.py          # Offboard手动控制脚本
├── run_tailsitter_mission.sh               # 快速启动脚本
├── requirements.txt                        # Python依赖
├── TAILSITTER_MISSION_GUIDE.md            # 详细使用指南
├── CLAUDE.md                              # PX4代码库指南
└── README_TAILSITTER.md                   # 本文件
```

## 🚀 快速开始

### 方法1: 使用快速启动脚本（最简单）

```bash
cd /home/zr/PX4_ws/PX4_project
./run_tailsitter_mission.sh
```

按照提示选择运行模式即可。

### 方法2: 手动启动（推荐学习）

#### 步骤1: 安装依赖

```bash
pip3 install mavsdk
```

#### 步骤2: 启动PX4 SITL + Gazebo

在终端1中：
```bash
cd PX4-Autopilot
make px4_sitl gz_quadtailsitter
```

等待Gazebo完全启动（约10-15秒）。

#### 步骤3: 运行任务脚本

在终端2中：
```bash
cd /home/zr/PX4_ws/PX4_project
python3 tailsitter_offboard_control.py
```

## 📊 任务流程说明

### 1. 垂直起飞阶段
- **模式**: 多旋翼（MC）
- **高度**: 0m → 20m
- **姿态**: 垂直（俯仰角 ~90°）
- **持续时间**: 约10秒

### 2. 前向过渡阶段
- **模式**: TRANSITION_TO_FW
- **过程**:
  - 俯仰角从90°降低到-60°
  - 多旋翼推力转为前向推力
  - 加速到15 m/s以上
- **持续时间**: 约1.5秒（参数VT_F_TRANS_DUR）
- **完成条件**: 俯仰角≤-60° 且空速≥15m/s

### 3. 固定翼巡航阶段
- **模式**: 固定翼（FW）
- **飞行路径**: 矩形航线（约200m × 200m）
- **巡航速度**: 18 m/s
- **高度**: 25-30m
- **持续时间**: 约40-50秒

### 4. 反向过渡阶段
- **模式**: TRANSITION_TO_MC
- **过程**:
  - 减速到低速
  - 俯仰角从-60°抬升到-15°
  - 前向推力转为垂直推力
- **持续时间**: 约5秒（参数VT_B_TRANS_DUR）
- **完成条件**: 俯仰角≥-15°

### 5. 垂直降落阶段
- **模式**: 多旋翼（MC）
- **过程**: 悬停 → 缓慢下降 → 着陆
- **持续时间**: 约15-20秒

## 🎮 控制原理

### VTOL模式状态机

```
     起飞          前向过渡        巡航          反向过渡       降落
      ↓               ↓             ↓              ↓            ↓
   [MC_MODE] → [TRANSITION_TO_FW] → [FW_MODE] → [TRANSITION_TO_MC] → [MC_MODE]
      ↑                                                                   ↓
      └───────────────────────────  降落  ←──────────────────────────────┘
```

### 关键代码文件

1. **VTOL控制**: `PX4-Autopilot/src/modules/vtol_att_control/`
   - `tailsitter.cpp`: Tailsitter特定控制逻辑
   - `tailsitter.h`: Tailsitter类定义
   - `vtol_type.cpp`: VTOL基类实现

2. **参数配置**: `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4018_gz_quadtailsitter`
   - VTOL过渡参数
   - 固定翼/多旋翼控制参数
   - 执行器配置

3. **姿态设定点**（你选中的代码）: `tailsitter.cpp:413`
   ```cpp
   _v_att_sp  // 姿态设定点结构体
   ```
   这个变量存储了期望的姿态（四元数）和推力，在过渡过程中被连续更新。

## 🔧 关键参数

### VTOL过渡参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `VT_TYPE` | 0 | VTOL类型（0=Tailsitter） |
| `VT_F_TRANS_DUR` | 1.5s | 前向过渡持续时间 |
| `VT_B_TRANS_DUR` | 5s | 反向过渡持续时间 |
| `VT_ARSP_TRANS` | 15 m/s | 过渡空速阈值 |
| `VT_FW_DIFTHR_EN` | 7 | 启用差分推力 |

### 固定翼参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `FW_AIRSPD_MIN` | 14 m/s | 最小空速 |
| `FW_AIRSPD_TRIM` | 18 m/s | 巡航空速 |
| `FW_AIRSPD_MAX` | 22 m/s | 最大空速 |
| `FW_PSP_OFF` | 2° | 俯仰设定偏移 |

### 多旋翼参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `MC_ROLL_P` | 3.0 | 滚转P增益 |
| `MC_PITCH_P` | 3.0 | 俯仰P增益 |
| `MC_AIRMODE` | 2 | 空中模式（允许翻转） |

## 📈 监控和调试

### 实时监控

脚本会实时打印：
- 任务进度（当前航点/总航点数）
- VTOL状态（MC/FW/TRANSITION）
- 位置信息（纬度、经度、高度）

### 查看日志

```bash
# PX4日志位置
ls PX4-Autopilot/build/px4_sitl_default/logs/

# 上传到在线分析工具
# 访问 https://logs.px4.io 并上传.ulg文件
```

### 在线查看消息

在PX4控制台（终端1）输入：
```bash
# 查看VTOL状态
listener vtol_vehicle_status

# 查看姿态
listener vehicle_attitude

# 查看位置
listener vehicle_local_position
```

## 🎥 Gazebo可视化技巧

### 相机控制
- **旋转**: 鼠标中键拖动
- **平移**: Shift + 鼠标中键
- **缩放**: 滚轮
- **跟随**: 右键无人机 → "Follow"

### 查看信息
- 左侧面板可查看模型属性
- 右上角显示仿真时间
- 底部显示实时FPS

## 🐛 常见问题

### Q1: 无人机不起飞
**A**: 检查GPS/EKF状态，确保位置估计正常。在PX4控制台输入 `commander status`

### Q2: 过渡失败或不完整
**A**: 可能是速度不够。尝试：
- 增加前飞距离
- 降低 `VT_ARSP_TRANS` 参数
- 延长 `VT_F_TRANS_DUR` 时间

### Q3: 固定翼模式飞行不稳定
**A**: 调整PID参数或增加巡航速度

### Q4: 脚本连接失败
**A**: 确保PX4 SITL正在运行且端口14540未被占用

## 📚 扩展阅读

- [PX4 VTOL官方文档](https://docs.px4.io/main/en/frames_vtol/)
- [Tailsitter设计原理](https://docs.px4.io/main/en/frames_vtol/vtol_tailsitter.html)
- [MAVSDK Python API](https://mavsdk-python-docs.s3.amazonaws.com/)
- [详细使用指南](TAILSITTER_MISSION_GUIDE.md)

## 🎓 学习要点

1. **VTOL状态机**: 理解不同模式间的切换逻辑
2. **姿态控制**: 过渡过程中姿态如何平滑变化
3. **推力混合**: 多旋翼推力和固定翼推力的混合策略
4. **MAVLink通信**: 如何通过MAVLink控制无人机

## 📝 下一步

- [ ] 调整参数优化过渡性能
- [ ] 添加更复杂的航点任务
- [ ] 尝试不同的飞行速度和高度
- [ ] 实现自定义的过渡触发逻辑
- [ ] 在真机上测试（需要实际硬件）

## 💡 贡献

欢迎提交Issue和Pull Request！

---

**作者**: Claude Code
**日期**: 2025-10-15
**PX4版本**: v1.14+
**许可**: 遵循PX4的BSD许可
