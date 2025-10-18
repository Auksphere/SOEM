# Jaka Robot Controller

基于导纳控制 (Admittance Control) 的Jaka机械臂控制器，专为仅支持位置控制的机械臂设计，支持力传感器反馈。

## 功能特性

- **导纳控制 (Admittance Control)**：适用于位置控制机械臂的力控制方法
- **125Hz高频控制**：满足实时控制要求
- **力传感器集成**：支持ROS2力传感器数据反馈
- **多任务模式**：支持调节、圆形、直线、球面轨迹跟踪
- **安全保护**：包含关节限位和紧急停止功能
- **数据记录**：自动记录关节角度、力数据和轨迹跟踪信息

## 系统架构

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   ROS2 Force    │───▶│  Jaka Controller │───▶│  Jaka Robot     │
│   Sensor Node   │    │     Node         │    │   Hardware      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │
                              ▼
                       ┌─────────────────┐
                       │   Data Logging  │
                       │   (CSV Files)   │
                       └─────────────────┘
```

## 文件说明

### 主要文件

- `jaka_publisher.py`: 主控制器节点，实现导纳控制算法
- `test_admittance_control.py`: 导纳控制算法测试和仿真脚本
- `test_jaka_basic.py`: 基本功能测试脚本
- `start_jaka_controller.py`: 控制器启动脚本
- `jaka_config.ini`: 配置文件

### 依赖文件

- `python/__common.py`: Jaka API环境初始化
- `python/jkrc`: Jaka机械臂Python API
- `../misc_func.py`: GUFIC算法辅助函数
- `../filter.py`: 信号滤波器实现

## 安装和配置

## 导纳控制原理

### 为什么使用导纳控制？

JAKA机械臂仅支持位置控制模式，不支持直接的力矩控制和速度控制。因此，传统的阻抗控制（输出力矩命令）无法直接应用。导纳控制通过以下方式解决这个问题：

1. **阻抗控制 vs 导纳控制**：
   - 阻抗控制：输入位置/速度 → 输出力矩 (需要力矩控制接口)
   - 导纳控制：输入力 → 输出位置/速度 (适用于位置控制接口)

2. **控制策略**：
   ```
   测量力 → 力误差 → 导纳模型 → 位置修正 → 位置命令 → 机械臂
   ```

3. **导纳模型**：
   ```
   位置修正 = 导纳增益 × 力误差
   ```

### 算法流程

1. **轨迹生成**：计算期望的末端位置轨迹
2. **力测量**：获取力传感器反馈
3. **力误差计算**：测量力 - 期望力
4. **导纳计算**：根据力误差计算位置修正量
5. **逆运动学**：将修正后的末端位置转换为关节角度
6. **位置命令**：发送关节位置命令给机械臂

### 1. 系统要求

- Ubuntu 20.04 或更高版本
- ROS2 Humble 或更高版本
- Python 3.8+
- Jaka机械臂SDK

### 2. 依赖安装

```bash
# 安装Python依赖
pip install numpy scipy

# 安装ROS2依赖
sudo apt install ros-humble-sensor-msgs ros-humble-geometry-msgs

# 构建ROS2工作空间
cd ../../ros2_ws
colcon build
source install/setup.bash
```

### 3. Jaka SDK配置

1. 从Jaka官方获取机械臂SDK
2. 将SDK解压到 `python/` 目录下
3. 确保以下文件存在：
   - `python/out/shared/libjakaAPI.so`
   - `python/out/python3/jkrc.so`

### 4. 网络配置

1. 配置机械臂IP地址（默认：192.168.125.3）
2. 确保控制电脑与机械臂网络连通
3. 修改 `jaka_config.ini` 中的IP地址设置

## 使用方法

### 1. 基本功能测试

首先运行基本测试确保机械臂连接正常：

```bash
cd /home/auksphere/robotics/SOEM/gufic/control
python3 test_jaka_basic.py
```

测试包括：
- 机械臂连接测试
- 基本运动测试
- 125Hz高频控制测试

### 2. 导纳控制测试

运行导纳控制算法测试和仿真：

```bash
cd /home/auksphere/robotics/SOEM/gufic/control
python3 test_admittance_control.py
```

测试包括：
- 导纳参数影响测试
- 轨迹跟踪验证
- 力控制仿真
- 结果可视化

### 3. 启动力传感器

```bash
# 启动力传感器节点
cd ../../ros2_ws
ros2 launch launch/force_sensor.launch.py

# 或者直接运行节点
ros2 run ros2_force_sensor force_sensor_node
```

### 3. 运行控制器

#### 方法1：使用启动脚本（推荐）

```bash
python3 start_jaka_controller.py
```

这将自动启动力传感器节点和Jaka控制器。

#### 方法2：手动启动

```bash
# 终端1：启动力传感器
ros2 run ros2_force_sensor force_sensor_node

# 终端2：启动Jaka控制器
python3 jaka_publisher.py
```

### 4. 监控和调试

```bash
# 查看关节状态
ros2 topic echo /joint_states

# 查看末端力扭矩
ros2 topic echo /ee_wrench

# 查看力传感器原始数据
ros2 topic echo /force_sensor_data
```

## 配置参数

编辑 `jaka_config.ini` 文件进行参数调整：

### 机械臂连接
```ini
[robot]
ip_address = 192.168.125.3  # 机械臂IP地址
port = 10000                  # 连接端口
timeout = 5.0                 # 连接超时时间
```

### 控制参数
```ini
[control]
frequency = 125              # 控制频率 (Hz)
max_time = 10.0             # 最大运行时间 (s)
task_type = sphere          # 任务类型
```

### 力控制参数
```ini
[force_control]
desired_fz = 10.0           # 期望接触力 (N)
kp_force = 1.0              # 力控制比例增益
ki_force = 0.1              # 力控制积分增益
```

## 数据记录

控制器会自动记录数据到 `./log/` 目录：

- `joint_angles_real.csv`: 关节角度数据
- `contact_forces_real.csv`: 接触力数据
- `trajectory_tracking_real.csv`: 轨迹跟踪数据

## 安全注意事项

1. **紧急停止**：按Ctrl+C可立即停止控制器
2. **关节限位**：自动检查关节角度限制
3. **力限制**：当检测到过大力时自动停止
4. **网络监控**：监控与机械臂的通信状态

## 故障排除

### 常见问题

1. **无法连接机械臂**
   - 检查网络连接和IP地址
   - 确认机械臂处于就绪状态
   - 检查防火墙设置

2. **导入错误**
   - 确认Jaka SDK正确安装
   - 检查Python路径设置
   - 验证依赖包安装

3. **控制频率不稳定**
   - 检查系统负载
   - 调整控制器优先级
   - 减少其他程序占用

4. **力传感器数据异常**
   - 检查传感器连接
   - 验证ROS2节点状态
   - 确认话题发布正常

### 调试模式

启用详细日志输出：

```bash
export ROS_LOG_LEVEL=DEBUG
python3 jaka_publisher.py
```

## API参考

### JakaRobotState类

主要机械臂状态管理类：

```python
# 连接机械臂
robot_state = JakaRobotState("192.168.125.3")
robot_state.connect_robot()

# 更新状态
robot_state.update_robot_state()

# 获取位姿
pos, rot = robot_state.get_pose()

# 设置关节速度
robot_state.set_joint_velocities(velocities, dt)
```

### JakaControllerNode类

ROS2控制节点：

```python
# 创建控制节点
controller = JakaControllerNode()

# 执行控制循环
rclpy.spin(controller)
```

## 贡献和支持

- 报告问题：请在GitHub仓库中创建Issue
- 功能请求：欢迎提交Pull Request
- 文档改进：帮助完善使用说明

## 许可证

本项目遵循MIT许可证。详见LICENSE文件。

## 版本历史

- v1.0.0: 初始版本，支持基本GUFIC控制
- v1.1.0: 添加力传感器集成
- v1.2.0: 优化125Hz控制性能
- v1.3.0: 添加多任务模式支持

---

更多技术细节请参考源代码注释和相关论文。