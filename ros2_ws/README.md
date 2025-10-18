# ROS2 Force Sensor Package

这个包将六维力传感器的EtherCAT数据发布为ROS2话题。

## 功能特性

- 通过EtherCAT接口读取六维力传感器数据
- 将数据发布为自定义ROS2消息类型
- 支持可配置的发布频率
- 包含力和力矩的偏移校准
- 提供通信质量监控

## 依赖

- ROS2 (测试环境: Humble/Iron)
- SOEM EtherCAT库
- C++17 编译器

## 安装和编译

1. 确保已经编译了SOEM库：
```bash
cd /home/auksphere/robotics/SOEM
mkdir -p build && cd build
cmake ..
make
```

2. 在ROS2工作空间中编译此包：
```bash
# 确保在ROS2工作空间的src目录中
cd ~/ros2_ws/src
ln -s /home/auksphere/robotics/SOEM/ros2_force_sensor .

# 回到工作空间根目录并编译
cd ~/ros2_ws
colcon build --packages-select ros2_force_sensor
```

3. 设置环境：
```bash
source ~/ros2_ws/install/setup.bash
```

## 使用方法

### 基本启动

```bash
# 使用默认参数启动
ros2 launch ros2_force_sensor force_sensor.launch.py

# 指定网络接口
ros2 launch ros2_force_sensor force_sensor.launch.py interface:=enp2s0

# 指定发布频率
ros2 launch ros2_force_sensor force_sensor.launch.py frequency:=200.0
```

### 直接运行节点

```bash
ros2 run ros2_force_sensor force_sensor_node
```

### 查看发布的数据

```bash
# 查看话题列表
ros2 topic list

# 查看力传感器数据
ros2 topic echo /force_sensor_data

# 查看消息频率
ros2 topic hz /force_sensor_data

# 查看消息结构
ros2 interface show ros2_force_sensor/msg/ForceSensorData
```

## 消息格式

自定义消息 `ForceSensorData` 包含：

```
std_msgs/Header header          # 时间戳和坐标系
uint16 data_no                  # 数据序列号
geometry_msgs/Vector3 force     # 力数据 (Fx, Fy, Fz) [N]
geometry_msgs/Vector3 torque    # 力矩数据 (Mx, My, Mz) [Nm] 
uint32 roundtrip_time_us        # 往返时间 [微秒]
uint32 wkc                      # 工作计数器
bool communication_ok           # 通信状态
```

## 参数配置

在 `config/force_sensor_params.yaml` 中可以配置：

- `interface`: EtherCAT网络接口名 (默认: "eth0")
- `cycle_time_us`: 循环时间，控制频率 (默认: 8000us = 125Hz)
- `fx_offset`, `fy_offset`, `fz_offset`: 力的偏移校准值
- `frame_id`: 坐标系ID (默认: "force_sensor")

## 频率设置

常用的频率设置：

- 1000 Hz: `cycle_time_us: 1000`
- 500 Hz:  `cycle_time_us: 2000`
- 200 Hz:  `cycle_time_us: 5000`
- 125 Hz:  `cycle_time_us: 8000`
- 100 Hz:  `cycle_time_us: 10000`

## 故障排除

1. **无法找到EtherCAT设备**
   - 检查网络接口名是否正确
   - 确保以root权限运行或设置了正确的网络权限

2. **编译错误**
   - 确保SOEM库已正确编译
   - 检查CMakeLists.txt中的路径设置

3. **通信质量问题**
   - 监控 `communication_ok` 字段
   - 检查 `roundtrip_time_us` 和 `wkc` 值

## 可视化

可以使用ROS2工具来可视化数据：

```bash
# 使用rqt_plot绘制力数据
ros2 run rqt_plot rqt_plot /force_sensor_data/force/x /force_sensor_data/force/y /force_sensor_data/force/z

# 使用rviz2显示力向量
rviz2
```

## 许可证

Apache-2.0 License