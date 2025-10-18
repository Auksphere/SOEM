# 六维力传感器数据消息
# 包含力和力矩的完整数据

# 消息头，包含时间戳和坐标系信息
std_msgs/Header header

# 数据序列号
uint16 data_no

# 力数据 (单位: N)
geometry_msgs/Vector3 force
    # force.x = Fx
    # force.y = Fy  
    # force.z = Fz

# 力矩数据 (单位: Nm)
geometry_msgs/Vector3 torque
    # torque.x = Mx
    # torque.y = My
    # torque.z = Mz

# 通信质量信息
uint32 roundtrip_time_us  # 往返时间（微秒）
uint32 wkc               # 工作计数器
bool communication_ok    # 通信状态