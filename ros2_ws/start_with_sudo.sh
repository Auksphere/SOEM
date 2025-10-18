#!/bin/bash

echo "=== ROS2 Force Sensor with sudo permissions ==="
echo ""

# 检查当前是否为root用户
if [ "$EUID" -ne 0 ]; then
    echo "This script needs to be run with sudo for EtherCAT access."
    echo "Usage: sudo $0"
    echo ""
    echo "Or run manually:"
    echo "sudo -E env \"PATH=\$PATH\" ros2 launch ros2_force_sensor force_sensor.launch.py"
    exit 1
fi

# 设置ROS2环境
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✅ ROS2 Jazzy environment loaded"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble environment loaded"
else
    echo "❌ ROS2 environment not found"
    exit 1
fi

# 设置本地包环境
cd /home/auksphere/robotics/SOEM/ros2_force_sensor
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Local package environment loaded"
else
    echo "❌ Local package not built. Please run 'colcon build' first"
    exit 1
fi

echo ""
echo "🚀 Starting Force Sensor Node with sudo permissions..."
echo "Interface: ${1:-enp8s0}"
echo "Cycle time: ${2:-8000} us"
echo ""

# 启动节点
ros2 launch ros2_force_sensor force_sensor.launch.py interface:=${1:-enp8s0} cycle_time_us:=${2:-8000}