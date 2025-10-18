#!/usr/bin/env python3
"""
Jaka机械臂控制器启动脚本
用于启动真实机械臂控制，包括力传感器反馈
"""

import os
import sys
import subprocess
import signal
import time

def setup_environment():
    """设置运行环境"""
    # 添加必要的路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(current_dir)
    
    # 添加GUFIC路径
    if parent_dir not in sys.path:
        sys.path.append(parent_dir)
    
    # 添加ROS2工作空间路径
    ros2_ws_path = os.path.join(os.path.dirname(parent_dir), 'ros2_ws')
    if os.path.exists(ros2_ws_path):
        # Source ROS2 workspace
        setup_script = os.path.join(ros2_ws_path, 'install', 'setup.bash')
        if os.path.exists(setup_script):
            print(f"Found ROS2 workspace at: {ros2_ws_path}")
        else:
            print("Warning: ROS2 workspace not built. Please build first.")

def check_dependencies():
    """检查依赖项"""
    print("Checking dependencies...")
    
    # Check if Jaka API is available
    jaka_lib_path = os.path.join(os.path.dirname(__file__), 'python', 'out', 'shared', 'libjakaAPI.so')
    if not os.path.exists(jaka_lib_path):
        print(f"Warning: Jaka API library not found at {jaka_lib_path}")
        print("Please ensure Jaka SDK is properly installed.")
    
    # Check Python packages
    required_packages = ['numpy', 'scipy', 'rclpy']
    missing_packages = []
    
    for package in required_packages:
        try:
            __import__(package)
        except ImportError:
            missing_packages.append(package)
    
    if missing_packages:
        print(f"Missing Python packages: {missing_packages}")
        print("Please install with: pip install " + " ".join(missing_packages))
        return False
    
    print("Dependencies check passed.")
    return True

def start_force_sensor():
    """启动力传感器节点"""
    print("Starting force sensor node...")
    
    # 构建力传感器启动命令
    ros2_ws_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'ros2_ws')
    
    # 使用启动文件
    launch_file = os.path.join(ros2_ws_path, 'launch', 'force_sensor.launch.py')
    
    if os.path.exists(launch_file):
        cmd = ['ros2', 'launch', launch_file]
    else:
        # 直接启动节点
        cmd = ['ros2', 'run', 'ros2_force_sensor', 'force_sensor_node']
    
    try:
        process = subprocess.Popen(cmd, 
                                 stdout=subprocess.PIPE, 
                                 stderr=subprocess.PIPE,
                                 preexec_fn=os.setsid)
        print(f"Force sensor process started with PID: {process.pid}")
        return process
    except Exception as e:
        print(f"Failed to start force sensor: {e}")
        return None

def start_jaka_controller():
    """启动Jaka控制器"""
    print("Starting Jaka controller...")
    
    controller_script = os.path.join(os.path.dirname(__file__), 'jaka_publisher.py')
    
    try:
        process = subprocess.Popen(['python3', controller_script],
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE,
                                 preexec_fn=os.setsid)
        print(f"Jaka controller process started with PID: {process.pid}")
        return process
    except Exception as e:
        print(f"Failed to start Jaka controller: {e}")
        return None

def signal_handler(sig, frame):
    """信号处理函数"""
    print("\nReceived interrupt signal. Shutting down...")
    
    # 终止所有子进程
    try:
        for process in active_processes:
            if process and process.poll() is None:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
    except Exception as e:
        print(f"Error during shutdown: {e}")
    
    sys.exit(0)

# 全局变量存储活跃进程
active_processes = []

def main():
    """主函数"""
    print("=== Jaka Robot Controller Launcher ===")
    
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 设置环境
    setup_environment()
    
    # 检查依赖
    if not check_dependencies():
        return 1
    
    # 启动力传感器节点
    force_sensor_process = start_force_sensor()
    if force_sensor_process:
        active_processes.append(force_sensor_process)
        time.sleep(2)  # 等待力传感器启动
    else:
        print("Warning: Force sensor node not started. Controller may not work properly.")
    
    # 启动Jaka控制器
    controller_process = start_jaka_controller()
    if controller_process:
        active_processes.append(controller_process)
    else:
        print("Failed to start Jaka controller.")
        return 1
    
    print("All nodes started successfully.")
    print("Press Ctrl+C to stop all nodes.")
    
    try:
        # 监控进程
        while True:
            time.sleep(1)
            
            # 检查进程是否还在运行
            for i, process in enumerate(active_processes):
                if process and process.poll() is not None:
                    print(f"Process {i} has terminated.")
                    stdout, stderr = process.communicate()
                    if stdout:
                        print(f"Process {i} stdout:", stdout.decode())
                    if stderr:
                        print(f"Process {i} stderr:", stderr.decode())
    
    except KeyboardInterrupt:
        signal_handler(signal.SIGINT, None)
    
    return 0

if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)