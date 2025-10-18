#!/usr/bin/env python3
"""
简化的Jaka机械臂控制器测试版本
用于验证基本功能
"""

import time
import math
import numpy as np
import os
import sys

# Add necessary paths
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, 'python'))

# Import Jaka API
import __common

def test_jaka_connection():
    """测试Jaka机械臂连接"""
    try:
        __common.init_env()
        import jkrc
        
        # 连接机械臂
        robot_ip = "192.168.125.3"
        rc = jkrc.RC(robot_ip)
        
        print(f"Connecting to Jaka robot at {robot_ip}...")
        result = rc.login()
        
        if result == 0:
            print("✓ Successfully connected to robot")
            
            # 获取机器人状态
            status = rc.get_robot_status()
            print(f"Robot status: {status}")
            
            # 获取关节位置
            joint_pos = rc.get_joint_position()
            if joint_pos[0] == 0:
                print(f"Current joint positions: {joint_pos[1]}")
            
            # 获取TCP位置
            tcp_pos = rc.get_tcp_position()
            if tcp_pos[0] == 0:
                print(f"Current TCP position: {tcp_pos[1]}")
            
            # 断开连接
            rc.logout()
            print("✓ Robot disconnected successfully")
            return True
            
        else:
            print(f"✗ Failed to connect to robot. Error code: {result}")
            return False
            
    except Exception as e:
        print(f"✗ Exception during robot connection test: {e}")
        return False

def test_basic_movement():
    """测试基本运动功能"""
    try:
        __common.init_env()
        import jkrc
        
        robot_ip = "192.168.125.3"
        rc = jkrc.RC(robot_ip)
        
        print("Testing basic robot movement...")
        
        # 连接并启用机器人
        if rc.login() != 0:
            print("Failed to login")
            return False
            
        print("✓ Logged in successfully")
        
        # 上电
        power_result = rc.power_on()
        if power_result == 0:
            print("✓ Robot powered on")
        else:
            print(f"Power on failed: {power_result}")
            
        # 使能
        enable_result = rc.enable_robot()
        if enable_result == 0:
            print("✓ Robot enabled")
        else:
            print(f"Enable failed: {enable_result}")
            
        # 获取当前位置
        current_pos = rc.get_joint_position()
        if current_pos[0] == 0:
            print(f"Current position: {current_pos[1]}")
            
            # 小幅运动测试
            target_pos = current_pos[1].copy()
            target_pos[5] += 0.1  # 移动第6轴10度
            
            print(f"Moving to: {target_pos}")
            move_result = rc.joint_move(target_pos, 0, True, 0.5)  # 慢速移动
            
            if move_result == 0:
                print("✓ Movement command sent successfully")
                time.sleep(3)  # 等待运动完成
                
                # 检查新位置
                new_pos = rc.get_joint_position()
                if new_pos[0] == 0:
                    print(f"New position: {new_pos[1]}")
            else:
                print(f"Movement failed: {move_result}")
        
        # 回到原位置
        if current_pos[0] == 0:
            print("Returning to original position...")
            rc.joint_move(current_pos[1], 0, True, 0.5)
            time.sleep(3)
        
        # 断开连接
        rc.disable_robot()
        rc.power_off()
        rc.logout()
        print("✓ Test completed successfully")
        return True
        
    except Exception as e:
        print(f"✗ Exception during movement test: {e}")
        return False

def test_high_frequency_control():
    """测试125Hz高频控制"""
    try:
        __common.init_env()
        import jkrc
        
        robot_ip = "192.168.125.3"
        rc = jkrc.RC(robot_ip)
        
        print("Testing 125Hz control loop...")
        
        # 连接并启用机器人
        if rc.login() != 0:
            print("Failed to login")
            return False
            
        rc.power_on()
        rc.enable_robot()
        
        # 获取初始位置
        initial_pos = rc.get_joint_position()
        if initial_pos[0] != 0:
            print("Failed to get initial position")
            return False
            
        print(f"Initial position: {initial_pos[1]}")
        
        # 125Hz控制循环参数
        frequency = 125  # Hz
        dt = 1.0 / frequency
        duration = 2.0  # 运行2秒
        steps = int(duration / dt)
        
        print(f"Running {steps} steps at {frequency}Hz for {duration}s")
        
        # 正弦波运动参数
        amplitude = 0.1  # 10度幅值
        omega = 2 * math.pi * 0.5  # 0.5Hz频率
        
        start_time = time.time()
        
        for i in range(steps):
            loop_start = time.time()
            
            # 计算目标位置（正弦波）
            t = i * dt
            target_pos = initial_pos[1].copy()
            target_pos[5] = initial_pos[1][5] + amplitude * math.sin(omega * t)
            
            # 发送运动指令
            rc.joint_move(target_pos, 0, True, 2*math.pi)  # 高速模式
            
            # 控制频率
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
            
            # 每秒打印一次状态
            if i % frequency == 0:
                current_pos = rc.get_joint_position()
                if current_pos[0] == 0:
                    error = abs(current_pos[1][5] - target_pos[5])
                    print(f"Step {i}: Target={target_pos[5]:.4f}, Actual={current_pos[1][5]:.4f}, Error={error:.4f}")
        
        actual_duration = time.time() - start_time
        actual_frequency = steps / actual_duration
        
        print(f"✓ Control loop completed")
        print(f"Planned: {steps} steps in {duration}s ({frequency}Hz)")
        print(f"Actual: {steps} steps in {actual_duration:.3f}s ({actual_frequency:.1f}Hz)")
        
        # 回到初始位置
        rc.joint_move(initial_pos[1], 0, True, 0.5)
        time.sleep(2)
        
        # 断开连接
        rc.disable_robot()
        rc.power_off()
        rc.logout()
        
        return True
        
    except Exception as e:
        print(f"✗ Exception during high-frequency test: {e}")
        return False

def main():
    """主测试函数"""
    print("=== Jaka Robot Controller Test Suite ===\n")
    
    tests = [
        ("Connection Test", test_jaka_connection),
        ("Basic Movement Test", test_basic_movement),
        ("High Frequency Control Test", test_high_frequency_control),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        print(f"\n--- {test_name} ---")
        try:
            result = test_func()
            results.append((test_name, result))
            print(f"Result: {'PASS' if result else 'FAIL'}")
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
            break
        except Exception as e:
            print(f"Test failed with exception: {e}")
            results.append((test_name, False))
        
        print("-" * 50)
    
    # 打印总结
    print("\n=== Test Results Summary ===")
    passed = 0
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"{test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\nTotal: {passed}/{len(results)} tests passed")
    
    return passed == len(results)

if __name__ == '__main__':
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nTest suite interrupted by user")
        sys.exit(1)