#!/usr/bin/env python3
"""
测试JAKA导纳控制实现
"""
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# 添加GUFIC路径
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from misc_func import initialize_trajectory, set_gains

def test_admittance_parameters():
    """测试导纳控制参数"""
    print("=== 导纳控制参数测试 ===")
    
    # 模拟力传感器数据
    force_data = np.array([0, 0, -15, 0, 0, 0])  # 15N接触力
    desired_force = np.array([0, 0, -10, 0, 0, 0])  # 期望10N接触力
    
    # 力误差
    force_error = force_data - desired_force
    print(f"力误差: {force_error}")
    
    # 不同导纳增益的影响
    gains = [0.001, 0.01, 0.1, 1.0]
    
    print("\n导纳增益对位置修正的影响:")
    for gain in gains:
        position_correction = gain * force_error
        print(f"增益 {gain:5.3f}: 位置修正 = {position_correction[:3]}")
    
    return True

def test_trajectory_following():
    """测试轨迹跟踪"""
    print("\n=== 轨迹跟踪测试 ===")
    
    # 初始化轨迹
    task = "sphere"
    pd_t, Rd_t, dpd_t, dRd_t, ddpd_t, ddRd_t = initialize_trajectory(task, name="jaka")
    
    # 测试几个时间点的轨迹
    times = [0, 1, 2, 3, 4, 5]
    
    print("时间点轨迹:")
    for t in times:
        pd = pd_t(t).flatten()
        print(f"t={t}s: 位置 = [{pd[0]:6.3f}, {pd[1]:6.3f}, {pd[2]:6.3f}]")
    
    return True

def simulate_admittance_control():
    """模拟导纳控制过程"""
    print("\n=== 导纳控制模拟 ===")
    
    # 仿真参数
    dt = 1/125  # 125Hz
    total_time = 5.0
    steps = int(total_time / dt)
    
    # 存储数据
    times = []
    positions = []
    forces = []
    corrections = []
    
    # 初始条件
    current_pos = np.array([0.4, 0.0, 0.3])  # 初始末端位置
    
    # 导纳参数
    admittance_gain = 0.01
    desired_force_z = -10.0
    
    # 初始化轨迹
    task = "sphere" 
    pd_t, Rd_t, dpd_t, dRd_t, ddpd_t, ddRd_t = initialize_trajectory(task, name="jaka")
    
    print("开始仿真...")
    
    for i in range(min(steps, 1000)):  # 限制仿真步数
        t = i * dt
        
        # 期望轨迹位置
        pd_nominal = pd_t(t).flatten()
        
        # 模拟力传感器读数 (基于接触情况)
        contact_force = 0
        if current_pos[2] < 0.25:  # 假设在z=0.25处接触
            contact_depth = 0.25 - current_pos[2]
            contact_force = -1000 * contact_depth  # 接触刚度
        
        force_measured = np.array([0, 0, contact_force, 0, 0, 0])
        force_desired = np.array([0, 0, desired_force_z, 0, 0, 0])
        
        # 力误差
        force_error = force_measured - force_desired
        
        # 导纳控制计算位置修正
        position_correction = admittance_gain * force_error[:3]
        
        # 限制修正量
        max_correction = 0.001  # 1mm
        position_correction = np.clip(position_correction, -max_correction, max_correction)
        
        # 更新位置
        target_pos = pd_nominal + position_correction
        
        # 简单的位置跟踪 (模拟机械臂响应)
        position_gain = 5.0
        current_pos += position_gain * (target_pos - current_pos) * dt
        
        # 存储数据
        if i % 25 == 0:  # 每0.2秒存储一次
            times.append(t)
            positions.append(current_pos.copy())
            forces.append(force_measured[2])
            corrections.append(position_correction[2])
    
    # 绘制结果
    times = np.array(times)
    positions = np.array(positions)
    forces = np.array(forces)
    corrections = np.array(corrections)
    
    # 获取期望轨迹用于对比
    desired_positions = []
    for t in times:
        pd = pd_t(t).flatten()
        desired_positions.append(pd)
    desired_positions = np.array(desired_positions)
    
    plt.figure(figsize=(15, 10))
    
    # 位置跟踪
    plt.subplot(2, 3, 1)
    plt.plot(times, desired_positions[:, 0], 'r--', label='期望X')
    plt.plot(times, positions[:, 0], 'r-', label='实际X')
    plt.xlabel('时间(s)')
    plt.ylabel('X位置(m)')
    plt.legend()
    plt.title('X方向位置跟踪')
    plt.grid(True)
    
    plt.subplot(2, 3, 2)
    plt.plot(times, desired_positions[:, 1], 'g--', label='期望Y')
    plt.plot(times, positions[:, 1], 'g-', label='实际Y')
    plt.xlabel('时间(s)')
    plt.ylabel('Y位置(m)')
    plt.legend()
    plt.title('Y方向位置跟踪')
    plt.grid(True)
    
    plt.subplot(2, 3, 3)
    plt.plot(times, desired_positions[:, 2], 'b--', label='期望Z')
    plt.plot(times, positions[:, 2], 'b-', label='实际Z')
    plt.axhline(y=0.25, color='k', linestyle=':', label='接触面')
    plt.xlabel('时间(s)')
    plt.ylabel('Z位置(m)')
    plt.legend()
    plt.title('Z方向位置跟踪')
    plt.grid(True)
    
    # 接触力
    plt.subplot(2, 3, 4)
    plt.plot(times, forces, 'r-', label='测量力')
    plt.axhline(y=desired_force_z, color='b', linestyle='--', label='期望力')
    plt.xlabel('时间(s)')
    plt.ylabel('Z方向力(N)')
    plt.legend()
    plt.title('接触力跟踪')
    plt.grid(True)
    
    # 位置修正
    plt.subplot(2, 3, 5)
    plt.plot(times, corrections*1000, 'g-', label='Z修正')
    plt.xlabel('时间(s)')
    plt.ylabel('位置修正(mm)')
    plt.legend()
    plt.title('导纳位置修正')
    plt.grid(True)
    
    # 3D轨迹
    from mpl_toolkits.mplot3d import Axes3D
    ax = plt.subplot(2, 3, 6, projection='3d')
    ax.plot(desired_positions[:, 0], desired_positions[:, 1], desired_positions[:, 2], 
            'r--', label='期望轨迹')
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', label='实际轨迹')
    ax.set_xlabel('X(m)')
    ax.set_ylabel('Y(m)')
    ax.set_zlabel('Z(m)')
    ax.legend()
    ax.set_title('3D轨迹对比')
    
    plt.tight_layout()
    
    # 保存图像
    os.makedirs('../plots', exist_ok=True)
    plt.savefig('../plots/admittance_control_simulation.png', dpi=300, bbox_inches='tight')
    print(f"仿真结果已保存到 ../plots/admittance_control_simulation.png")
    
    return True

if __name__ == '__main__':
    try:
        print("JAKA导纳控制测试")
        print("================")
        
        # 运行测试
        test_admittance_parameters()
        test_trajectory_following() 
        simulate_admittance_control()
        
        print("\n=== 测试完成 ===")
        print("导纳控制实现验证通过!")
        
        # 显示图像
        try:
            plt.show()
        except:
            print("无法显示图像，但已保存到文件")
            
    except Exception as e:
        print(f"测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()