#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
import csv
import os
import sys
from scipy.linalg import expm
import threading
import queue

# Add GUFIC path to import utilities
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from misc_func import initialize_trajectory, set_gains, vee_map, hat_map
from filter import ButterLowPass
from scipy.linalg import block_diag

# Add Jaka API path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'python')))
import __common
__common.init_env()

# ROS2 message imports
from ros2_force_sensor.msg import ForceSensorData
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

def adjoint_g(g):
    """Compute the adjoint matrix for a homogeneous transformation matrix"""
    R = g[:3, :3]
    p = g[:3, 3]
    Ad_g = np.zeros((6, 6))
    Ad_g[:3, :3] = R
    Ad_g[3:, 3:] = R
    Ad_g[:3, 3:] = hat_map(p) @ R
    return Ad_g

class JakaRobotState:
    """真实Jaka机械臂状态管理类"""
    def __init__(self, robot_ip="192.168.125.3"):
        # Import jkrc after environment setup
        import jkrc
        self.jkrc = jkrc
        
        self.robot_ip = robot_ip
        self.rc = None
        self.n_joints = 6
        
        # Robot state variables
        self.q = np.zeros(6)  # Joint positions
        self.dq = np.zeros(6)  # Joint velocities
        self.tau = np.zeros(6)  # Joint torques
        
        # End-effector state
        self.ee_pos = np.zeros(3)  # End-effector position
        self.ee_rot = np.eye(3)   # End-effector rotation matrix
        self.ee_wrench = np.zeros(6)  # End-effector force/torque
        
        # Jacobian matrices
        self.Jp = np.zeros((3, 6))  # Position jacobian
        self.Jr = np.zeros((3, 6))  # Rotation jacobian
        
        # Initialize force sensor filter
        dt = 1.0 / 125.0  # 125Hz control frequency
        fs = 125.0
        cutoff = 10
        self.lp_filter = ButterLowPass(cutoff, fs, order=5)
        
        # Initialize state-space filter
        cut_off_freq = 5
        self.Ad, self.Bd = self.define_filter(cut_off_freq, dt)
        self.filter_state = np.zeros((12, 1))
        
        # Connection status
        self.connected = False
        self.enabled = False
        
    def connect_robot(self):
        """连接到Jaka机械臂"""
        try:
            self.rc = self.jkrc.RC(self.robot_ip)
            result = self.rc.login()
            if result == 0:
                print(f"Successfully connected to Jaka robot at {self.robot_ip}")
                self.connected = True
                
                # Power on and enable robot
                power_result = self.rc.power_on()
                enable_result = self.rc.enable_robot()
                
                if power_result == 0 and enable_result == 0:
                    print("Robot powered on and enabled successfully")
                    self.enabled = True
                else:
                    print(f"Failed to enable robot: power={power_result}, enable={enable_result}")
                    
                return True
            else:
                print(f"Failed to connect to robot: {result}")
                return False
        except Exception as e:
            print(f"Error connecting to robot: {e}")
            return False
    
    def disconnect_robot(self):
        """断开机械臂连接"""
        if self.rc and self.connected:
            try:
                if self.enabled:
                    self.rc.disable_robot()
                    self.rc.power_off()
                self.rc.logout()
                print("Robot disconnected successfully")
            except Exception as e:
                print(f"Error disconnecting robot: {e}")
        
        self.connected = False
        self.enabled = False
    
    def define_filter(self, cutoff, dt, dim=6):
        """定义状态空间滤波器"""
        ws = cutoff
        A = np.array([[0, 1],
                      [-ws**2, -2 * 1 * ws]])
        B = np.array([[0], [ws**2]])
        
        # Calculate Ad = exp(A*dt)
        Ad1 = expm(A * dt)
        
        # Calculate Bd using approximation: Bd ≈ A^(-1)*(Ad - I)*B
        if np.linalg.det(A) != 0:
            Bd1 = np.linalg.inv(A) @ (Ad1 - np.eye(2)) @ B
        else:
            Bd1 = B * dt

        # Stack Ad and Bd for dim times
        Ad = block_diag(*[Ad1 for _ in range(dim)])
        Bd = block_diag(*[Bd1 for _ in range(dim)])

        return Ad, Bd
    
    def lp_filter_implemented(self, force_torque):
        """实现低通滤波"""
        # 0, 2, 4, 6, 8, 10 indices are filtered values
        xf = self.filter_state[::2]
        # 1, 3, 5, 7, 9, 11 indices are filtered derivative values
        dxf = self.filter_state[1::2]

        self.filter_state = self.Ad @ self.filter_state + self.Bd @ force_torque.reshape((-1,1))
        return xf, dxf
    
    def update_robot_state(self):
        """更新机械臂状态"""
        if not self.connected or not self.enabled:
            return False
            
        try:
            # 保存上一次的关节位置用于计算速度
            q_prev = self.q.copy()
            
            # Get joint positions
            joint_pos = self.rc.get_joint_position()
            if joint_pos[0] == 0:  # Success
                self.q = np.array(joint_pos[1])
                
                # 计算关节速度 (数值微分)
                dt = 1.0 / 125.0  # 125Hz控制频率
                if hasattr(self, '_first_update'):
                    self.dq = (self.q - q_prev) / dt
                else:
                    self.dq = np.zeros(6)
                    self._first_update = True
            
            # Get TCP position and orientation
            tcp_pos = self.rc.get_tcp_position()
            if tcp_pos[0] == 0:  # Success
                tcp_data = tcp_pos[1]
                self.ee_pos = np.array([tcp_data[0], tcp_data[1], tcp_data[2]]) / 1000.0  # Convert mm to m
                
                # Convert RPY to rotation matrix
                roll, pitch, yaw = tcp_data[3], tcp_data[4], tcp_data[5]
                self.ee_rot = self.rpy_to_rotation_matrix(roll, pitch, yaw)
            
            # Update Jacobian (基于实际几何参数)
            self.update_jacobian()
            
            return True
        except Exception as e:
            print(f"Error updating robot state: {e}")
            return False
    
    def rpy_to_rotation_matrix(self, roll, pitch, yaw):
        """Convert RPY angles to rotation matrix"""
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
        
        return R_z @ R_y @ R_x
    
    def update_jacobian(self):
        """更新雅可比矩阵 - 基于XML文件的实际几何参数"""
        
        # 基于XML文件的JAKA机械臂几何参数
        # 关节位置 (从XML中提取)
        joint_positions = [
            [0, 0, 0.10765],              # Joint 1: base to link1
            [0, 0, 0],                    # Joint 2: link1 to link2 
            [0.595, 0, 0],                # Joint 3: link2 to link3
            [0.5715, 0, -0.1315],         # Joint 4: link3 to link4
            [0, -0.115, 0],               # Joint 5: link4 to link5
            [0, 0.1035, 0]                # Joint 6: link5 to link6
        ]
        
        # 关节轴方向 (从XML中提取)
        joint_axes = [
            [0, 0, 1],     # Joint 1: Z轴旋转
            [0, 0, 1],     # Joint 2: Z轴旋转 (经过旋转变换)
            [0, 0, 1],     # Joint 3: Z轴旋转
            [0, 0, 1],     # Joint 4: Z轴旋转
            [0, 0, 1],     # Joint 5: Z轴旋转 (经过旋转变换)
            [0, 0, 1]      # Joint 6: Z轴旋转 (经过旋转变换)
        ]
        
        # 关节旋转变换 (quat from XML: w, x, y, z)
        joint_rotations = [
            [1, 0, 0, 0],                           # Joint 1: 无旋转
            [0.707105, 0.707108, 0, 0],            # Joint 2: 90度绕X轴
            [1, 0, 0, 0],                           # Joint 3: 无旋转
            [1, 0, 0, 0],                           # Joint 4: 无旋转
            [0.707105, 0.707108, 0, 0],            # Joint 5: 90度绕X轴
            [0.707105, -0.707108, 0, 0]            # Joint 6: -90度绕X轴
        ]
        
        # 末端执行器位置 (相对于最后一个关节)
        ee_offset = [0.0, 0.0, 0.14]  # 从XML中的site位置
        
        # 计算正向运动学 - 得到各关节的变换矩阵
        T = np.eye(4)
        transforms = [T.copy()]  # T0 (base frame)
        
        for i in range(6):
            # 关节平移
            trans = np.eye(4)
            trans[:3, 3] = joint_positions[i]
            
            # 关节旋转 (四元数转旋转矩阵)
            qw, qx, qy, qz = joint_rotations[i]
            R = self.quaternion_to_rotation_matrix(qw, qx, qy, qz)
            rot_transform = np.eye(4)
            rot_transform[:3, :3] = R
            
            # 关节角度旋转
            joint_rot = np.eye(4)
            axis = np.array(joint_axes[i])
            if i < len(self.q):
                joint_rot[:3, :3] = self.rotation_matrix_from_axis_angle(axis, self.q[i])
            
            # 组合变换
            T = T @ trans @ rot_transform @ joint_rot
            transforms.append(T.copy())
        
        # 添加末端执行器偏移
        ee_trans = np.eye(4)
        ee_trans[:3, 3] = ee_offset
        T_ee = T @ ee_trans
        
        # 计算雅可比矩阵
        self.Jp.fill(0)
        self.Jr.fill(0)
        
        # 末端执行器位置
        pe = T_ee[:3, 3]
        
        for i in range(6):
            # 获取第i个关节的位置和Z轴方向
            Ti = transforms[i]
            pi = Ti[:3, 3]  # 关节i的位置
            
            # 关节轴在全局坐标系中的方向
            axis_local = np.array(joint_axes[i])
            zi = Ti[:3, :3] @ axis_local  # 转换到全局坐标系
            
            # 位置雅可比: zi × (pe - pi)
            r = pe - pi
            self.Jp[:, i] = np.cross(zi, r)
            
            # 姿态雅可比: zi
            self.Jr[:, i] = zi
    
    def quaternion_to_rotation_matrix(self, w, x, y, z):
        """四元数转旋转矩阵"""
        # 归一化四元数
        norm = np.sqrt(w*w + x*x + y*y + z*z)
        if norm == 0:
            return np.eye(3)
        w, x, y, z = w/norm, x/norm, y/norm, z/norm
        
        # 旋转矩阵
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])
        return R
    
    def rotation_matrix_from_axis_angle(self, axis, angle):
        axis = np.array(axis)
        axis = axis / np.linalg.norm(axis)  # 归一化
        
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
        return R
    
    def get_pose(self):
        """获取末端执行器位姿"""
        return self.ee_pos.copy(), self.ee_rot.copy()
    
    def get_body_jacobian(self):
        """获取体坐标系雅可比矩阵"""
        J = np.vstack((self.Jp, self.Jr))
        
        # Transform to body frame
        g = np.eye(4)
        g[:3, :3] = self.ee_rot
        g[:3, 3] = self.ee_pos
        
        Ad_g_inv = np.linalg.inv(adjoint_g(g))
        return Ad_g_inv @ J
    
    def get_body_ee_velocity(self):
        """获取末端执行器在体坐标系的速度"""
        Jb = self.get_body_jacobian()
        return Jb @ self.dq.reshape(-1, 1)
    
    def get_ee_force(self, force_torque_data):
        """处理末端力传感器数据"""
        force_torque = np.array([
            force_torque_data[0], force_torque_data[1], force_torque_data[2],  # Force
            force_torque_data[3], force_torque_data[4], force_torque_data[5]   # Torque
        ])
        
        ft, dft = self.lp_filter_implemented(force_torque)
        return ft, dft
    
    def set_joint_positions(self, joint_positions):
        """设置关节位置 (JAKA只支持位置控制)"""
        if not self.connected or not self.enabled:
            return False
            
        try:
            # Send joint move command
            result = self.rc.joint_move(joint_positions.tolist(), 0, True, 2*math.pi)
            return result == 0
        except Exception as e:
            print(f"Error setting joint positions: {e}")
            return False

class JakaControllerNode(Node):
    """Jaka机械臂ROS2控制节点"""
    def __init__(self):
        super().__init__('jaka_controller_node')
        
        # Robot configuration
        self.robot_ip = "192.168.125.3"  # 根据实际IP修改
        self.control_frequency = 125  # Hz
        self.dt = 1.0 / self.control_frequency
        
        # Initialize robot state
        self.robot_state = JakaRobotState(self.robot_ip)
        
        # Control parameters
        self.task = "sphere"  # or "regulation", "circle", "line"
        self.fz = -10.0  # Desired contact force in Z direction (负值表示压力)
        
        # Initialize trajectory and gains
        self.pd_t, self.Rd_t, self.dpd_t, self.dRd_t, self.ddpd_t, self.ddRd_t = initialize_trajectory(self.task, name="jaka")
        # 注意：对于导纳控制，这些增益的含义略有不同
        self.Kp, self.KR, self.Kd, self.kp_force, self.kd_force, self.ki_force, self.zeta = set_gains("GUFIC", self.task, "jaka")
        
        # Control state variables
        self.iter = 0
        self.int_force_prev = np.zeros((6, 1))
        self.int_sat = 50
        
        # 导纳控制参数
        self.admittance_gain = 0.01  # 导纳增益
        self.max_position_correction = 0.05  # 最大位置修正量 (m)
        self.max_joint_correction = 0.05  # 最大关节角修正量 (rad)
        
        # Energy tank parameters
        self.T_f_low, self.T_f_high, self.delta_f = 0.5, 20, 1
        self.T_i_low, self.T_i_high, self.delta_i = 0.5, 20, 1
        
        # ROS2 Publishers and Subscribers
        self.force_sensor_sub = self.create_subscription(
            ForceSensorData,
            '/force_sensor_data',
            self.force_sensor_callback,
            10
        )
        
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # self.wrench_pub = self.create_publisher(
        #     WrenchStamped,
        #     '/ee_wrench',
        #     10
        # )
        
        # Force sensor data
        self.latest_force_data = np.zeros(6)
        self.force_data_received = False
        
        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        # CSV logging
        self.setup_logging()
        
        # Connect to robot
        self.get_logger().info("Connecting to Jaka robot...")
        if self.robot_state.connect_robot():
            self.get_logger().info("Robot connected successfully")
        else:
            self.get_logger().error("Failed to connect to robot")
            
        # Initialize robot pose
        self.gd = np.eye(4)
        self.max_time = 10.0  # seconds
        
    def setup_logging(self):
        """设置CSV数据记录"""
        os.makedirs('./log', exist_ok=True)
        
        # Joint state log
        self.joint_csv_file = open('./log/joint_angles_real.csv', mode='w', newline='')
        self.joint_csv_writer = csv.writer(self.joint_csv_file)
        self.joint_csv_writer.writerow(['time', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6'])
        
        # Force data log
        self.force_csv_file = open('./log/contact_forces_real.csv', mode='w', newline='')
        self.force_csv_writer = csv.writer(self.force_csv_file)
        self.force_csv_writer.writerow(['time', 'fx', 'fy', 'fz', 'mx', 'my', 'mz', 
                                       'force_magnitude', 'torque_magnitude'])
        
        # Trajectory tracking log
        self.trajectory_csv_file = open('./log/trajectory_tracking_real.csv', mode='w', newline='')
        self.trajectory_csv_writer = csv.writer(self.trajectory_csv_file)
        self.trajectory_csv_writer.writerow(['time', 'desired_x', 'desired_y', 'desired_z', 
                                           'actual_x', 'actual_y', 'actual_z',
                                           'error_x', 'error_y', 'error_z', 'error_norm'])
    
    def force_sensor_callback(self, msg):
        """力传感器数据回调"""
        self.latest_force_data = np.array([
            msg.force.x, msg.force.y, msg.force.z,
            msg.torque.x, msg.torque.y, msg.torque.z
        ])
        self.force_data_received = True
    
    def get_desired_force(self):
        """获取期望的接触力"""
        return np.array([0, 0, self.fz, 0, 0, 0]).reshape((-1,1))
    
    def admittance_control(self):
        """导纳控制算法 (适用于位置控制机械臂)"""
        if not self.force_data_received:
            return self.robot_state.q.copy()  # 返回当前位置
            
        t = self.iter * self.dt
        
        # Update robot state
        if not self.robot_state.update_robot_state():
            return self.robot_state.q.copy()
        
        # Get robot state
        Jb = self.robot_state.get_body_jacobian()
        p, R = self.robot_state.get_pose()
        
        # Desired trajectory (nominal trajectory)
        pd_nominal = self.pd_t(t).reshape(-1)
        Rd_nominal = self.Rd_t(t)
        
        # Measured force/torque
        Fe, d_Fe = self.robot_state.get_ee_force(self.latest_force_data)
        
        # Desired force/torque
        Fd_star = self.get_desired_force()
        
        # Force error
        e_force = Fe - Fd_star  # 注意：导纳控制中力误差的符号与阻抗控制相反
        
        # Force integration for steady-state error elimination
        self.int_force_prev += e_force * self.dt
        self.int_force_prev = np.clip(self.int_force_prev, -self.int_sat, self.int_sat)
        
        # Admittance parameters (虚拟质量-阻尼-刚度系统)
        M_adm = np.eye(6) * 1.0     # 虚拟惯性矩阵
        D_adm = np.eye(6) * 20.0    # 虚拟阻尼矩阵  
        K_adm = np.eye(6) * 100.0   # 虚拟刚度矩阵
        
        # 计算位置修正量 (基于力误差)
        # 使用简化的导纳模型: dx = gain * F_error
        position_correction = self.admittance_gain * e_force
        
        # 将末端执行器的位置修正转换为关节空间
        # 只考虑位置修正，忽略姿态修正以简化计算
        position_correction_ee = position_correction[:3]  # 只取位置部分
        
        try:
            # 使用雅可比伪逆将末端位置修正转换为关节角修正
            J_pos = self.robot_state.Jp  # 位置雅可比
            joint_correction = np.linalg.pinv(J_pos) @ position_correction_ee.flatten()
        except:
            joint_correction = np.zeros(6)
        
        # 限制位置修正量
        position_correction_ee = np.clip(position_correction_ee, 
                                       -self.max_position_correction, 
                                       self.max_position_correction)
        
        # 限制关节修正量的大小以确保安全
        joint_correction = np.clip(joint_correction, 
                                 -self.max_joint_correction, 
                                 self.max_joint_correction)
        
        # 计算期望关节位置（名义轨迹 + 力导纳修正）
        
        # 第一步：计算名义轨迹的关节角 (使用逆运动学)
        try:
            nominal_joint_positions = self.inverse_kinematics_simple(pd_nominal, Rd_nominal)
        except:
            # 如果逆运动学失败，使用当前位置
            nominal_joint_positions = self.robot_state.q.copy()
        
        # 第二步：应用力导纳修正
        target_joint_positions = nominal_joint_positions + joint_correction
        
        # 关节限位保护
        joint_limits_min = np.array([-3.14, -2.27, -3.14, -3.14, -2.27, -3.14])
        joint_limits_max = np.array([3.14, 2.27, 3.14, 3.14, 2.27, 3.14])
        
        target_joint_positions = np.clip(target_joint_positions, 
                                       joint_limits_min, joint_limits_max)
        
        return target_joint_positions
    
    def inverse_kinematics_simple(self, target_pos, target_rot, initial_q=None):
        """简单的逆运动学求解器 (数值方法)"""
        if initial_q is None:
            q_current = self.robot_state.q.copy()
        else:
            q_current = initial_q.copy()
            
        # 迭代参数
        max_iterations = 10
        tolerance = 0.001  # 1mm tolerance
        step_size = 0.1
        
        for i in range(max_iterations):
            # 更新当前状态的雅可比
            self.robot_state.q = q_current
            self.robot_state.update_jacobian()
            
            # 计算当前末端位置
            current_pos, current_rot = self.robot_state.get_pose()
            
            # 位置误差
            pos_error = target_pos - current_pos
            
            # 旋转误差 (简化处理)
            rot_error = np.zeros(3)  # 暂时忽略旋转误差
            
            # 总误差
            error = np.concatenate([pos_error, rot_error])
            
            # 检查收敛
            if np.linalg.norm(pos_error) < tolerance:
                break
                
            # 雅可比矩阵
            J = np.vstack([self.robot_state.Jp, self.robot_state.Jr])
            
            try:
                # 使用伪逆求解
                dq = step_size * np.linalg.pinv(J) @ error
                q_current += dq
                
                # 关节限位
                joint_limits_min = np.array([-3.14, -2.27, -3.14, -3.14, -2.27, -3.14])
                joint_limits_max = np.array([3.14, 2.27, 3.14, 3.14, 2.27, 3.14])
                q_current = np.clip(q_current, joint_limits_min, joint_limits_max)
                
            except np.linalg.LinAlgError:
                break
        
        # 恢复原来的关节角度
        self.robot_state.q = self.robot_state.q  # 保持当前状态
        
        return q_current
    
    def control_loop(self):
        """主控制循环"""
        if not self.robot_state.connected or not self.robot_state.enabled:
            return
            
        # Check time limit
        t = self.iter * self.dt
        if t >= self.max_time:
            self.get_logger().info(f"Control completed. Max time {self.max_time}s reached.")
            self.destroy_node()
            return
        
        # Print progress
        if self.iter % 125 == 0:  # Every second at 125Hz
            self.get_logger().info(f"Control step: {self.iter}, Time: {t:.3f}s")
        
        # Execute control algorithm
        target_joint_positions = self.admittance_control()
        
        # Send command to robot
        self.robot_state.set_joint_positions(target_joint_positions)
        
        # Publish ROS topics
        self.publish_joint_states()
        # self.publish_wrench()  # 注释掉如果不需要发布力扭矩信息
        
        # Log data
        self.log_data(t)
        
        self.iter += 1
    
    def publish_joint_states(self):
        """发布关节状态"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint_{i+1}' for i in range(6)]
        msg.position = self.robot_state.q.tolist()
        msg.velocity = self.robot_state.dq.tolist()
        self.joint_state_pub.publish(msg)
    
    def publish_wrench(self):
        """发布末端力扭矩"""
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ee_link"
        
        msg.wrench.force.x = float(self.latest_force_data[0])
        msg.wrench.force.y = float(self.latest_force_data[1])
        msg.wrench.force.z = float(self.latest_force_data[2])
        msg.wrench.torque.x = float(self.latest_force_data[3])
        msg.wrench.torque.y = float(self.latest_force_data[4])
        msg.wrench.torque.z = float(self.latest_force_data[5])
        
        self.wrench_pub.publish(msg)
    
    def log_data(self, t):
        """记录数据到CSV"""
        # Log joint states
        self.joint_csv_writer.writerow([t] + self.robot_state.q.tolist())
        
        # Log force data
        force_magnitude = np.linalg.norm(self.latest_force_data[:3])
        torque_magnitude = np.linalg.norm(self.latest_force_data[3:])
        self.force_csv_writer.writerow([t] + self.latest_force_data.tolist() + 
                                      [force_magnitude, torque_magnitude])
        
        # Log trajectory tracking
        pd_current = self.pd_t(t).flatten()
        p_actual = self.robot_state.ee_pos
        error = p_actual - pd_current
        error_norm = np.linalg.norm(error)
        
        self.trajectory_csv_writer.writerow([
            t,
            pd_current[0], pd_current[1], pd_current[2],
            p_actual[0], p_actual[1], p_actual[2],
            error[0], error[1], error[2], error_norm
        ])
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.get_logger().info("Shutting down Jaka controller...")
        
        # Disconnect robot
        self.robot_state.disconnect_robot()
        
        # Close CSV files
        if hasattr(self, 'joint_csv_file'):
            self.joint_csv_file.close()
        if hasattr(self, 'force_csv_file'):
            self.force_csv_file.close()
        if hasattr(self, 'trajectory_csv_file'):
            self.trajectory_csv_file.close()
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = JakaControllerNode()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Keyboard interrupt received")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
