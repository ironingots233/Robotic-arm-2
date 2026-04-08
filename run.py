import serial
import struct
import math
import time
import concurrent.futures
import socket
from motor_driver import SerialPort, MotorCmd, MotorData, move
from servo_driver import RobotHandController


#===================物理参数===================

L0 = 0.13  # 底座旋转中心到大臂关节(ID:1)的垂直高度
L1 = 0.25424  # 大臂(ID:1)到小臂(ID:2)的长度
L2 = 0.25424  # 小臂(ID:2)到腕部(ID:3)的长度
L3 = 0.196  # 腕部(ID:3)到末端执行器(夹爪/末端点)的长度
A1, B1, C1 = 4.1740, -0.0525, 0.0245 # 动力学系数
A2, B2, C2 = 2.2243, -0.4785, 0.0394
A3, B3, C3 = 0.1385, 0.0526, -0.0098

#=================运动算法=================
# 丝滑启动(无bug)
def enable_gravity(duration=2.5):
    """
    在开启完全重力补偿前，使用 S 型曲线平滑过渡力矩，防止电机抽动。
    duration: 平滑过渡耗时（秒）
    """
    start_time = time.time()
    
    while True:
        elapsed_time = time.time() - start_time
        
        # 1. 判断是否完成过渡
        if elapsed_time >= duration:
            break
            
        # 2. 计算 S 型曲线比例 (从 0 柔和地增加到 1)
        linear_factor = elapsed_time / duration
        ramp_factor = 0.5 * (1.0 - math.cos(math.pi * linear_factor))
        
        # 3. 实时读取当前角度（假设 dt1.q 等变量能实时获取到底层数据）
        q1 = dt1.q
        q2 = dt2.q
        q3 = dt3.q
        
        # 4. 计算全量理论力矩
        # (注意：A1, B1, C1 等参数需确保在此函数作用域内可访问，可以作为全局变量或传参进来)
        tau3_target = A3 * math.cos(q1 + q2 + q3) + B3 * math.sin(q1 + q2 + q3) + C3
        tau2_target = A2 * math.cos(q1 + q2) + B2 * math.sin(q1 + q2) + C2 + tau3_target
        tau1_target = A1 * math.cos(q1) + B1 * math.sin(q1) + C1 + tau2_target
        
        # 5. 乘以缓坡比例，下发给电机
        mt3.tau = tau3_target * ramp_factor
        mt2.tau = tau2_target * ramp_factor
        mt1.tau = tau1_target * ramp_factor
        ser.sendRecv(mt1, dt1)
        ser.sendRecv(mt2, dt2)
        ser.sendRecv(mt3, dt3)
        
        # 给定一个极小的延时，防止跑满 CPU
        time.sleep(0.005) 
        
# 正运动学(无bug)
def forward_kinematics(q0, q1, q2, q3):
    """
    正运动学：输入当前关节角度，返回末端执行器的空间坐标 (X, Y, Z)
    """
    th1 = q1
    th2 = q1 + q2
    th3 = q1 + q2 + q3
    
    r = L1 * math.cos(th1) + L2 * math.cos(th2) + L3 * math.cos(th3)
    z = L0 + L1 * math.sin(th1) + L2 * math.sin(th2) + L3 * math.sin(th3)
    
    x = r * math.cos(q0)
    y = r * math.sin(q0)
    
    return x, y, z

# 逆运动学
def inverse_kinematics(x, y, z, phi=0.0):
    """
    逆运动学：输入目标空间坐标 (X,Y,Z) 和末端俯仰角 phi ,得到各个电机角度
    """
    q0 = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2)
    
    r_w = r - L3 * math.cos(phi)
    z_w = z - L0 - L3 * math.sin(phi)
    
    D_sq = r_w**2 + z_w**2
    cos_q2 = (D_sq - L1**2 - L2**2) / (2 * L1 * L2)
    
    cos_q2 = max(-1.0, min(1.0, cos_q2))
    if cos_q2 >= 1.0 and D_sq > (L1+L2)**2 + 0.001:
        raise ValueError("⚠️ 目标坐标超出机械臂最大臂展！")
    
    q2 = -math.acos(cos_q2)  
    k1 = L1 + L2 * math.cos(q2)
    k2 = L2 * math.sin(q2)
    
    # 利用算出的 q2 来推导 q1，q1 会自动适应新的手肘姿态
    q1 = math.atan2(z_w, r_w) - math.atan2(k2, k1)
    
    # 保证末端俯仰角 phi 不变
    q3 = phi - (q1 + q2)
    
    return q0, q1, q2, q3

# 移动到指定坐标
def move_to_target(target_x, target_y, target_z, target_phi=0.0, target_raw=0.0, gripper_state=None, max_joint_speed=1.0, min_duration=0.2):
    
    q0_end, q1_end, q2_end, q3_end = inverse_kinematics(target_x, target_y, target_z, target_phi)

    # 2. 获取当前实际角度作为起点
    q0_start = dt0.q
    q1_start = dt1.q
    q2_start = dt2.q
    q3_start = dt3.q

    # 3. 计算运动耗时 T
    max_angle_diff = max(
        abs(q0_end - q0_start),
        abs(q1_end - q1_start),
        abs(q2_end - q2_start),
        abs(q3_end - q3_start)
    )
    
    # 耗时 = 最大角度差 / 设定最大角速度，并使用 min_duration 兜底
    T = max(max_angle_diff / max_joint_speed, min_duration)

    # 控制舵机
    try:
        if hand.is_connected():
            
            # --- 控制夹爪 ---
            hand.gripper(gripper_state)

            # --- 控制手腕 ---
            _, current_raw = hand.get_wrist_state()
            hand.wrist(target_raw)

    except NameError:
        pass # 若全局 hand 对象未被定义，则跳过


    # 4. 定义五次多项式系数计算闭包函数
    def get_quintic_coeffs(q_s, q_e, duration):
        h = q_e - q_s
        a0 = q_s
        a1 = 0.0
        a2 = 0.0
        a3 = 10 * h / (duration**3)
        a4 = -15 * h / (duration**4)
        a5 = 6 * h / (duration**5)
        return a0, a1, a2, a3, a4, a5


    coeffs0 = get_quintic_coeffs(q0_start, q0_end, T)
    coeffs1 = get_quintic_coeffs(q1_start, q1_end, T)
    coeffs2 = get_quintic_coeffs(q2_start, q2_end, T)
    coeffs3 = get_quintic_coeffs(q3_start, q3_end, T)

    # 定义计算当前 t 时刻期望位置 q 和期望速度 dq 的闭包函数
    def calc_q_dq(coeffs, t):
        a0, a1, a2, a3, a4, a5 = coeffs
        q = a0 + a1*t + a2*(t**2) + a3*(t**3) + a4*(t**4) + a5*(t**5)
        dq = a1 + 2*a2*t + 3*a3*(t**2) + 4*a4*(t**3) + 5*a5*(t**4)
        return q, dq

    # 5. 进入插值运动循环
    start_time = time.time()
    while True:
        t = time.time() - start_time
        
        # 限制 t 不超过总耗时 T
        if t >= T:
            t = T

        # 获取当前时刻各关节的期望位置和速度
        mt0.q, mt0.dq = calc_q_dq(coeffs0, t)
        mt1.q, mt1.dq = calc_q_dq(coeffs1, t)
        mt2.q, mt2.dq = calc_q_dq(coeffs2, t)
        mt3.q, mt3.dq = calc_q_dq(coeffs3, t)

        # 实时计算动力学前馈力矩 (重力补偿)
        mt3.tau = A3 * math.cos(dt1.q + dt2.q + dt3.q) + B3 * math.sin(dt1.q + dt2.q + dt3.q) + C3
        mt2.tau = A2 * math.cos(dt1.q + dt2.q) + B2 * math.sin(dt1.q + dt2.q) + C2 + mt3.tau
        mt1.tau = A1 * math.cos(dt1.q) + B1 * math.sin(dt1.q) + C1 + mt2.tau
        mt0.tau = 0.0 

        # 下发指令
        ser.sendRecv(mt0, dt0)
        ser.sendRecv(mt1, dt1)
        ser.sendRecv(mt2, dt2)
        ser.sendRecv(mt3, dt3)

        # 运动结束判断
        if t >= T:
            print("\n✅ 运动到达目标点！")
            break

        # 控制控制环路频率 (例如约 500Hz)
        time.sleep(0.002)


if __name__ == "__main__":
    
    # 实例化
    hand = RobotHandController(port='/dev/ttyUSB4')
    ser = SerialPort("/dev/ttyUSB1") 
    mt0 = MotorCmd(id=0, direction=1, offset=+0.68)
    mt1 = MotorCmd(id=1, direction=-1, offset=+0.926-0.1829+3.14)
    mt2 = MotorCmd(id=2, direction=1, offset=+3.162-0.1829)
    mt3 = MotorCmd(id=3, direction=-1, offset=-0.1)
    dt0 = MotorData()
    dt1 = MotorData()
    dt2 = MotorData()
    dt3 = MotorData()
    for _ in range(5):
        ser.sendRecv(mt0, dt0)
        ser.sendRecv(mt1, dt1)
        ser.sendRecv(mt2, dt2)
        ser.sendRecv(mt3, dt3)
        time.sleep(0.005)
    mt0.mode, mt1.mode, mt2.mode, mt3.mode = 1, 1, 1, 1
    # 初始化






    # 斜坡启动
    enable_gravity(duration=0.5)


    

    #更新指令位置
    mt0.q = dt0.q
    mt1.q = dt1.q
    mt2.q = dt2.q
    mt3.q = dt3.q


    #赋予力气与阻抗
    for mt in [mt0, mt1, mt2, mt3]:
        mt.kp = 0.05
        mt.kd = 0.008

    try:

        

        move_to_target(
            target_x=0.25, 
            target_y=-0.2, 
            target_z=0.35, 
            target_phi=0, 
            target_raw=0.5, 
            gripper_state=-1
        )
        


        while True:

            mt3.tau = A3 * math.cos(dt1.q + dt2.q + dt3.q) + B3 * math.sin(dt1.q + dt2.q + dt3.q) + C3
            mt2.tau = A2 * math.cos(dt1.q + dt2.q) + B2 * math.sin(dt1.q + dt2.q) + C2 + mt3.tau
            mt1.tau = A1 * math.cos(dt1.q) + B1 * math.sin(dt1.q) + C1 + mt2.tau
            
            ser.sendRecv(mt0, dt0)
            ser.sendRecv(mt1, dt1)
            ser.sendRecv(mt2, dt2)
            ser.sendRecv(mt3, dt3)

            # 获取舵机实际角度
            servo1_step, servo1_rad = hand.get_wrist_state()
            curr_x, curr_y, curr_z = forward_kinematics(dt0.q, dt1.q, dt2.q, dt3.q)
            print(f" X:{curr_x:>7.3f} m, Y:{curr_y:>7.3f} m, Z:{curr_z:>7.3f} m | "
                  f" raw:{servo1_rad:>+6.2f} Pitch:{dt3.q:>+6.2f} ", end='\r')
            
            time.sleep(0.01)











    except KeyboardInterrupt:
        for mt in [mt0, mt1, mt2, mt3]:
            mt.mode = 0
            mt.kp = 0.0        
            mt.kd = 0.0       
            mt.dq = 0.0       
            mt.tau = 0.0
        ser.sendRecv(mt0, dt0)
        ser.sendRecv(mt1, dt1)
        ser.sendRecv(mt2, dt2)
        ser.sendRecv(mt3, dt3)
        print("\n程序停止")