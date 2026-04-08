import time
import csv
from motor_driver import SerialPort, MotorCmd, MotorData, move
import random
import math

# ================= 配置参数 =================
# 机械臂物理参数 (单位：米)
L1 = 0.250  # 1号电机到2号电机的臂长 (250mm)
L2 = 0.250  # 2号电机到3号电机的臂长 (250mm)
L3 = 0.200  # 3号电机到末端执行器的长度 (200mm)

# 关节物理限位 (单位：弧度)
Q1_MIN, Q1_MAX = -0.1, 2.9
Q2_MIN, Q2_MAX = 0.0, -2.9
Q3_MIN, Q3_MAX = -1.57, 1.57

# 安全高度配置 (单位：米)
# 假设基座高度为0。设置 0.03m (30mm) 的安全余量防止刮擦桌面
SAFE_Z_MARGIN = 0
# ==========================================

def is_pose_safe(q1, q2, q3):
    """
    利用正运动学计算机械臂各节点高度，判断是否会碰撞桌面。
    重要假设：这里假设 q=0 时机械臂是水平向前伸直的，向上转动角度为正。
    (如果你的机械臂 q=0 时是垂直向上的，请将下方的 math.sin 全部替换为 math.cos)
    """
    # 节点1高度 (2号电机转轴处)
    z1 = L1 * math.sin(q1)
    
    # 节点2高度 (3号电机转轴处)
    # 角度是绝对角度（相对地面的夹角），即前两个关节角之和
    z2 = z1 + L2 * math.sin(q1 + q2)
    
    # 节点3高度 (末端执行器尖端)
    z3 = z2 + L3 * math.sin(q1 + q2 + q3)
    
    # 检查是否有任何一个部件低于安全阈值
    if z1 < SAFE_Z_MARGIN or z2 < SAFE_Z_MARGIN or z3 < SAFE_Z_MARGIN:
        return False
    return True

def generate_random_waypoints(num_points=50):
    """
    生成一组随机且安全的关节角度目标点
    """
    waypoints = []
    attempts = 0
    max_attempts = num_points * 20  # 防止死循环，设置最大随机尝试次数
    
    print(f"正在生成 {num_points} 个随机且安全的动作路点...")
    
    while len(waypoints) < num_points and attempts < max_attempts:
        attempts += 1
        
        # 1. 独立生成符合各自物理限位的随机角度
        q1_rand = random.uniform(Q1_MIN, Q1_MAX)
        q2_rand = random.uniform(Q2_MIN, Q2_MAX)
        q3_rand = random.uniform(Q3_MIN, Q3_MAX)
        
        # 2. 碰撞检测拦截
        if is_pose_safe(q1_rand, q2_rand, q3_rand):
            waypoints.append((q1_rand, q2_rand, q3_rand))
            
    if len(waypoints) < num_points:
        print(f"⚠️ 警告：受限于防碰撞规则，在 {max_attempts} 次尝试中仅找到了 {len(waypoints)} 个安全点。")
    else:
        print("✅ 路点生成完毕！")
        
    return waypoints







recorded_data = []

def collect():
    ser.sendRecv(mt1, dt1)
    ser.sendRecv(mt2, dt2)
    ser.sendRecv(mt3, dt3)

    recorded_data.append({
            "time": time.time(),
            "mt1_pos": dt1.q,
            "mt1_tau": dt1.tau,
            "mt2_pos": dt2.q,
            "mt2_tau": dt2.tau,
            "mt3_pos": dt3.q,
            "mt3_tau": dt3.tau
        })


#初始化
ser = SerialPort("/dev/ttyUSB1") 
mt0 = MotorCmd(id=0, direction=1, offset=+0.68)
mt1 = MotorCmd(id=1, direction=-1, offset=+0.926-0.1829+3.14)
mt2 = MotorCmd(id=2, direction=1, offset=+3.162-0.1829)
mt3 = MotorCmd(id=3, direction=-1, offset=-0.1)
dt0 = MotorData()
dt1 = MotorData()
dt2 = MotorData()
dt3 = MotorData()




#预备
pos1 = move(ser, mt1, dt1, target_pos=2.8, duration=0.5, kp=2.0, kd=0.2)
pos2 = move(ser, mt2, dt2, target_pos=0, duration=2.0, kp=1.0, kd=0.1)
pos3 = move(ser, mt3, dt3, target_pos=1.57, duration=2.0, kp=1.0, kd=0.1)

time.sleep(1)
collect()

#爪子动
while pos3 > -1.4 :
    pos3 = move(ser, mt3, dt3, target_pos=pos3-0.5, duration=0.5, kp=1.0, kd=0.1)
    time.sleep(1)
    collect()
move(ser, mt3, dt3, target_pos=0, duration=2.0, kp=1.0, kd=0.1)


#小臂动
while pos2 > -2.5 :
    pos2 = move(ser, mt2, dt2, target_pos=pos2-0.5, duration=0.5, kp=1.0, kd=0.1)
    time.sleep(1)
    collect()
move(ser, mt2, dt2, target_pos=0, duration=2, kp=1.0, kd=0.1)

#大臂动
while pos1 > 0.3 :
    pos1 = move(ser, mt1, dt1, target_pos=pos1-0.5, duration=1, kp=2.0, kd=0.2)
    time.sleep(1)
    collect()


#归位
move(ser, mt1, dt1, target_pos=2.8, duration=3.0, kp=2.0, kd=0.1)
move(ser, mt2, dt2, target_pos=-2.8, duration=2.0, kp=1.0, kd=0.1)

time.sleep(1)



#---------------随机组合---------------


target_waypoints = generate_random_waypoints(num_points=60)
    
for idx, (target_q1, target_q2, target_q3) in enumerate(target_waypoints):
    print(f"[{idx+1}/{len(target_waypoints)}] 正在移动到随机点: q1={target_q1:.2f}, q2={target_q2:.2f}, q3={target_q3:.2f}")
        
        # 2. 发送位置指令给电机（调用你原有的电机控制接口）
        # mt1.set_position(target_q1)
        # mt2.set_position(target_q2)
        # mt3.set_position(target_q3)
    move(ser, mt3, dt3, target_pos=target_q3, duration=0.5, kp=1.0, kd=0.1)
    move(ser, mt2, dt2, target_pos=target_q2, duration=1, kp=1.0, kd=0.1)
    move(ser, mt1, dt1, target_pos=target_q1, duration=2, kp=2.0, kd=0.2)
    
    
        
        # 3. 给定足够的运动和稳定时间 (避免震荡干扰扭矩读数)
    time.sleep(1) 
    collect()


#归位
move(ser, mt3, dt3, target_pos=0.78, duration=0.5, kp=1.0, kd=0.1)
move(ser, mt1, dt1, target_pos=0.2, duration=2.0, kp=2.0, kd=0.2)
move(ser, mt2, dt2, target_pos=2.9, duration=2.5, kp=1.0, kd=0.1)





csv_filename = "motor_data.csv"
try:
    with open(csv_filename, mode='w', newline='') as file:
        # 定义表头
        writer = csv.DictWriter(file, fieldnames=["time", "mt1_pos", "mt1_tau", "mt2_pos", "mt2_tau", "mt3_pos", "mt3_tau"])
        writer.writeheader()
        # 写入数据
        writer.writerows(recorded_data)
    print(f"\n[数据已保存] 成功保存 {len(recorded_data)} 条记录至 {csv_filename}")
except Exception as e:
    print(f"\n[数据保存失败] {e}")




mt0.mode = 0
mt1.mode = 0
mt2.mode = 0
mt3.mode = 0
ser.sendRecv(mt0, dt0)
ser.sendRecv(mt1, dt1)
ser.sendRecv(mt2, dt2)
ser.sendRecv(mt3, dt3)
print("\n程序停止")










#结束
mt0.mode = 0
mt1.mode = 0
mt2.mode = 0
mt3.mode = 0
ser.sendRecv(mt0, dt0)
ser.sendRecv(mt1, dt1)
ser.sendRecv(mt2, dt2)
ser.sendRecv(mt3, dt3)
print("\n程序停止")






