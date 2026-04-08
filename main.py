import cv2
import torch
import numpy as np
import math
import time
from PIL import Image
from transformers import AutoModel, AutoProcessor

# 导入底层驱动（假设文件在同一目录下）
from motor_driver import SerialPort, MotorCmd, MotorData
from servo_driver import RobotHandController

# ================= 配置参数 =================
MODEL_PATH = "XiaomiRobotics/Xiaomi-Robotics-0-LIBERO"
MOTOR_PORT = "/dev/ttyUSB1"   # 大臂电机串口
SERVO_PORT = "/dev/ttyUSB4"   # 舵机/夹爪串口
BASE_CAM_ID = 2              # 全局摄像头
WRIST_CAM_ID = 0             # 腕部摄像头

# 机械臂几何参数 (单位: m)
L0, L1, L2, L3 = 0.13, 0.25424, 0.25424, 0.196

# 动力学重力补偿系数 (从 run.py 继承)
DYN_COEFFS = {
    'A1': 4.1740, 'B1': -0.0525, 'C1': 0.0245,
    'A2': 2.2243, 'B2': -0.4785, 'C2': 0.0394,
    'A3': 0.1385, 'B3': 0.0526, 'C3': -0.0098
}

# ================= 硬件初始化 =================

def init_hardware():
    print("🔧 正在初始化硬件驱动...")
    # 1. 舵机控制器 (夹爪+手腕)
    hand = RobotHandController(port=SERVO_PORT)
    
    # 2. 电机控制器
    ser = SerialPort(MOTOR_PORT)
    # 根据 run.py 的 offset 设置
    mt = [
        MotorCmd(id=0, direction=1,  offset=+0.68),
        MotorCmd(id=1, direction=-1, offset=+0.926-0.1829+3.14),
        MotorCmd(id=2, direction=1,  offset=+3.162-0.1829),
        MotorCmd(id=3, direction=-1, offset=-0.1)
    ]
    dt = [MotorData() for _ in range(4)]
    
    for m in mt: m.mode = 1 # 设为位置/力矩混合模式
    
    return hand, ser, mt, dt

# ================= 核心运动函数 =================

def forward_kinematics(q0, q1, q2, q3):
    th1, th2, th3 = q1, q1 + q2, q1 + q2 + q3
    r = L1 * math.cos(th1) + L2 * math.cos(th2) + L3 * math.cos(th3)
    z = L0 + L1 * math.sin(th1) + L2 * math.sin(th2) + L3 * math.sin(th3)
    x, y = r * math.cos(q0), r * math.sin(q0)
    return x, y, z

def inverse_kinematics(x, y, z, phi=0.0, elbow_up=True):
    q0 = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2)
    r_w, z_w = r - L3 * math.cos(phi), z - L0 - L3 * math.sin(phi)
    D_sq = r_w**2 + z_w**2
    cos_q2 = np.clip((D_sq - L1**2 - L2**2) / (2 * L1 * L2), -1.0, 1.0)
    
    q2 = -math.acos(cos_q2) if elbow_up else math.acos(cos_q2)
    k1, k2 = L1 + L2 * math.cos(q2), L2 * math.sin(q2)
    q1 = math.atan2(z_w, r_w) - math.atan2(k2, k1)
    q3 = phi - (q1 + q2)
    return q0, q1, q2, q3

def gravity_comp(dt, mt):
    """ 计算并应用重力补偿 """
    q1, q2, q3 = dt[1].q, dt[2].q, dt[3].q
    tau3 = DYN_COEFFS['A3'] * math.cos(q1 + q2 + q3) + DYN_COEFFS['B3'] * math.sin(q1 + q2 + q3) + DYN_COEFFS['C3']
    tau2 = DYN_COEFFS['A2'] * math.cos(q1 + q2) + DYN_COEFFS['B2'] * math.sin(q1 + q2) + DYN_COEFFS['C2'] + tau3
    tau1 = DYN_COEFFS['A1'] * math.cos(q1) + DYN_COEFFS['B1'] * math.sin(q1) + DYN_COEFFS['C1'] + tau2
    mt[3].tau, mt[2].tau, mt[1].tau, mt[0].tau = tau3, tau2, tau1, 0.0

def move_to_target(target_pos, ser, mt, dt, hand, duration_scale=1.0):
    """ 
    执行五次多项式插值移动
    target_pos: [x, y, z, phi, raw, gripper]
    """
    tx, ty, tz, tphi, traw, tgrip = target_pos
    try:
        q_end = inverse_kinematics(tx, ty, tz, tphi)
    except Exception as e:
        print(f"❌ 逆解失败: {e}")
        return

    q_start = [d.q for d in dt]
    max_diff = max([abs(q_end[i] - q_start[i]) for i in range(4)])
    T = max(max_diff / 1.0, 0.5) * duration_scale # 限制速度

    # 同步控制舵机
    if hand.is_connected():
        # 简单映射：模型输出通常在 [-1, 1], 我们将其映射到夹爪状态
        g_state = 1 if tgrip > 0 else -1
        hand.set_gripper(g_state, speed=int(450/T))
        hand.rotate_wrist(traw, speed=1000)

    # 五次多项式轨迹
    start_time = time.time()
    while True:
        t = time.time() - start_time
        if t >= T: t = T
        
        # 计算插值 q 和 dq
        for i in range(4):
            h = q_end[i] - q_start[i]
            mt[i].q = q_start[i] + (10*h/T**3)*t**3 + (-15*h/T**4)*t**4 + (6*h/T**5)*t**5
            mt[i].dq = (30*h/T**3)*t**2 + (-60*h/T**4)*t**3 + (30*h/T**5)*t**4

        gravity_comp(dt, mt)
        for i in range(4): ser.sendRecv(mt[i], dt[i])
        
        if t >= T: break
        time.sleep(0.002)

# ================= 主逻辑 =================

def main():
    # 1. 初始化模型
    print("🤖 正在加载 VLA 模型 (Xiaomi-LIBERO)...")
    model = AutoModel.from_pretrained(MODEL_PATH, trust_remote_code=True, load_in_8bit=True, device_map="cuda:0").eval()
    processor = AutoProcessor.from_pretrained(MODEL_PATH, trust_remote_code=True, use_fast=False)

    # 2. 初始化硬件
    hand, ser, mt, dt = init_hardware()
    
    # 3. 初始同步与重力补偿启动
    for i in range(4): ser.sendRecv(mt[i], dt[i])
    print("⏳ 正在进行重力补偿平滑启动...")
    # 简化版平滑启动
    st = time.time()
    while time.time() - st < 1.0:
        gravity_comp(dt, mt)
        for i in range(4): ser.sendRecv(mt[i], dt[i])
        time.sleep(0.01)
    
    # 设置运动增益
    for m in mt:
        m.kp, m.kd = 0.12, 0.008

    try:
        while True:
            # --- A. 环境感知 ---
            print("\n📸 采集视觉观测...")
            caps = [cv2.VideoCapture(BASE_CAM_ID), cv2.VideoCapture(WRIST_CAM_ID)]
            imgs = []
            for cap in caps:
                ret, frame = cap.read()
                if ret:
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    imgs.append(Image.fromarray(frame_rgb).resize((224, 224)))
                else:
                    imgs.append(Image.new('RGB', (224, 224), color='black'))
                cap.release()

            # --- B. 读取本体状态 ---
            for i in range(4): ser.sendRecv(mt[i], dt[i])
            curr_x, curr_y, curr_z = forward_kinematics(dt[0].q, dt[1].q, dt[2].q, dt[3].q)
            _, curr_wrist_rad = hand.get_wrist_state()
            curr_wrist_rad = curr_wrist_rad if curr_wrist_rad is not None else 0.0
            
            # 构造 32 维本体向量 (x, y, z, r, p, y, gripper, ...)
            state_vec = np.zeros(32)
            state_vec[0:3] = [curr_x, curr_y, curr_z]
            state_vec[4] = dt[3].q # Pitch
            state_vec[5] = curr_wrist_rad # Yaw/Raw
            state_vec[6] = 1.0 # 假设初始张开
            proprio_state = state_vec.reshape(1, 1, 32)

            # --- C. 模型推理 ---
            instruction = "Pick up the black film roll."
            prompt = (f"<|im_start|>user\n# Base View\n<|vision_start|><|image_pad|><|vision_end|>\n"
                      f"# Left-Wrist View\n<|vision_start|><|image_pad|><|vision_end|>\n"
                      f"Generate robot actions for the task:\n{instruction} /no_cot<|im_end|>\n"
                      f"<|im_start|>assistant\n<cot></cot><|im_end|>\n")
            
            inputs = processor(text=[prompt], images=imgs, return_tensors="pt").to(model.device)
            inputs["state"] = torch.from_numpy(proprio_state).to(model.device, model.dtype).view(1, 1, -1)
            inputs["action_mask"] = processor.get_action_mask("libero_all").to(model.device, model.dtype)
            
            print("🧠 模型推理中...")
            with torch.no_grad():
                outputs = model(**inputs)
            
            actions = processor.decode_action(outputs.actions, robot_type="libero_all").squeeze(0).cpu().numpy()

            # --- D. 动作执行 (执行前 3 个预测步) ---
            print(f"🚀 执行模型预测的前 3 步动作...")
            for step in range(3):
                action = actions[step]
                # 严格按照要求：提取前三个坐标 X, Y, Z
                target_x, target_y, target_z = action[0], action[1], action[2]
                
                # 其他参数使用模型预测或安全默认值
                target_phi = action[4]   # 模型预测的 Pitch
                target_raw = action[5]   # 模型预测的 Wrist Raw
                gripper_val = action[6]  # 夹爪

                print(f"  Step {step+1} -> 目标: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
                
                # 执行移动
                move_to_target(
                    [target_x, target_y, target_z, target_phi, target_raw, gripper_val],
                    ser, mt, dt, hand
                )

            # 询问或延时，防止连续运动失控
            # time.sleep(1) 

    except KeyboardInterrupt:
        print("\n🛑 紧急停止...")
    finally:
        # 安全卸荷
        for m in mt:
            m.mode, m.kp, m.kd, m.tau = 0, 0, 0, 0
            ser.sendRecv(m, MotorData())
        hand.shutdown()
        print("✅ 硬件已安全关闭")

if __name__ == "__main__":
    main()