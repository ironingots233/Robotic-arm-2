import cv2
import torch
import numpy as np
from PIL import Image
from transformers import AutoModel, AutoProcessor

# 1. 初始化模型
print("1. 正在加载模型与处理器...")
model_path = "XiaomiRobotics/Xiaomi-Robotics-0-LIBERO"
model = AutoModel.from_pretrained(
    model_path, 
    trust_remote_code=True, 
    attn_implementation="eager", 
    load_in_8bit=True,
    device_map="cuda:0"
).eval()

processor = AutoProcessor.from_pretrained(model_path, trust_remote_code=True, use_fast=False)

print("2. 正在准备输入数据...")
# 构造指令: "拿起黑色的胶卷"
language_instruction = "Pick up the black film roll."
instruction = (
    f"<|im_start|>user\nThe following observations are captured from multiple views.\n"
    f"# Base View\n<|vision_start|><|image_pad|><|vision_end|>\n"
    f"# Left-Wrist View\n<|vision_start|><|image_pad|><|vision_end|>\n"
    f"Generate robot actions for the task:\n{language_instruction} /no_cot<|im_end|>\n"
    f"<|im_start|>assistant\n<cot></cot><|im_end|>\n"
)

# ================= 新增：双摄像头读取逻辑 =================
print("📸 正在尝试打开双摄像头...")
cap_base = cv2.VideoCapture(2)   # 全局底座摄像头
cap_wrist = cv2.VideoCapture(0)  # 腕部摄像头

# 处理全局底座视角 (Base View)
if not cap_base.isOpened():
    print("❌ 无法打开全局摄像头 (ID:1)！该视角退回使用全黑测试图。")
    image_base = Image.new('RGB', (224, 224), color='black')
else:
    ret_base, frame_base = cap_base.read()
    cap_base.release() # 获取完一帧后释放
    if ret_base:
        print("✅ 成功获取真实的全局摄像头画面！")
        frame_base_rgb = cv2.cvtColor(frame_base, cv2.COLOR_BGR2RGB)
        image_base = Image.fromarray(frame_base_rgb).resize((224, 224))
    else:
        print("❌ 无法读取全局画面帧！该视角退回使用全黑测试图。")
        image_base = Image.new('RGB', (224, 224), color='black')

# 处理腕部视角 (Wrist View)
if not cap_wrist.isOpened():
    print("❌ 无法打开腕部摄像头 (ID:0)！该视角退回使用全黑测试图。")
    image_wrist = Image.new('RGB', (224, 224), color='black')
else:
    ret_wrist, frame_wrist = cap_wrist.read()
    cap_wrist.release() # 获取完一帧后释放
    if ret_wrist:
        print("✅ 成功获取真实的腕部摄像头画面！")
        frame_wrist_rgb = cv2.cvtColor(frame_wrist, cv2.COLOR_BGR2RGB)
        image_wrist = Image.fromarray(frame_wrist_rgb).resize((224, 224))
    else:
        print("❌ 无法读取腕部画面帧！该视角退回使用全黑测试图。")
        image_wrist = Image.new('RGB', (224, 224), color='black')
# ==========================================================

# ================= 当前机器人物理状态数据 =================
# 当前末端空间座标 (单位通常是米)
current_x, current_y, current_z = 0.0, 0.0, 0.0
# 当前末端旋转姿态 (欧拉角，单位通常是弧度)
current_roll, current_pitch, current_yaw = 0.0, 0.0, 0.0
# 当前夹爪状态 (比如 1.0 表示完全张开，-1.0 或 0.0 表示完全闭合)
current_gripper = 1.0 

state_vector = np.zeros(32)
state_vector[0] = current_x
state_vector[1] = current_y
state_vector[2] = current_z
state_vector[3] = current_roll
state_vector[4] = current_pitch
state_vector[5] = current_yaw
state_vector[6] = current_gripper

# 机器人的当前姿态数据
proprio_state = state_vector.reshape(1, 1, 32)
# ==========================================================

# 将文本、图片和本体状态交给模型处理
inputs = processor(
    text=[instruction],
    images=[image_base, image_wrist],
    videos=None,
    padding=True,
    return_tensors="pt",
).to(model.device)

robot_type = "libero_all"
inputs["seed"] = 42
inputs["state"] = torch.from_numpy(proprio_state).to(model.device, model.dtype).view(1, 1, -1)
inputs["action_mask"] = processor.get_action_mask(robot_type).to(model.device, model.dtype)

print("3. 模型开始思考并生成动作...")
with torch.no_grad():
    outputs = model(**inputs)

# 翻译模型的输出为真实的机器人控制指令
action_chunk = processor.decode_action(outputs.actions, robot_type=robot_type)
print(f"✅ 成功！生成的机器人动作指令数据形状为: {action_chunk.shape}")

actions_numpy = action_chunk.squeeze(0).cpu().numpy()  # 形状变为 [10, 32]

print("开始解析未来 10 步的动作轨迹：\n" + "-"*40)

# 遍历这 10 个连续动作
for step in range(len(actions_numpy)):
    action = actions_numpy[step]
    
    # 提取空间位置座标 (X, Y, Z)
    pos_x, pos_y, pos_z = action[0], action[1], action[2]
    # 提取末端执行器姿态 (欧拉角: Roll, Pitch, Yaw)
    roll, pitch, yaw = action[3], action[4], action[5]
    # 提取夹爪开合状态 
    gripper_state = action[6]
    
    print(f"Step {step+1}:")
    print(f"  👉 位置座标 (X,Y,Z): ({pos_x:.4f}, {pos_y:.4f}, {pos_z:.4f})")
    print(f"  👉 旋转姿态 (R,P,Y): ({roll:.4f}, {pitch:.4f}, {yaw:.4f})")
    print(f"  👉 夹爪状态: {gripper_state:.4f}")