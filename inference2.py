import os, time
import cv2
import numpy as np
import torch

os.environ.setdefault("HF_ENDPOINT", "https://hf-mirror.com")
for _k in list(os.environ):
    if _k.lower().endswith("_proxy"):
        del os.environ[_k]

from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.factory import make_pre_post_processors
from driver import RobotArm
from control import CameraThread, format_image

FOLLOWER_MOTOR_PORT = "/dev/ttyUSB1"
FOLLOWER_SERVO_PORT = "/dev/ttyUSB5"

POLICY_PATH = "/home/hzy/mybot/output_lerobot_train/smolvla_cocacola/checkpoints/010000/pretrained_model"
TASK = "Pick up the yellow block."
FPS = 30
SAFE_POSE = (0.2, 0.0, 0.045)

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def img_to_tensor(frame_bgr: np.ndarray) -> torch.Tensor:
    chw = format_image(cv2.resize(frame_bgr, (320, 240)))
    t = torch.from_numpy(chw.copy()).float().div_(255.0)
    return t


def run_inference():
    print("=== SmolVLA Inference Deployment ===")
    print(f"Device: {DEVICE}")
    print(f"Policy: {POLICY_PATH}")
    print(f"Task  : {TASK}")
    print("====================================")

    print("Loading SmolVLA policy...")
    policy = SmolVLAPolicy.from_pretrained(POLICY_PATH)
    policy.to(DEVICE)
    policy.eval()
    policy.reset()
    print("Policy loaded.")

    print("Loading preprocessor & postprocessor...")
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy.config,
        pretrained_path=POLICY_PATH,
        preprocessor_overrides={"device_processor": {"device": str(DEVICE)}},
    )
    print("Preprocessor & postprocessor loaded.")

    print("Opening cameras (320x240 @ 30)...")
    cam_global = CameraThread(0, width=320, height=240, fps=30)
    cam_wrist = CameraThread(2, width=320, height=240, fps=30)
    time.sleep(0.5)

    ret1, _ = cam_global.read()
    ret2, _ = cam_wrist.read()
    if not ret1 or not ret2:
        print("摄像头初始化失败，请检查设备号。")
        cam_global.release()
        cam_wrist.release()
        return

    follower = None
    try:
        print("Opening follower arm...")
        follower = RobotArm(FOLLOWER_MOTOR_PORT, FOLLOWER_SERVO_PORT)
        follower.__enter__()
        follower.enable(1, 0.1)
        print("Follower arm enabled.")

        print(f"Moving follower to safe pose {SAFE_POSE}...")
        follower.move_to(*SAFE_POSE)
        time.sleep(1.0)
        print("推理开始。按 [ESC] 安全退出。")

        freq_count = 0
        freq_t0 = time.perf_counter()
        freq_hz = 0.0

        while True:
            loop_start = time.perf_counter()

            ret1, frame_global = cam_global.read()
            ret2, frame_wrist = cam_wrist.read()
            if not ret1 or not ret2:
                time.sleep(0.005)
                continue

            arm_state = follower.read()
            try:
                state6 = np.array([
                    arm_state.x, arm_state.y, arm_state.z,
                    arm_state.phi, arm_state.roll, arm_state.gripper,
                ], dtype=np.float32)
                if state6.shape != (6,) or np.isnan(state6).any() or np.isinf(state6).any():
                    raise ValueError(f"invalid state {state6}")
            except Exception as e:
                print(f"\r[硬件警告] 跳过当前帧: {e}{' '*10}", end="")
                continue

            obs = {
                "observation.images.global": img_to_tensor(frame_global),
                "observation.images.wrist": img_to_tensor(frame_wrist),
                "observation.state": torch.from_numpy(state6),
                "task": TASK,
            }

            with torch.inference_mode():
                obs = preprocessor(obs)
                action = policy.select_action(obs)
                action = postprocessor(action)

            a = action.squeeze(0).detach().cpu().numpy().astype(np.float32)
            if a.shape != (6,) or np.isnan(a).any() or np.isinf(a).any():
                print(f"\r[策略警告] 非法动作，跳过: {a}{' '*10}", end="")
                continue

            try:
                follower.follow(
                    x=float(a[0]), y=float(a[1]), z=float(a[2]),
                    phi=float(a[3]), roll=float(a[4]), gripper=float(a[5]),
                )
            except ValueError:
                pass

            freq_count += 1
            now = time.perf_counter()
            if now - freq_t0 >= 1.0:
                freq_hz = freq_count / (now - freq_t0)
                freq_count = 0
                freq_t0 = now

            print(
                f"\r推理中 | a=[{a[0]:+.3f},{a[1]:+.3f},{a[2]:+.3f},{a[3]:+.2f},{a[4]:+.2f},{a[5]:+.2f}] "
                f"| {freq_hz:.0f} Hz",
                end="", flush=True,
            )

            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.0, (1.0 / FPS) - elapsed))

    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C，正在清理...")
    finally:
        print("\n正在清理...")
        cam_global.release()
        cam_wrist.release()
        cv2.destroyAllWindows()

        if follower is not None:
            try:
                follower.move_to(*SAFE_POSE)
            except Exception as e:
                print(f"回安全位失败: {e}")
            try:
                follower.disable()
            except Exception:
                pass
            try:
                follower.__exit__(None, None, None)
            except Exception:
                pass
        print("Cleaned up.")


if __name__ == "__main__":
    run_inference()
