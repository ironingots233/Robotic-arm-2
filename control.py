import time
import math
import cv2
import numpy as np
import threading
import json
import scservo_sdk as scs
from pathlib import Path
from queue import Queue
from lerobot.datasets import LeRobotDataset
from driver import RobotArm

LEADER_PORT = "/dev/ttyUSB4"
LEADER_BAUDRATE = 1000000
LEADER_SERVO_IDS = [1, 2, 3, 4, 5, 6]
LEADER_ZERO_OFFSETS = {1: 331.2, 2: 148, 3: 56, 4: -30, 5: 71.6, 6: -48.3}
LEADER_DIRECTIONS = {1: 1, 2: -1, 3: -1, 4: -1, 5: 1, 6: 1}
LEADER_Z_OFFSET = 0.1
ADDR_PRESENT_POS = 56
LEN_PRESENT_POS = 2
ADDR_TORQUE_ENABLE = 40
PROTOCOL_VERSION = 0

L1 = 0.25424
L2 = 0.25424
L3 = 0.195

FOLLOWER_MOTOR_PORT = "/dev/ttyUSB1"
FOLLOWER_SERVO_PORT = "/dev/ttyUSB5"


def leader_forward_kinematics(q0, q1, q2, q3):
    th1 = q1
    th2 = q1 + q2
    th3 = th2 + q3
    c1, s1 = math.cos(th1), math.sin(th1)
    c2, s2 = math.cos(th2), math.sin(th2)
    c3, s3 = math.cos(th3), math.sin(th3)
    r = L1 * c1 + L2 * c2 + L3 * c3
    z = L1 * s1 + L2 * s2 + L3 * s3
    return r * math.cos(q0), -r * math.sin(q0), z


def patch_setPacketTimeout(self, packet_length):
    self.packet_start_time = self.getCurrentTime()
    self.packet_timeout = (self.tx_time_per_byte * packet_length) + (self.tx_time_per_byte * 3.0) + 50


class LeaderArm:
    def __init__(self):
        self.port_handler = scs.PortHandler(LEADER_PORT)
        self.port_handler.setPacketTimeout = patch_setPacketTimeout.__get__(
            self.port_handler, scs.PortHandler
        )
        self.packet_handler = scs.PacketHandler(PROTOCOL_VERSION)
        self.group_sync_read = scs.GroupSyncRead(
            self.port_handler, self.packet_handler, ADDR_PRESENT_POS, LEN_PRESENT_POS
        )
        for sid in LEADER_SERVO_IDS:
            self.group_sync_read.addParam(sid)
        self._lock = threading.Lock()
        self._state = None
        self._running = False

    def open(self):
        if not self.port_handler.setBaudRate(LEADER_BAUDRATE):
            raise RuntimeError(f"Failed to set baudrate to {LEADER_BAUDRATE}")
        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port {LEADER_PORT}")

    def release_servos(self):
        for sid in LEADER_SERVO_IDS:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, sid, ADDR_TORQUE_ENABLE, 0
            )
        print("Leader servos released (torque disabled).")

    def close(self):
        self.port_handler.closePort()

    def read_state(self):
        with self._lock:
            return self._state

    def start_reading(self):
        self._running = True
        t = threading.Thread(target=self._read_loop, daemon=True)
        t.start()

    def stop_reading(self):
        self._running = False

    def _read_loop(self):
        while self._running:
            comm_result = self.group_sync_read.txRxPacket()
            if comm_result != scs.COMM_SUCCESS:
                self.port_handler.clearPort()
                time.sleep(0.001)
                continue

            positions = []
            for sid in LEADER_SERVO_IDS:
                if self.group_sync_read.isAvailable(sid, ADDR_PRESENT_POS, LEN_PRESENT_POS):
                    raw = self.group_sync_read.getData(sid, ADDR_PRESENT_POS, LEN_PRESENT_POS)
                    positions.append(raw)
                else:
                    positions.append(-1)

            joint_rads = []
            for sid, p in zip(LEADER_SERVO_IDS, positions):
                deg = (p * 360.0 / 4096 - LEADER_ZERO_OFFSETS[sid]) * LEADER_DIRECTIONS[sid]
                deg = (deg + 180) % 360 - 180
                joint_rads.append(math.radians(deg))

            q0, q1, q2, q3 = joint_rads[0], joint_rads[1], joint_rads[2], joint_rads[3]
            roll = joint_rads[4]
            gripper_angle = joint_rads[5]

            x, y, z = leader_forward_kinematics(q0, q1, q2, q3)
            phi = q1 + q2 + q3

            with self._lock:
                self._state = {
                    "x": x,
                    "y": y,
                    "z": z - LEADER_Z_OFFSET,
                    "phi": phi,
                    "roll": roll,
                    "gripper": gripper_angle,
                }


class CameraThread:
    def __init__(self, src, width=320, height=240, fps=20):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret = False
        self.frame = None
        self.stopped = False
        self.lock = threading.Lock()
        threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            with self.lock:
                self.ret, self.frame = ret, frame
            time.sleep(0.001)

    def read(self):
        with self.lock:
            if self.ret:
                return True, self.frame.copy()
            else:
                return False, None

    def release(self):
        self.stopped = True
        self.cap.release()


def format_image(frame):
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return np.transpose(img_rgb, (2, 0, 1))


def teleop_and_record():
    print("=== Leader-Follower Teleop + Data Recording ===")
    print(f"Leader  port: {LEADER_PORT}")
    print(f"Follower motor port: {FOLLOWER_MOTOR_PORT}")
    print(f"Follower servo port: {FOLLOWER_SERVO_PORT}")
    print("=========================================================")
    print(" 🔴 [R] 键 : 开始 / 停止录制当前回合")
    print(" 🛑 [ESC]  : 安全退出并保存所有数据")
    print("=========================================================")

    print("正在初始化摄像头 (320x240 )...")
    cam_global = CameraThread(0, width=320, height=240, fps=30)
    cam_wrist = CameraThread(2, width=320, height=240, fps=30)

    time.sleep(0.5)
    ret1, _ = cam_global.read()
    ret2, _ = cam_wrist.read()
    if not ret1 or not ret2:
        print("❌ 错误：无法同时打开两个摄像头！请检查设备号。")
        return

    features = {
        "observation.images.global": {"dtype": "image", "shape": (3, 240, 320), "names": ["channels", "height", "width"]},
        "observation.images.wrist": {"dtype": "image", "shape": (3, 240, 320), "names": ["channels", "height", "width"]},
        "observation.state": {
            "dtype": "float32",
            "shape": (6,),
            "names": ["x", "y", "z", "phi", "raw", "gripper"]
        },
        "action": {
            "dtype": "float32",
            "shape": (6,),
            "names": ["x", "y", "z", "phi", "raw", "gripper"]
        }
    }

    DATASET_ROOT = "/home/hzy/mybot/data/cocacola"
    DATASET_REPO_ID = "my_dualcam_6dof_dataset"

    dataset_info_path = Path(DATASET_ROOT) / "meta" / "info.json"
    if dataset_info_path.exists():
        print(f"📂 检测到已有数据集 ({DATASET_ROOT})，将续录新回合...")
        dataset = LeRobotDataset.resume(
            repo_id=DATASET_REPO_ID,
            root=DATASET_ROOT,
            streaming_encoding=True,
            encoder_threads=2,
        )
        episode_count = dataset.meta.total_episodes
        print(f"   已有 {episode_count} 个回合，将从第 {episode_count + 1} 回合开始录制。")
    else:
        print("🆕 未检测到已有数据集，将创建新数据集...")
        dataset = LeRobotDataset.create(
            repo_id=DATASET_REPO_ID,
            fps=30,
            features=features,
            use_videos=True,
            streaming_encoding=True,
            encoder_threads=2,
            root=DATASET_ROOT,
        )
        episode_count = 0

    frame_queue = Queue(maxsize=0)

    def writer_worker():
        while True:
            item = frame_queue.get()

            try:
                if item is None:
                    break

                if "command" in item:
                    command = item["command"]
                    if command == "save":
                        try:
                            dataset.save_episode()
                            print("\n✅ [后台] 当前回合已成功存入硬盘，内存已释放！")
                        except Exception as e:
                            print(f"\n❌ [后台] 保存回合失败: {e}")
                    continue

                raw_global = item.pop("global_raw")
                raw_wrist = item.pop("wrist_raw")

                global_img = format_image(cv2.resize(raw_global, (320, 240)))
                wrist_img = format_image(cv2.resize(raw_wrist, (320, 240)))

                item["observation.images.global"] = global_img
                item["observation.images.wrist"] = wrist_img

                dataset.add_frame(item)

            except Exception as e:
                print(f"\n❌ [后台写入错误] 丢弃当前帧: {e}")

            finally:
                frame_queue.task_done()

    writer_thread = threading.Thread(target=writer_worker, daemon=True)
    writer_thread.start()

    try:
        print("Opening leader arm...")
        leader = LeaderArm()
        leader.open()
        print("Leader arm connected.")

        print("Opening follower arm...")
        follower = RobotArm(FOLLOWER_MOTOR_PORT, FOLLOWER_SERVO_PORT)
        follower.__enter__()
        follower.enable(1, 0.1)
        print("Follower arm enabled.")

        leader.start_reading()
        print("Reading leader arm state...")

        print("\nWaiting for first valid reading from leader arm...")
        state = None
        while state is None:
            state = leader.read_state()
            time.sleep(0.05)

        print(f"Leader initial position: x={state['x']:.3f}, y={state['y']:.3f}, z={state['z']:.3f}, "
              f"phi={state['phi']:.2f}, roll={state['roll']:.2f}, gripper={state['gripper']:.2f}")

        print("Moving follower to leader position...")
        try:
            follower.move_to(
                x=state["x"],
                y=state["y"],
                z=state["z"],
                phi=state["phi"],
                roll=state["roll"],
                gripper=state["gripper"],
            )
        except ValueError as e:
            print(f"IK error during initial move: {e}")
            print("Trying with phi=-0.5 as fallback...")
            follower.move_to(
                x=state["x"],
                y=state["y"],
                z=state["z"],
                phi=-0.5,
                roll=state["roll"],
                gripper=state["gripper"],
            )

        print("Initial move complete. Entering follow mode...\n")

        import keyboard

        r_key_pressed = False
        is_recording = False
        current_episode_frames = 0
        frame_counter = 0
        total_loop_time = 0.0

        freq_count = 0
        freq_t0 = time.perf_counter()
        freq_hz = 0.0

        print("🚀 系统就绪！移动主臂控制从臂，按 [R] 开始录制。")

        while True:
            loop_start = time.perf_counter()

            ret1, frame_global = cam_global.read()
            ret2, frame_wrist = cam_wrist.read()
            if not ret1 or not ret2:
                continue

            state = leader.read_state()
            if state is None:
                time.sleep(0.005)
                continue

            cur_x = state["x"]
            cur_y = state["y"]
            cur_z = state["z"]
            cur_phi = state["phi"]
            cur_raw = state["roll"]
            cur_gripper = state["gripper"]

            try:
                follower.follow(
                    x=cur_x,
                    y=cur_y,
                    z=cur_z,
                    phi=cur_phi,
                    roll=cur_raw,
                    gripper=cur_gripper,
                )
            except ValueError:
                pass

            if keyboard.is_pressed('r'):
                if not r_key_pressed:
                    r_key_pressed = True
                    if not is_recording:
                        frame_queue.put({"command": "start"})
                        is_recording = True
                        current_episode_frames = 0
                        print(f"\n🔴 开始录制 第 {episode_count + 1} 回合...")
                    else:
                        is_recording = False
                        episode_count += 1
                        frame_queue.put({"command": "save"})
                        print(f"\n⏳ 正在后台保存 第 {episode_count} 回合，您可继续操作...")
            else:
                r_key_pressed = False

            if keyboard.is_pressed('esc'):
                print("\n接收到退出指令...")
                break

            if is_recording:
                arm_state = follower.read()

                try:
                    current_state = np.array([
                        arm_state.x, arm_state.y, arm_state.z,
                        arm_state.phi, arm_state.roll, arm_state.gripper
                    ], dtype=np.float32)

                    if current_state.shape != (6,):
                        raise ValueError(f"数据长度不为6，实际形状: {current_state.shape}")

                    if np.isnan(current_state).any() or np.isinf(current_state).any():
                        raise ValueError(f"数据中包含 NaN 或 Inf: {current_state}")

                except Exception as e:
                    print(f"\r⚠️ [硬件警告] 丢包/异常: {e} | 已跳过当前帧{' '*10}", end="")
                    continue

                target_action = np.array([cur_x, cur_y, cur_z, cur_phi, cur_raw, cur_gripper], dtype=np.float32)

                frame_queue.put({
                    "global_raw": frame_global,
                    "wrist_raw": frame_wrist,
                    "observation.state": current_state,
                    "action": target_action,
                    "task": "Pick up the block and put it in the box."
                })
                current_episode_frames += 1

            freq_count += 1
            now = time.perf_counter()
            dt = now - freq_t0
            if dt >= 1.0:
                freq_hz = freq_count / dt
                freq_count = 0
                freq_t0 = now

            if is_recording:
                print(f"\r录制中... 帧数: {current_episode_frames} | 队列: {frame_queue.qsize()} | "
                      f"x={cur_x:.3f}, y={cur_y:.3f}, z={cur_z:.3f} | {freq_hz:.0f} Hz", end="")
            else:
                print(f"\r跟随中... x={cur_x:.3f}, y={cur_y:.3f}, z={cur_z:.3f}, "
                      f"phi={cur_phi:.2f}, roll={cur_raw:.2f}, grip={cur_gripper:.2f} | {freq_hz:.0f} Hz",
                      end="", flush=True)

            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0, (1.0 / 30.0) - elapsed))

            actual_elapsed = time.perf_counter() - loop_start
            total_loop_time += actual_elapsed
            frame_counter += 1
            if frame_counter % 90 == 0:
                avg_ms = (total_loop_time / 90) * 1000
                print(f"\n[性能] 平均帧耗时: {avg_ms:.2f} ms")
                total_loop_time = 0.0

    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C 强制打断")
    finally:
        print("\n正在保存数据并清理...")
        if is_recording:
            frame_queue.put({"command": "save"})

        frame_queue.put(None)
        frame_queue.join()
        writer_thread.join(timeout=10.0)

        cam_global.release()
        cam_wrist.release()
        cv2.destroyAllWindows()

        try:
            print("\n📦 正在生成数据集统计信息 (Consolidate)...")
            dataset.consolidate()

            print("🔧 正在检查并注入图像归一化参数 (ImageNet Stats)...")
            stats_path = Path(dataset.root) / "meta" / "stats.json"

            if stats_path.exists():
                with open(stats_path, "r") as f:
                    stats = json.load(f)

                dummy_image_stats = {
                    "mean": [0.485, 0.456, 0.406],
                    "std": [0.229, 0.224, 0.225]
                }
                keys_to_add = ["observation.images.global", "observation.images.wrist"]
                fixed = False

                for key in keys_to_add:
                    if key not in stats:
                        stats[key] = dummy_image_stats
                        fixed = True

                if fixed:
                    with open(stats_path, "w") as f:
                        json.dump(stats, f, indent=4)
                    print("✅ 缺失的图像 Key 修复成功！无需再手动运行 key.py。")
                else:
                    print("✅ 统计信息完整，无需修复。")
        except Exception as e:
            print(f"⚠️ 生成或修复 stats.json 时发生错误: {e}")

        print("🎉 数据集处理完毕！")

        leader.stop_reading()
        leader.release_servos()
        leader.close()
        follower.move_to(0.2, 0, 0.045)
        follower.disable()
        follower.__exit__(None, None, None)
        print("Cleaned up.")


if __name__ == "__main__":
    teleop_and_record()
