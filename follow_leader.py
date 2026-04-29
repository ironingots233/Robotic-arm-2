import time
import math
import threading
import scservo_sdk as scs
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


def main():
    print("=== Leader-Follower Arm Control ===")
    print(f"Leader  port: {LEADER_PORT}")
    print(f"Follower motor port: {FOLLOWER_MOTOR_PORT}")
    print(f"Follower servo port: {FOLLOWER_SERVO_PORT}")

    leader = LeaderArm()
    print("Opening leader arm...")
    leader.open()
    print("Leader arm connected.")

    print("Opening follower arm...")
    follower = RobotArm(FOLLOWER_MOTOR_PORT, FOLLOWER_SERVO_PORT)
    follower.__enter__()
    follower.enable(1,0.1)
    print("Follower arm enabled.")

    leader.start_reading()
    print("Reading leader arm state...")

    print("\nWaiting for first valid reading from leader arm...")
    state = None
    while state is None:
        state = leader.read_state()



    print(f"Leader initial position: x={state['x']:.3f}, y={state['y']:.3f}, z={state['z']:.3f}, "
          f"phi={state['phi']:.2f}, roll={state['roll']:.2f}, gripper={state['gripper']:.2f}")

    print(f"\nMoving follower to leader position...")
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

    print("Initial move complete. Entering follow mode (Ctrl+C to stop)...\n")

    freq_count = 0
    freq_t0 = time.perf_counter()
    freq_hz = 0.0

    try:
        while True:
            state = leader.read_state()
            if state is None:
                time.sleep(0.005)
                continue

            try:
                follower.follow(
                    x=state["x"],
                    y=state["y"],
                    z=state["z"],
                    phi=state["phi"],
                    roll=state["roll"],
                    gripper=state["gripper"],
                )
            except ValueError:
                pass

            freq_count += 1
            now = time.perf_counter()
            dt = now - freq_t0
            if dt >= 1.0:
                freq_hz = freq_count / dt
                freq_count = 0
                freq_t0 = now

            print(f"\rFollowing: x={state['x']:.3f}, y={state['y']:.3f}, z={state['z']:.3f}, "
                  f"phi={state['phi']:.2f}, roll={state['roll']:.2f}, grip={state['gripper']:.2f} "
                  f"| {freq_hz:.0f} Hz",
                  end="", flush=True)

    except KeyboardInterrupt:
        print("\n\nStopped by user.")
    finally:
        leader.stop_reading()
        leader.release_servos()
        leader.close()
        follower.disable()
        follower.__exit__(None, None, None)
        print("Cleaned up.")


if __name__ == "__main__":
    main()
