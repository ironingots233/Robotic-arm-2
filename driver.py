import serial
import struct
import math
import time
import threading
from dataclasses import dataclass
from typing import Optional, Tuple

@dataclass 
class JointConfig:
    '''单个关节电机的机械配置'''
    motor_id: int                     # 电机ID
    direction: int                    # 反转，+1 或 -1
    offset: float                     # 零点偏移量 (弧度)
    limits: Tuple[float, float]       # 物理限位 (最小, 最大) 弧度
    gravity: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # 重力补偿系数
 
@dataclass
class ArmState:
    """机械臂的全局实时状态"""
    x: float = 0.0                          # 夹爪末端的笛卡尔坐标系
    y: float = 0.0          
    z: float = 0.0
    phi: float = 0.0                        # 夹爪俯仰角
    roll: float = 0.0                       # 夹爪旋转角
    gripper: float = 0.0                    # 夹爪开合程度
    joints: Tuple[float, ...] = ()          # 各个关节当前的真是角度
    velocities: Tuple[float, ...] = ()      # 各个关节当前的转速
    torques: Tuple[float, ...] = ()         # 各个关节当前的扭矩
    temps: Tuple[int, ...] = ()             # 各个电机的当前温度

# 机械臂物理参数
DEFAULT_JOINTS = ( 
    JointConfig(0,  1,  0.68,                   limits=(-1.57,  1.57)),
    JointConfig(1, -1,  0.926 - 0.1829 + 3.14,  limits=(-0.128, 2.98),  gravity=(4.1740, -0.0525,  0.0245)),
    JointConfig(2,  1,  3.162 - 0.1829,         limits=(-2.93,  0.0),   gravity=(2.2243, -0.4785,  0.0394)),
    JointConfig(3, -1,  0,                      limits=(-2.3,   2.1),   gravity=(0.1385,  0.0526, -0.0098)),
)

class _MotorBus:
    '''关节电机通信类'''
    GEAR = 6.33                 # 电机减速比
    _HEADER = b'\xFE\xEE'       # 指令包头，必须以 FE EE 开头
    _RX_HEADER = b'\xFD\xEE'    # 回传包头，必定以 FD EE 开头
    
    # struct 打包，与 C 语言底层通信
    _STRUCT_MOTOR_TX = struct.Struct('<hhiHH')
    _STRUCT_CRC = struct.Struct('<H')
    _STRUCT_MOTOR_RX = struct.Struct('<hhib')

    # 初始化串口
    def __init__(self, port: str, baudrate: int = 4000000):
        self._ser = serial.Serial(port, baudrate, timeout=0.005)

    # 一次完整的收发任务
    def transact(self, motor_id: int, mode: int,
                 pos: float, vel: float, tau: float,
                 kp: float, kd: float) -> Optional[dict]:
        
        # 限制参数范围（见宇树文档）
        tau_c = max(-127.99, min(127.99, tau / self.GEAR))
        kp_c = max(0.0, min(25.599, kp))
        kd_c = max(0.0, min(25.599, kd))

        # 将弧度转换为电机底层的刻度值
        pos_int = int(pos * self.GEAR / (2 * math.pi) * 32768)
        vel_int = int(vel * self.GEAR / (2 * math.pi) * 256)
        tau_int = int(tau_c * 256)
        kp_int = int(kp_c * 1280)
        kd_int = int(kd_c * 1280)

        # 组装数据包
        id_mode_byte = (motor_id & 0x0F) | ((mode & 0x07) << 4)
        payload = self._STRUCT_MOTOR_TX.pack(tau_int, vel_int, pos_int, kp_int, kd_int)
        
        # 拼接：帧头 + ID/模式 + 载荷数据
        frame = bytearray(self._HEADER)
        frame.append(id_mode_byte)
        frame.extend(payload)
        
        # 计算并追加 CRC 校验码
        frame.extend(self._STRUCT_CRC.pack(_crc16(frame)))

        # 每次发送前清空接收缓存
        self._ser.reset_input_buffer()
        self._ser.write(frame)

        # 接收与解析数据
        rx = self._ser.read(16)
        
        # 长度不够或帧头不对，直接认定失败返回 None
        if len(rx) == 16 and rx[:2] == self._RX_HEADER:
            # 校验 CRC
            if self._STRUCT_CRC.unpack_from(rx, 14)[0] == _crc16(rx[:14]):
                # 校验电机 ID
                mid = rx[2] & 0x0F
                if mid == motor_id:
                    # 解包并转换回物理单位
                    t, w, p, temp = self._STRUCT_MOTOR_RX.unpack_from(rx, 3)
                    return {
                        'id':   mid,
                        'pos':  p / 32768.0 * (2 * math.pi) / self.GEAR,
                        'vel':  w / 256.0 * (2 * math.pi) / self.GEAR,
                        'tau':  t / 256.0 * self.GEAR,
                        'temp': temp,
                    }
        return None

    # 
    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()

class _ServoBus:
    '''舵机通信类'''
    _HEADER = b'\xFF\xFF'

    def __init__(self, port: str, baudrate: int = 1000000):
        '''发起连接，初始化'''
        self._ser = serial.Serial(port, baudrate, timeout=0.01)
        self._pos_struct    = struct.Struct('<HHH')
        self._torque_struct = struct.Struct('<H')
        self._lock = threading.Lock()

    def write_pos(self, sid: int, pos: int, speed: int = 2000):
        '''控制位置与速度'''
        if pos < 0:    pos = 0
        elif pos > 4095: pos = 4095
        if speed < 0:    speed = 0
        elif speed > 3400: speed = 3400
        with self._lock:
            self._write(sid, 0x2A, self._pos_struct.pack(pos, 0, speed))
            self._ser.flush()
            self._drain_echo()

    def write_torque(self, sid: int, torque: int):
        '''控制力矩'''
        if torque < 0:     torque = 0
        elif torque > 1000: torque = 1000
        with self._lock:
            self._write(sid, 0x30, self._torque_struct.pack(torque))
            self._ser.flush()
            self._drain_echo()

    def read_pos(self, sid: int, retries: int = 2) -> Optional[int]:
        '''安全读取电机数据'''
        with self._lock:
            for attempt in range(retries + 1):
                val = self._read_pos_once(sid)
                if val is not None:
                    return val
                if attempt < retries:
                    time.sleep(0.001)
        return None

    def _read_pos_once(self, sid: int) -> Optional[int]:
        '''单次读取与数据包解析'''
        ser = self._ser
        ser.reset_input_buffer()
        self._write(sid, 0x38, b'\x02', instruction=0x02)
        ser.flush()
        resp = ser.read(8)
        if len(resp) < 8:
            return None
        buf = bytes(resp)
        if buf[:2] != self._HEADER:
            idx = buf.find(self._HEADER)
            if idx < 0 or idx + 8 > len(buf):
                extra = ser.read(8 - (len(buf) - idx) if idx >= 0 else 8)
                buf = (buf[idx:] if idx >= 0 else b'') + extra
                if len(buf) < 8 or buf[:2] != self._HEADER:
                    return None
            else:
                buf = buf[idx:idx + 8]
        if buf[2] != sid or buf[3] != 0x04:
            return None
        if buf[7] != (~sum(buf[2:7])) & 0xFF:
            return None
        if buf[4] != 0x00:
            return None
        p = buf[5] | (buf[6] << 8)
        return p - 65536 if p > 32767 else p

    def _drain_echo(self):
        '''排空回显'''
        n = self._ser.in_waiting
        if n:
            self._ser.read(n)

    def close(self):
        '''关闭串口'''
        if self._ser and self._ser.is_open:
            self._ser.close()

    def _write(self, sid, addr, data: bytes, instruction=0x03):
        '''底层发包协议封装'''
        length  = len(data) + 3
        payload = bytes([sid, length, instruction, addr]) + data
        self._ser.write(self._HEADER + payload + bytes([(~sum(payload)) & 0xFF]))

class _Gripper:
    '''末端执行器封装'''
    STEPS_PER_RAD = 4095 / (2 * math.pi)

    # 定义机械零点与限位
    def __init__(self, bus: _ServoBus,
                 gripper_id=2, wrist_id=1,
                 gripper_center=2975, gripper_range=(2975, 6000),
                 wrist_center=3400, wrist_range=(2600, 4200)):
        self._bus = bus
        self._gid, self._wid = gripper_id, wrist_id
        self._gcenter = gripper_center
        self._gmin, self._gmax = gripper_range
        self._wcenter = wrist_center
        self._wmin, self._wmax = wrist_range


        self._last_wrist_raw: Optional[int] = None
        self._last_grip_raw:  Optional[int] = None
        self._last_grip_speed: Optional[int] = None
        self._last_wrist_speed: Optional[int] = None

    def init_torque(self, val=300):
        '''使能力矩'''
        self._bus.write_torque(self._gid, val)
        time.sleep(0.002)
        self._bus.write_torque(self._wid, val)
        self._last_wrist_raw = None
        self._last_grip_raw  = None

    def disable_torque(self):
        '''释放力矩'''
        self._bus.write_torque(self._gid, 0)
        time.sleep(0.002)
        self._bus.write_torque(self._wid, 0)
        self._last_wrist_raw = None
        self._last_grip_raw  = None

    def set_gripper(self, angle: float, speed=1500):
        '''给夹爪下达指令'''
        raw = int(self._gcenter + angle * self.STEPS_PER_RAD)
        if raw < self._gmin:   raw = self._gmin
        elif raw > self._gmax: raw = self._gmax
        if raw == self._last_grip_raw and speed == self._last_grip_speed:
            return
        self._last_grip_raw = raw
        self._last_grip_speed = speed
        self._bus.write_pos(self._gid, raw, speed)

    def set_wrist(self, angle: float, speed=1500):
        '''给手腕下达指令'''
        raw = int(self._wcenter + angle * self.STEPS_PER_RAD)
        if raw < self._wmin:   raw = self._wmin
        elif raw > self._wmax: raw = self._wmax
        if raw == self._last_wrist_raw and speed == self._last_wrist_speed:
            return                             
        self._last_wrist_raw = raw
        self._last_wrist_speed = speed
        self._bus.write_pos(self._wid, raw, speed)

    def read_wrist(self) -> Optional[float]:
        '''读取腕部角度'''
        p = self._bus.read_pos(self._wid)
        return (p - self._wcenter) / self.STEPS_PER_RAD if p is not None else None

    def read_gripper(self) -> float:
        '''读取夹爪角度'''
        p = self._bus.read_pos(self._gid)
        return (p - self._gcenter) / self.STEPS_PER_RAD if p is not None else 0.0

def _crc16(data) -> int:
    '''校验码计算'''
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc

def _quintic_coeffs(qs, qe, T):
    '''计算轨迹曲线（五次多项式轨迹规划）'''
    h  = qe - qs
    T3 = T ** 3
    return (qs, 0.0, 0.0, 10 * h / T3, -15 * h / (T3 * T), 6 * h / (T3 * T * T))

def _eval_quintic(c, t):
    '''实时计算当前时刻的期望状态'''
    a0, a1, a2, a3, a4, a5 = c
    t2 = t  * t
    t3 = t2 * t
    t4 = t3 * t
    t5 = t4 * t
    q  = a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5
    dq = a1 + 2*a2*t + 3*a3*t2 + 4*a4*t3 + 5*a5*t4
    return q, dq

class RobotArm:
    '''机械臂类'''
    L1 = 0.25424   # 大臂长度 (m)
    L2 = 0.25424   # 小臂长度 (m)
    L3 = 0.195     # 夹爪长度 (m)

    def __init__(self, motor_port: str, servo_port: str,
                 joints: Tuple[JointConfig, ...] = DEFAULT_JOINTS):
        '''初始化'''
        self._joints = joints
        self._motor  = _MotorBus(motor_port)
        self._servo  = _ServoBus(servo_port)
        self._hand   = _Gripper(self._servo)

        self._mode    = [0]   * 4
        self._kp      = [0.0] * 4
        self._kd      = [0.0] * 4
        self._cmd_q   = [0.0] * 4
        self._cmd_dq  = [0.0] * 4
        self._cmd_tau = [0.0] * 4

        self._q    = [0.0] * 4
        self._dq   = [0.0] * 4
        self._tau  = [0.0] * 4
        self._temp = [0]   * 4

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.disable()
        self._motor.close()
        self._servo.close()

    def enable(self, kp: float = 1.0, kd: float = 0.1):
        """机械臂使能"""
        self._sync_all()
        for i in range(4):
            self._cmd_q[i] = self._q[i]
            self._mode[i]  = 1
            self._kp[i]    = kp
            self._kd[i]    = kd
        self._sync_all()
        self._hand.init_torque()

    def disable(self):
        """机械臂失能"""
        for i in range(4):
            self._cmd_q[i]   = self._q[i]
            self._mode[i]    = 0
            self._kp[i]      = 0.0
            self._kd[i]      = 0.0
            self._cmd_tau[i] = 0.0
        self._sync_all()
        self._hand.disable_torque()

    # 动作
    def move_to(self, x: float, y: float, z: float,
                phi: float = -0.5, roll: float = 0.0, gripper: float = 0.0,
                max_speed: float = 1.0, min_duration: float = 0.2):
        """移动到目标位置（阻塞）"""
        x, y, z, phi = self._clamp_coords(x, y, z, phi)
        q_end   = self.inverse_kinematics(x, y, z, phi)
        q_end = tuple(
            max(j.limits[0], min(j.limits[1], v))
            for j, v in zip(self._joints, q_end)
        )
        q_start = tuple(self._q)

        T = max(
            max(abs(e - s) for e, s in zip(q_end, q_start)) / max_speed,
            min_duration
        )
        coeffs = [_quintic_coeffs(s, e, T) for s, e in zip(q_start, q_end)]

        t0 = time.perf_counter()
        while True:
            t = time.perf_counter() - t0
            if t >= T:
                break
            for i, c in enumerate(coeffs):
                self._cmd_q[i], self._cmd_dq[i] = _eval_quintic(c, t)
            self._apply_gravity()
            self._sync_all()

        self._hand.set_gripper(gripper)
        self._hand.set_wrist(roll)

    def follow(self, x: float, y: float, z: float,
               phi: float = 0.0, roll: float = 0.0, gripper: float = 0.0):
        """跟随指定目标（非阻塞）"""
        x, y, z, phi = self._clamp_coords(x, y, z, phi)
        q = self.inverse_kinematics(x, y, z, phi)
        q = tuple(
            max(j.limits[0], min(j.limits[1], v))
            for j, v in zip(self._joints, q)
        )
        cmd_q = self._cmd_q
        cmd_q[0] = q[0]; cmd_q[1] = q[1]; cmd_q[2] = q[2]; cmd_q[3] = q[3]
        self._apply_gravity()

        def _send_servos():
            self._hand.set_wrist(roll, speed=3400)
            self._hand.set_gripper(gripper, speed=3400)

        t = threading.Thread(target=_send_servos, daemon=True)
        t.start()
        self._sync_all()
        t.join()

    def read(self) -> ArmState:
        """读取机械臂数据"""
        servo_result = [0.0, 0]

        def _read_servos():
            '''读取舵机数据'''
            w = self._hand.read_wrist()
            servo_result[0] = w if w is not None else 0.0
            servo_result[1] = self._hand.read_gripper()

        t = threading.Thread(target=_read_servos, daemon=True)
        t.start()
        self._sync_all()
        q = tuple(self._q)
        x, y, z = self.forward_kinematics(*q)

        t.join()

        return ArmState(
            x=x, y=y, z=z,
            phi=q[1] + q[2] + q[3],
            roll=servo_result[0],
            gripper=servo_result[1],
            joints=q,
            velocities=tuple(self._dq),
            torques=tuple(self._tau),
            temps=tuple(self._temp),
        )

    def set_gripper(self, angle: float, speed: int = 1500):
        """设置夹爪状态"""
        self._hand.set_gripper(angle, speed)

    def set_wrist(self, angle: float, speed: int = 1500):
        """设置手腕状态"""
        self._hand.set_wrist(angle, speed)

    def _clamp_coords(self, x: float, y: float, z: float, phi: float):
        r_xy = math.sqrt(x * x + y * y)
        if r_xy < 0.15:
            if r_xy < 1e-6:
                x, y = 0.15, 0.0
            else:
                scale = 0.15 / r_xy
                x *= scale
                y *= scale
        if z < -0.14:
            z = -0.14
        phi = max(-1.57, min(1.57, phi))
        return x, y, z, phi

    def forward_kinematics(self, q0, q1, q2, q3):
        "正运动学，输入电机角度返回坐标"
        th1 = q1
        th2 = q1 + q2
        th3 = th2 + q3
        c1, s1 = math.cos(th1), math.sin(th1)
        c2, s2 = math.cos(th2), math.sin(th2)
        c3, s3 = math.cos(th3), math.sin(th3)
        r = self.L1 * c1 + self.L2 * c2 + self.L3 * c3
        z = self.L1 * s1 + self.L2 * s2 + self.L3 * s3
        return r * math.cos(q0), r * math.sin(q0), z

    def inverse_kinematics(self, x, y, z, phi=0.0):
        '''逆运动学，输入坐标返回电机角度'''
        q0 = math.atan2(y, x)
        r  = math.sqrt(x * x + y * y)
        rw = r - self.L3 * math.cos(phi)
        zw = z - self.L3 * math.sin(phi)

        D_sq   = rw * rw + zw * zw
        cos_q2 = (D_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_q2 = max(-1.0, min(1.0, cos_q2))

        q2 = -math.acos(cos_q2)
        k1 = self.L1 + self.L2 * math.cos(q2)
        k2 = self.L2 * math.sin(q2)
        q1 = math.atan2(zw, rw) - math.atan2(k2, k1)
        q3 = phi - (q1 + q2)

        return tuple(
            max(j.limits[0], min(j.limits[1], v))
            for j, v in zip(self._joints, (q0, q1, q2, q3))
        )

    def _sync_all(self):
        """发包"""
        transact = self._motor.transact
        mode     = self._mode
        kp_list  = self._kp
        kd_list  = self._kd
        cmd_q    = self._cmd_q
        cmd_dq   = self._cmd_dq
        cmd_tau  = self._cmd_tau
        q        = self._q
        dq       = self._dq
        tau      = self._tau
        temp     = self._temp

        for i, j in enumerate(self._joints):
            direction = j.direction
            offset    = j.offset
            fb = transact(
                j.motor_id, mode[i],
                pos=cmd_q[i] * direction + offset,
                vel=cmd_dq[i] * direction,
                tau=cmd_tau[i] * direction,
                kp=kp_list[i], kd=kd_list[i],
            )
            if fb is not None:
                q[i]    = (fb['pos'] - offset) * direction
                dq[i]   = fb['vel']  * direction
                tau[i]  = fb['tau']  * direction
                temp[i] = fb['temp']

    def _apply_gravity(self):
        """计算重力补偿"""
        q1 = self._q[1]
        q12 = q1 + self._q[2]
        q123 = q12 + self._q[3]
        angles = (0.0, q1, q12, q123)
        tau = 0.0
        cmd_tau = self._cmd_tau
        joints = self._joints
        for i in range(3, 0, -1):
            A, B, C = joints[i].gravity
            a = angles[i]
            tau += A * math.cos(a) + B * math.sin(a) + C
            cmd_tau[i] = tau
        cmd_tau[0] = 0.0



if __name__ == "__main__":
    with RobotArm("/dev/ttyUSB1", "/dev/ttyUSB5") as arm:
        arm.enable(1,0.1)
        arm.move_to(0.2, 0, 0, phi=-0.5, roll = 0, gripper=1)
        try:
            while True:
                arm.set_gripper(1, speed=3400)
                time.sleep(1)
                arm.set_gripper(0, speed=3400)
                time.sleep(1)

        except KeyboardInterrupt:
            arm.move_to(0.15, 0, -0.076, phi=-0.77)