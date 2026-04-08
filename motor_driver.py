import serial
import struct
import math
import time
from dataclasses import dataclass

@dataclass
class MotorCmd:
    motorType: int = 1
    mode: int = 0
    id: int = 0
    kp: float = 0.0
    kd: float = 0.0   
    q: float = 0.0    
    dq: float = 0.0   
    tau: float = 0.0
    offset: float = 0.0     # 物理零点偏移量
    direction: int = 1      # 电机方向 (1 为正，-1 为反)

@dataclass
class MotorData:
    motorType: int = 1
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    temp: int = 0
    merror: int = 0

class SerialPort:
    # 初始化
    def __init__(self, port, baudrate=4000000):
        # 尝试连接串口
        try:
            self.serial = serial.Serial(port, baudrate, timeout=0.01) # 10ms超时
        except Exception as e:
            print(f"打开串口 {port} 失败: {e}")
            self.serial = None

    # 指令运行逻辑
    def sendRecv(self, cmd: MotorCmd, data: MotorData):
        if not self.serial or not self.serial.is_open:
            return False

        # 1. 应用方向和偏置
        raw_tau = cmd.tau * cmd.direction
        raw_omega = cmd.dq * cmd.direction
        raw_pos = (cmd.q * cmd.direction) + cmd.offset

        # 2. 打包字节流
        cmd_bytes = self._pack_motor_cmd(cmd.id, cmd.mode, raw_tau, raw_omega, raw_pos, cmd.kp, cmd.kd)
        
        # 3. 发送数据前，强制清空缓存
        self.serial.reset_input_buffer()
        self.serial.write(cmd_bytes)
        self.serial.flush() # 强制等待物理层写入完毕
        
        # 4. 智能非阻塞接收（解决 RS485回显 和 数据延迟串台错位问题）
        start_time = time.time()
        rx_buffer = bytearray()
        
        # 给通信 15ms 的宽限时间来寻找正确的反馈
        while time.time() - start_time < 0.015:
            n = self.serial.in_waiting
            if n > 0:
                rx_buffer.extend(self.serial.read(n))
                
                # 在缓存中寻找返回帧头 0xFD 0xEE 
                # (注意发送帧头是 0xFE 0xEE，代码不会误读自己的发送指令)
                idx = rx_buffer.find(b'\xFD\xEE')
                if idx != -1:
                    # 找到包头，检查是否已经凑够完整的 16 字节
                    if len(rx_buffer) >= idx + 16:
                        full_frame = bytes(rx_buffer[idx : idx+16])
                        feedback = self._parse_motor_feedback(full_frame)
                        
                        # 重新启用强 ID 校验！
                        # 能够彻底挡住上一轮由于延迟而“迟到”的其他电机数据
                        if feedback and feedback["id"] == cmd.id:
                            data.q = (feedback["pos"] - cmd.offset) * cmd.direction
                            data.dq = feedback["omega"] * cmd.direction
                            data.tau = feedback["tau"] * cmd.direction
                            data.temp = feedback["temp"]
                            return True
                        else:
                            # 找到了包头，但ID不匹配 (这就是你遇到的错位串台数据)
                            # 剔除这个错乱包的包头，继续在缓冲池中往后找真正的新数据
                            rx_buffer = rx_buffer[idx+2:]
            else:
                time.sleep(0.001) # 没数据时短暂让出CPU，防止死循环卡顿

        return False

    # 打包
    def _pack_motor_cmd(self, motor_id, mode, tau, omega, pos, kp, kw):
        # 考虑传动比
        tau /= 6.33
        omega *= 6.33
        pos *= 6.33

        buf = bytearray(17) 
        buf[0] = 0xFE
        buf[1] = 0xEE
        buf[2] = (motor_id & 0x0F) | ((mode & 0x07) << 4) 
        
        tau = max(min(tau, 127.99), -127.99)
        kp = max(min(kp, 25.599), 0.0)
        kw = max(min(kw, 25.599), 0.0)

        t_set = int(tau * 256) 
        w_set = int((omega / (2 * math.pi)) * 256)
        pos_set = int((pos / (2 * math.pi)) * 32768)
        kp_set = int(kp * 1280)
        kw_set = int(kw * 1280)

        struct.pack_into('<hhiHH', buf, 3, t_set, w_set, pos_set, kp_set, kw_set)
        crc = self._crc16_ccitt(buf[:15])
        struct.pack_into('<H', buf, 15, crc)
        return bytes(buf)

    # 解包
    def _parse_motor_feedback(self, response_bytes):
        if len(response_bytes) < 16 or response_bytes[0] != 0xFD or response_bytes[1] != 0xEE:
            return None
            
        try:
            received_crc = struct.unpack_from('<H', response_bytes, 14)[0]
        except struct.error:
            return None

        calculated_crc = self._crc16_ccitt(response_bytes[:14])
        if received_crc != calculated_crc:
            return None
            
        # 【核心修复】：提取 Byte 2 中的电机 ID 和 错误码
        motor_id = response_bytes[2] & 0x0F
        # merror = (response_bytes[2] >> 4) & 0x07 # 如果你需要监控错误码可以解除注释

        try:
            tau_int, omega_int, pos_int, temp = struct.unpack_from('<hhib', response_bytes, 3)
        except struct.error:
            return None

        tau_fbk = tau_int / 256.0
        omega_fbk = (omega_int / 256.0) * (2 * math.pi)
        pos_fbk = (pos_int / 32768.0) * (2 * math.pi)
        
        tau_fbk *= 6.33
        omega_fbk /= 6.33
        pos_fbk /= 6.33

        # 将解析出的 ID 一起返回
        return {"id": motor_id, "tau": tau_fbk, "omega": omega_fbk, "pos": pos_fbk, "temp": temp}

    # 生成校验码
    def _crc16_ccitt(self, data):
        crc = 0x0000
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0x8408
                else:
                    crc >>= 1
        return crc


def move(ser: SerialPort, motor_cmd: MotorCmd, motor_data: MotorData, target_pos: float, duration: float, kp: float, kd: float):
    """
    使电机平滑移动到指定位置（使用五次多项式 Minimum Jerk 轨迹）
    """
    
    # 1. 智能获取起点位置（防止泄力掉头）
    if motor_cmd.kp > 0:
        # 【情况 A：电机已经受控且正在发力】
        # 绝不能发 kp=0 去读取真实位置，否则瞬间泄力！
        # 直接使用上一次规划的终点指令位置作为新的起点，保证轨迹连续。
        start_pos = motor_cmd.q
    else:
        # 【情况 B：电机尚未使能，处于软态】
        # 此时发送当前指令状态来获取真实的物理位置。
        success = False
        for _ in range(5):
            if ser.sendRecv(motor_cmd, motor_data):
                success = True
                break
            time.sleep(0.01)
            
        if not success:
            print(f"警告: 电机 ID={motor_cmd.id} 获取初始位置失败，停止当前移动任务。")
            return
            
        start_pos = motor_data.q
        # 第一次上电使能时，预先将指令位置对齐物理位置，防止跳变
        motor_cmd.q = start_pos 
        
    # 2. 设置控制模式准备运动
    motor_cmd.mode = 1  
    motor_cmd.kp = kp
    motor_cmd.kd = kd
    # 注意：不要粗暴地把 tau 清零，如果要加入重力补偿，tau 会非常有价值
    
    # 3. 轨迹规划并执行
    start_time = time.time()
    
    while True:
        elapsed_time = time.time() - start_time
        
        # 运动结束判断
        if elapsed_time >= duration:
            break
            
        # 计算五次多项式轨迹 (Minimum Jerk Trajectory)
        s = elapsed_time / duration # 归一化时间 0.0 ~ 1.0
        
        # 位置插值系数：10s^3 - 15s^4 + 6s^5 
        ratio = 10 * (s**3) - 15 * (s**4) + 6 * (s**5)
        # 速度前馈系数：位置公式的导数
        d_ratio = (30 * (s**2) - 60 * (s**3) + 30 * (s**4)) / duration
        
        # 更新指令并发送
        motor_cmd.q = start_pos + (target_pos - start_pos) * ratio
        motor_cmd.dq = (target_pos - start_pos) * d_ratio
        
        ser.sendRecv(motor_cmd, motor_data)
        
        # 控制大约在 500Hz 左右的指令下发频率
        time.sleep(0.002) 

    # 4. 运动结束，强制锁定到最终目标位置，速度清零
    motor_cmd.q = target_pos
    motor_cmd.dq = 0.0
    ser.sendRecv(motor_cmd, motor_data)
    return target_pos





def test():


    try:
        ser = SerialPort("/dev/ttyUSB1") 
        mt0 = MotorCmd(id=0, direction=1, offset=-0)
        mt1 = MotorCmd(id=1, direction=1, offset=+0.926)
        mt2 = MotorCmd(id=2, direction=-1, offset=+3.162)
        mt3 = MotorCmd(id=3, direction=-1, offset=0)
        dt0 = MotorData()
        dt1 = MotorData()
        dt2 = MotorData()
        dt3 = MotorData()


        move(ser, mt1, dt1, target_pos=0.5, duration=2.0, kp=1.0, kd=0.01)
        move(ser, mt2, dt2, target_pos=2.6, duration=2.0, kp=1.0, kd=0.01)
        time.sleep(10)
        mt0.mode = 0
        mt1.mode = 0
        mt2.mode = 0
        mt3.mode = 0
        ser.sendRecv(mt0, dt0)
        ser.sendRecv(mt1, dt1)
        ser.sendRecv(mt2, dt2)
        ser.sendRecv(mt3, dt3)
        print("\n程序停止")




    except KeyboardInterrupt:
        mt0.mode = 0
        mt1.mode = 0
        mt2.mode = 0
        mt3.mode = 0
        ser.sendRecv(mt0, dt0)
        ser.sendRecv(mt1, dt1)
        ser.sendRecv(mt2, dt2)
        ser.sendRecv(mt3, dt3)
        print("\n程序停止")








if __name__ == "__main__":
    test()