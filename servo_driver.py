import serial
import time
import math

# ================= 底层通信类 =================
class FeetechSTSServo:
    def __init__(self, port, baudrate=1000000, timeout=0.1):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
        except Exception as e:
            print(f"打开串口失败: {e}")
            self.ser = None

    def move_to(self, servo_id, target_position, target_speed=2000, target_time=0):
        if not self.ser or not self.ser.is_open:
            return

        servo_id = max(0, min(253, int(servo_id)))
        target_position = max(0, min(4095, int(target_position)))
        target_speed = max(0, min(3400, int(target_speed)))
        target_time = max(0, int(target_time))

        pos_l = target_position & 0xFF
        pos_h = (target_position >> 8) & 0xFF
        time_l = target_time & 0xFF
        time_h = (target_time >> 8) & 0xFF
        speed_l = target_speed & 0xFF
        speed_h = (target_speed >> 8) & 0xFF

        length = 0x09      
        instruction = 0x03 
        address = 0x2A     

        checksum_sum = (servo_id + length + instruction + address + 
                        pos_l + pos_h + time_l + time_h + speed_l + speed_h)
        checksum = (~checksum_sum) & 0xFF

        packet = [0xFF, 0xFF, servo_id, length, instruction, address, 
                  pos_l, pos_h, time_l, time_h, speed_l, speed_h, checksum]

        self.ser.write(bytearray(packet))

    def set_torque_limit(self, servo_id, torque_value):
        if not self.ser or not self.ser.is_open:
            return

        servo_id = max(0, min(253, int(servo_id)))
        torque_value = max(0, min(1000, int(torque_value)))

        torque_l = torque_value & 0xFF
        torque_h = (torque_value >> 8) & 0xFF

        length = 0x05      
        instruction = 0x03 
        address = 0x30     

        checksum_sum = (servo_id + length + instruction + address + torque_l + torque_h)
        checksum = (~checksum_sum) & 0xFF

        packet = [0xFF, 0xFF, servo_id, length, instruction, address, torque_l, torque_h, checksum]
        self.ser.write(bytearray(packet))
        time.sleep(0.005)

    def read_position(self, servo_id):
        """
        读取指定舵机的当前位置 (寄存器地址 0x38)
        """
        if not self.ser or not self.ser.is_open:
            return None

        # 发送请求前清空接收缓冲区，防止读到之前的脏数据
        self.ser.reset_input_buffer()

        servo_id = max(0, min(253, int(servo_id)))
        length = 0x04      # 长度 = 指令(1) + 地址(1) + 读取字节数(1) + 校验和(1)
        instruction = 0x02 # 0x02 代表读指令 (READ DATA)
        address = 0x38     # 0x38 (十进制56) 是“当前位置”的首地址
        data_len = 0x02    # 位置数据占 2 个字节 (Low 和 High)

        # 计算校验和
        checksum_sum = (servo_id + length + instruction + address + data_len)
        checksum = (~checksum_sum) & 0xFF

        # 发送读取包
        packet = [0xFF, 0xFF, servo_id, length, instruction, address, data_len, checksum]
        self.ser.write(bytearray(packet))

        # 读取应答包
        # 格式: 0xFF 0xFF ID Length Error Param1(Pos_L) Param2(Pos_H) Checksum
        # 总长 = 8 bytes
        response = self.ser.read(8)
        
        if len(response) == 8 and response[0] == 0xFF and response[1] == 0xFF:
            if response[2] == servo_id:
                # 检查错误标志位 (response[4])，如果是 0 则正常
                if response[4] != 0:
                    print(f"舵机 {servo_id} 返回硬件错误码: {response[4]}")
                
                pos_l = response[5]
                pos_h = response[6]
                position = pos_l | (pos_h << 8)
                
                # 处理可能出现的负数 (对于带有符号拓展配置的型号)
                if position > 32767:
                    position -= 65536
                    
                return position
        return None
    
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


# ================= 高级应用封装类 =================
class RobotHandController:
    def __init__(self, port):
        self.servo_driver = FeetechSTSServo(port=port)
        
        # 舵机 ID
        self.GRIPPER_ID = 1
        self.WRIST_ID = 2
        
        # 夹爪限位与参数
        self.GRIPPER_OPEN_POS = 3300
        self.GRIPPER_CLOSE_POS = 2850
        self.DEFAULT_TORQUE = 300
        
        # 手腕限位与参数
        self.WRIST_CENTER_POS = 3000
        self.WRIST_MIN_POS = 2200
        self.WRIST_MAX_POS = 3800
        
        # 转换比例: 1弧度对应的舵机步数
        self.STEPS_PER_RADIAN = 4095 / (2 * math.pi)

    def is_connected(self):
        return self.servo_driver.ser is not None and self.servo_driver.ser.is_open

    # ---------------- 夹爪开关 ----------------
    def gripper(self, state, speed=1500, torque=None):
        if not self.is_connected():
            return

        if torque is not None:
            self.servo_driver.set_torque_limit(self.GRIPPER_ID, torque)

        if state == 1:
            target_pos = self.GRIPPER_OPEN_POS

        elif state == -1:
            target_pos = self.GRIPPER_CLOSE_POS


        self.servo_driver.move_to(self.GRIPPER_ID, target_position=target_pos, target_speed=speed)




    # ---------------- 手腕控制 (核心修复点) ----------------
    def wrist(self, angle_radians, speed=1500):
        if not self.is_connected():
            return

        # 1. 自动适配物理限位大小，防止日后参数填反
        phys_min = min(self.WRIST_MIN_POS, self.WRIST_MAX_POS)
        phys_max = max(self.WRIST_MIN_POS, self.WRIST_MAX_POS)

        # 2. 先算出理论目标步数 (不带限位)
        target_pos = int(self.WRIST_CENTER_POS + (angle_radians * self.STEPS_PER_RADIAN))

        # 3. 直接在物理步数层面进行硬限位（100%杜绝浮点逻辑锁死）
        safe_pos = max(phys_min, min(phys_max, target_pos))

        # 4. 根据实际限制后的步数，反推真实的物理弧度
        actual_radians = (safe_pos - self.WRIST_CENTER_POS) / self.STEPS_PER_RADIAN

            
        self.servo_driver.move_to(self.WRIST_ID, target_position=safe_pos, target_speed=speed)

    def get_wrist_state(self):
        """
        获取手腕的当前物理步数和对应的真实弧度
        """
        if not self.is_connected():
            return None, None
            
        pos = self.servo_driver.read_position(self.WRIST_ID)
        
        if pos is not None:
            # 将原始步数反推回弧度: (当前步数 - 中位步数) / 比例
            actual_radians = -(pos - self.WRIST_CENTER_POS) / self.STEPS_PER_RADIAN
            return pos, actual_radians
            
        return None, None
    
    def shutdown(self):
        self.servo_driver.close()


# ================= 调用示例 =================
if __name__ == "__main__":
    PORT = '/dev/ttyUSB4' 
    hand = RobotHandController(port=PORT)
    
    if hand.is_connected():
        print("=== 开始测试 ===")
        
        # 1. 夹爪控制 (1开，-1关)
        hand.gripper(-1)
        time.sleep(1)
        hand.gripper(1)
        time.sleep(1)
        hand.ripper(-1)
        time.sleep(1)
        
        # 2. 手腕控制测试
        hand.wrist(-1)  # 逆时针转 
        time.sleep(1.5)
        
        hand.wrist(1.0)           # 顺时针转 1.0 rad，这次一定能转过来！
        time.sleep(1.5)
        
        hand.wrist(0)             # 正常回中
        time.sleep(2)
        
        current_step, current_rad = hand.get_wrist_state()
        print(f"当前手腕状态 -> 原始步数: {current_step}, 弧度: {current_rad:.3f} rad")


        hand.shutdown()
        print("=== 测试结束 ===")