import time
from driver import RobotArm

def test_polling_speed():
    # 串口配置
    MOTOR_PORT = "/dev/ttyUSB2"
    SERVO_PORT = "/dev/ttyUSB1"
    
    # 测试时长（秒）
    TEST_DURATION = 5.0

    print(f"正在连接硬件并准备测试轮询速度（预计耗时 {TEST_DURATION} 秒）...")
    
    try:
        with RobotArm(MOTOR_PORT, SERVO_PORT) as arm:
            # 1. 测试 arm.read() 的频率
            print("\n测试 [arm.read()] 接口...")
            count = 0
            start_time = time.perf_counter()
            while time.perf_counter() - start_time < TEST_DURATION:
                arm.read()
                count += 1
            
            end_time = time.perf_counter()
            actual_duration = end_time - start_time
            read_hz = count / actual_duration
            print(f"完成次数: {count}")
            print(f"平均频率: {read_hz:.2f} Hz (单次耗时: {1000/read_hz:.2f} ms)")

            # 2. 测试 arm.follow() 的频率（包含逆解、重力补偿和数据同步）
            print("\n测试 [arm.follow()] 接口 (控制循环)...")
            count = 0
            # 给定一个固定的测试目标
            target = (0.2, 0.0, 0.1)
            start_time = time.perf_counter()
            while time.perf_counter() - start_time < TEST_DURATION:
                arm.follow(*target, phi=-0.5, roll=0.0, gripper=1)
                count += 1
                
            end_time = time.perf_counter()
            actual_duration = end_time - start_time
            follow_hz = count / actual_duration
            print(f"完成次数: {count}")
            print(f"平均频率: {follow_hz:.2f} Hz (单次耗时: {1000/follow_hz:.2f} ms)")

    except Exception as e:
        print(f"测试出错: {e}")

if __name__ == "__main__":
    test_polling_speed()