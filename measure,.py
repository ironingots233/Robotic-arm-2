import time
from driver import RobotArm

with RobotArm("/dev/ttyUSB1", "/dev/ttyUSB5") as arm:
    arm.enable(0, 0)
    try:
        while True:
            state = arm.read()
            grip_str = "开" if state.gripper > 0 else ("关" if state.gripper < 0 else "?")
            print(f"坐标: x={state.x:+.4f}, y={state.y:+.4f}, z={state.z:+.4f} m | "
                  f"手腕: {state.roll:+.3f} rad | 夹爪: {state.gripper:.0f}", end='\r')
            time.sleep(0.01)
    except KeyboardInterrupt:
        arm.disable()
        print("\n程序停止")
