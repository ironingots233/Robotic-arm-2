import time
from driver import RobotArm

with RobotArm("/dev/ttyUSB1", "/dev/ttyUSB6") as arm:
    arm.enable(0, 0)
    try:
        while True:
            state = arm.read()
            print(f"电机0: {state.joints[0]:+.3f} rad | "
                  f"电机1: {state.joints[1]:+.3f} rad | "
                  f"电机2: {state.joints[2]:+.3f} rad | "
                  f"电机3: {state.joints[3]:+.3f} rad | "
                  f"手腕: {state.roll:+.3f} rad | "
                  f"夹爪: {state.gripper:.0f}", end='\r')

    except KeyboardInterrupt:
        arm.disable()
        print("\n程序停止")
