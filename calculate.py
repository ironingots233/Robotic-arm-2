import csv
import numpy as np
import matplotlib.pyplot as plt

def calculate_3axis_gravity_parameters(csv_filename):
    # 1. 读取三轴数据
    times = []
    q1, tau1 = [], []
    q2, tau2 = [], []
    q3, tau3 = [], []
    
    try:
        with open(csv_filename, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                times.append(float(row['time']))
                q1.append(float(row['mt1_pos']))
                tau1.append(float(row['mt1_tau']))
                q2.append(float(row['mt2_pos']))
                tau2.append(float(row['mt2_tau']))
                q3.append(float(row['mt3_pos']))
                tau3.append(float(row['mt3_tau']))
    except FileNotFoundError:
        print(f"找不到文件: {csv_filename}")
        return

    # 转换为 NumPy 数组
    times = np.array(times)
    q1, tau1 = np.array(q1), np.array(tau1)
    q2, tau2 = np.array(q2), np.array(tau2)
    q3, tau3 = np.array(q3), np.array(tau3)

    # 预计算绝对角度
    q12 = q1 + q2
    q123 = q1 + q2 + q3

    # ================= 核心计算：引入常数项 C 吸收摩擦力与零位误差 =================
    
    # 3. 计算末端关节(轴3)
    # 模型: tau3 = A3*cos() + B3*sin() + C3
    # np.ones_like(q123) 相当于一列全为 1 的数组，用来拟合常数 C
    X3 = np.column_stack((np.cos(q123), np.sin(q123), np.ones_like(q123)))
    Y3 = tau3
    params3, _, _, _ = np.linalg.lstsq(X3, Y3, rcond=None)
    A3, B3, C3 = params3[0], params3[1], params3[2]

    # 2. 计算中间关节(轴2)
    X2 = np.column_stack((np.cos(q12), np.sin(q12), np.ones_like(q12)))
    Y2 = tau2 - tau3
    params2, _, _, _ = np.linalg.lstsq(X2, Y2, rcond=None)
    A2, B2, C2 = params2[0], params2[1], params2[2]

    # 1. 计算基座关节(轴1)
    X1 = np.column_stack((np.cos(q1), np.sin(q1), np.ones_like(q1)))
    Y1 = tau1 - tau2
    params1, _, _, _ = np.linalg.lstsq(X1, Y1, rcond=None)
    A1, B1, C1 = params1[0], params1[1], params1[2]

    print("="*45)
    print("🎯 三轴高级重力补偿参数（已吸收摩擦力与偏置）：")
    print(f"A1 (轴1 cos) = {A1:.4f}, B1 (轴1 sin) = {B1:.4f}, C1 (静态偏置) = {C1:.4f}")
    print("-" * 45)
    print(f"A2 (轴2 cos) = {A2:.4f}, B2 (轴2 sin) = {B2:.4f}, C2 (静态偏置) = {C2:.4f}")
    print("-" * 45)
    print(f"A3 (轴3 cos) = {A3:.4f}, B3 (轴3 sin) = {B3:.4f}, C3 (静态偏置) = {C3:.4f}")
    print("="*45)

    # ================= 结果可视化 =================
    # 计算理论拟合力矩时，带上常数项 C
    tau3_fit = A3 * np.cos(q123) + B3 * np.sin(q123) + C3
    tau2_fit = A2 * np.cos(q12) + B2 * np.sin(q12) + C2 + tau3_fit
    tau1_fit = A1 * np.cos(q1) + B1 * np.sin(q1) + C1 + tau2_fit

    plt.figure(figsize=(15, 5))

    # 轴 3 绘图
    plt.subplot(1, 3, 1)
    plt.plot(times - times[0], tau3, 'o', markersize=3, label='Measured', alpha=0.6)
    plt.plot(times - times[0], tau3_fit, '-', linewidth=2, label='Fitted (with C)')
    plt.title("Motor 3 (末端关节)")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.legend()
    plt.grid(True)

    # 轴 2 绘图
    plt.subplot(1, 3, 2)
    plt.plot(times - times[0], tau2, 'o', markersize=3, label='Measured', alpha=0.6)
    plt.plot(times - times[0], tau2_fit, '-', linewidth=2, label='Fitted (with C)')
    plt.title("Motor 2 (中间关节)")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.legend()
    plt.grid(True)

    # 轴 1 绘图
    plt.subplot(1, 3, 3)
    plt.plot(times - times[0], tau1, 'o', markersize=3, label='Measured', alpha=0.6)
    plt.plot(times - times[0], tau1_fit, '-', linewidth=2, label='Fitted (with C)')
    plt.title("Motor 1 (基座关节)")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    calculate_3axis_gravity_parameters("motor_data.csv")