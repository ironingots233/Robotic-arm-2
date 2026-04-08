import cv2

def main():
    # 1. 创建 VideoCapture 对象
    # 参数 0 通常是内置摄像头，1 或 2 通常是外接 USB 摄像头
    cap = cv2.VideoCapture(2)

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("Error: 无法打开摄像头，请检查索引号或连接。")
        return

    print("摄像头已启动，按下 'q' 键退出程序。")

    while True:
        # 2. 逐帧读取视频
        # ret 是布尔值（是否读取成功），frame 是图像矩阵
        ret, frame = cap.read()

        if not ret:
            print("无法接收帧，正在退出...")
            break

        # 3. 在窗口中显示结果
        cv2.imshow('USB Camera Test', frame)

        # 4. 检测按键，按下 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 5. 释放资源
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()