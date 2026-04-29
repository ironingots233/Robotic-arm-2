import cv2

def main():
    # 初始化第一个摄像头 (ID 为 0)
    cap0 = cv2.VideoCapture(0)
    # 初始化第二个摄像头 (ID 为 1，如果打不开可以试 2)
    cap1 = cv2.VideoCapture(2)

    # 检查两个摄像头是否都成功打开
    if not cap0.isOpened() or not cap1.isOpened():
        print("❌ 错误：无法打开双摄像头！")
        print(f"摄像头 0 状态: {'成功' if cap0.isOpened() else '失败'}")
        print(f"摄像头 1 状态: {'成功' if cap1.isOpened() else '失败'}")
        
        # 释放已打开的资源
        cap0.release()
        cap1.release()
        return

    print("✅ 双摄像头已开启！")
    print("👉 窗口 1: Camera 0")
    print("👉 窗口 2: Camera 1")
    print("👉 按下键盘上的 'q' 键退出预览。")

    while True:
        # 分别读取两个摄像头的帧
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()
        
        if not ret0 or not ret1:
            print("❌ 错误：无法从其中一个摄像头读取画面。")
            break

        # 在两个独立的窗口中显示画面
        cv2.imshow('Camera 0 (Left/Main)', frame0)
        cv2.imshow('Camera 1 (Right/Sub)', frame1)

        # 检测 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("🛑 已退出预览。")
            break

    # 释放所有资源
    cap0.release()
    cap1.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()