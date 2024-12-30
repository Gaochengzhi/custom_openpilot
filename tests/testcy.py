import cv2

# 视频流的URL
video_url = "http://192.168.8.18:4747/video"

# 使用OpenCV从URL中读取视频流
cap = cv2.VideoCapture(video_url)

if not cap.isOpened():
    print("无法打开视频流，请检查URL或网络连接")
    exit()

print("正在播放视频流，按 'q' 键退出")

while True:
    # 读取视频帧
    ret, frame = cap.read()
    
    if not ret:
        print("无法接收视频帧，可能视频流已停止")
        break
    
    # 显示当前帧
    cv2.imshow("Video Stream", frame)
    
    # 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源并关闭显示窗口
cap.release()
cv2.destroyAllWindows()
