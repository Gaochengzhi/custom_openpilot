import av
import cv2
import time
import numpy as np
from threading import Thread

class USBCamera:
    def __init__(self, device_id):
        self.device_id = device_id
        self.container = av.open(f'/dev/video{device_id}')
        self.stream = self.container.streams.video[0]
        self.frame = None
        self.stopped = False
        self.fps = 0
        self.prev_time = time.time()
        self.frame_count = 0

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return

            try:
                for frame in self.container.decode(video=0):
                    if self.stopped:
                        return
                    
                    self.frame = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                    self.frame_count += 1
                    
                    # 计算FPS
                    current_time = time.time()
                    if current_time - self.prev_time >= 1.0:
                        self.fps = self.frame_count
                        self.frame_count = 0
                        self.prev_time = current_time
            except:
                continue

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.container.close()

def main():
    # 创建两个摄像头实例
    cam0 = USBCamera(0)
    cam2 = USBCamera(2)

    # 启动摄像头线程
    cam0.start()
    cam2.start()

    # 等待摄像头初始化
    time.sleep(2)

    try:
        while True:
            # 读取两个摄像头的画面
            frame0 = cam0.read()
            frame2 = cam2.read()

            if frame0 is not None:
                # 在画面上显示FPS
                cv2.putText(frame0, f"FPS: {cam0.fps}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow('Camera 0', frame0)

            if frame2 is not None:
                # 在画面上显示FPS
                cv2.putText(frame2, f"FPS: {cam2.fps}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow('Camera 2', frame2)

            # 按'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # 释放资源
        cam0.stop()
        cam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
