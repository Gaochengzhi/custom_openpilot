import numpy as np
import cv2


class RealCamera:
    """Base class for real camera implementations"""

    def __init__(self, camera_id=0, width=640, height=480):
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {camera_id}")

    def get_rgb_array_cpu(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to grab frame from camera")
        # OpenCV uses BGR format, convert to RGB
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def __del__(self):
        if hasattr(self, "cap"):
            self.cap.release()


class RGBCameraWide(RealCamera):
    def __init__(self, camera_id=0, width=640, height=480):
        super().__init__(camera_id, width, height)
        # 设置广角相机参数
        # 注意：实际效果取决于物理摄像头的硬件特性
        self.cap.set(cv2.CAP_PROP_FPS, 30)  # 设置帧率
        # 可以在这里添加其他相机参数设置


class RGBCameraRoad(RealCamera):
    def __init__(self, camera_id=0, width=640, height=480):
        super().__init__(camera_id, width, height)
        # 设置道路相机参数
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        # 可以在这里添加其他相机参数设置
