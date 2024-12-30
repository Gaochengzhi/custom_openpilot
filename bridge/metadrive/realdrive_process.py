import math
import time
import numpy as np
import cv2

from collections import namedtuple
from panda3d.core import Vec3
from multiprocessing.connection import Connection
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor
from openpilot.tools.sim.bridge.metadrive.middleware import Env
from openpilot.tools.sim.control_gui import ENVGuiController
from openpilot.common.realtime import Ratekeeper

from openpilot.tools.sim.lib.common import vec3
from openpilot.tools.sim.lib.camerad import W, H
import threading
import queue
import dearpygui.dearpygui as dpg

C3_POSITION = Vec3(0.0, 0, 1.22)
C3_HPR = Vec3(0, 0, 0)


realdrive_simulation_state = namedtuple(
    "realdrive_simulation_state", ["running", "done", "done_info"]
)
realdrive_vehicle_state = namedtuple(
    "realdrive_vehicle_state", ["velocity", "position", "bearing", "steering_angle"]
)

class AsyncVideoCapture:
    def __init__(self, cap, name, target_shape):
        self.cap = cap
        self.name = name
        self.target_shape = target_shape
        self.queue = queue.Queue(maxsize=4) # 限制队列大小为2
        self.stopped = False
        
        # 启动异步读取线程
        self.thread = threading.Thread(target=self._reader)
        self.thread.daemon = True
        self.thread.start()

    def _reader(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if not ret:
                self.stopped = True
                break
                
            # 调整尺寸
            if self.name == "rgb_wide":
                frame = cv2.flip(frame, -1)
            frame = cv2.resize(frame, (self.target_shape[1], self.target_shape[0]))
            
            # 如果队列满了,移除旧帧
            if self.queue.full():
                try:
                    self.queue.get_nowait()
                except queue.Empty:
                    pass
            
            self.queue.put(frame)

    def read(self):
        return True, self.queue.get()

    def release(self):
        self.stopped = True
        self.thread.join()
        self.cap.release()

def realdrive_process(
    dual_camera: bool,
    config: dict,
    camera_array,
    wide_camera_array,
    image_lock,
    controls_recv: Connection,
    simulation_state_send: Connection,
    vehicle_state_send: Connection,
    exit_event,
    op_engaged,
    test_duration,
    test_run,
):
    road_image = np.frombuffer(camera_array.get_obj(), dtype=np.uint8).reshape(
        (H, W, 3)
    )
    if dual_camera:
        assert wide_camera_array is not None
        wide_road_image = np.frombuffer(
            wide_camera_array.get_obj(), dtype=np.uint8
        ).reshape((H, W, 3))

    def reset():
        return 0

    start_time = None


    rk = Ratekeeper(100, None)

    steer_ratio = 18
    vc = [0, 0]

    env = Env()
    gui_controller = ENVGuiController(env)
    # cap_wide.set(cv2.CAP_PROP_FPS, 30)
    def init_cameras():
        cap_road = cv2.VideoCapture(0)
        if not cap_road.isOpened():
            raise ValueError("Cannot open camera at index 0")
        # cap_road.set(cv2.CAP_PROP_FRAME_WIDTH, 1928)
        # cap_road.set(cv2.CAP_PROP_FRAME_HEIGHT, 1208)

        # cap_wide = cv2.VideoCapture("http://192.168.8.22:4747/video")
        cap_wide = cv2.VideoCapture(2)
        
        if not cap_wide.isOpened():
            raise ValueError("Cannot open camera at index 2")
        # cap_wide.set(cv2.CAP_PROP_FRAME_WIDTH, 1928)
        # cap_wide.set(cv2.CAP_PROP_FRAME_HEIGHT, 1208)
        cap_road.set(cv2.CAP_PROP_FPS, 30)
        cap_wide.set(cv2.CAP_PROP_FPS, 30)

        # 创建异步捕获对象
        async_cap_road = AsyncVideoCapture(cap_road, "rgb_road", (1208, 1928))
        async_cap_wide = AsyncVideoCapture(cap_wide, "rgb_wide", (1208, 1928))
        
        return async_cap_road, async_cap_wide
    
    try:
        async_cap_road, async_cap_wide = init_cameras()
        dpg.show_viewport()
        print("realdrive_process started")
        def get_cam_as_rgb(cap_name):
            time_past = time.time()
            if cap_name == "rgb_road":
                cap = async_cap_road
                target_shape = (1208,1928)
            elif cap_name == "rgb_wide":
                cap = async_cap_wide
                target_shape = (1208,1928)
                
            ret, frame = cap.read()
            if not ret:
                raise RuntimeError("Failed to read frame from camera")
            # print("dddddddddddddddddddddddbud",cap_name,frame.shape)
            frame = cv2.resize(frame, (target_shape[1], target_shape[0]))
            time_past = time.time() - time_past
            # print("time_past",cap_name,time_past)
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        while not exit_event.is_set():
            gui_controller.update_property_display()
            dpg.render_dearpygui_frame()
            vehicle_state = realdrive_vehicle_state(
                velocity=vec3(
                    x=float(env.vector_velocity[0]), y=float(env.vector_velocity[1]), z=0
                ),
                position=env.postion,
                bearing=float(env.heading),
                steering_angle=env.steering_angle,
            )
            vehicle_state_send.send(vehicle_state)

            if controls_recv.poll(0):
                while controls_recv.poll(0):
                    steer_angle, gas, should_reset = controls_recv.recv()

                steer_metadrive = steer_angle * 1 / (20)
                # steer_metadrive = np.clip(steer_metadrive, -1, 1)

                vc = [steer_metadrive, gas]

                if should_reset:
                    lane_idx_prev = reset()
                    start_time = None

            is_engaged = op_engaged.is_set()
            if is_engaged and start_time is None:
                start_time = time.monotonic()

            if rk.frame % 5 == 0:
                # _, _, terminated, _, _ = env.step(vc)
                env.send(vc)
                timeout = (
                    True
                    if start_time is not None
                    and time.monotonic() - start_time >= test_duration
                    else False
                )

                if dual_camera:
                    wide_road_image[...] = get_cam_as_rgb("rgb_wide")
                road_image[...] = get_cam_as_rgb("rgb_road")
                image_lock.release()

            rk.keep_time()
    except Exception as e:
        print("Exception in realdrive_process:", e)
        raise
    finally:
        async_cap_road.release()
        async_cap_wide.release()
        gui_controller.cleanup()