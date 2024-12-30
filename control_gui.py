import time
import dearpygui.dearpygui as dpg
from pynput import keyboard  # 用于监听键盘输入
from openpilot.tools.sim.bridge.metadrive.middleware import Env  # 导入你的 Env 类


class ENVGuiController:
    def __init__(self, env_instance):
        self.env = env_instance  # 存储ENV实例的引用
        self.properties = self._get_env_properties()  # 获取ENV实例中的所有getter属性
        self.setup_gui()

        # 手动干预控制
        self.manual_override = False  # 默认优先使用原始自动控制
        self.manual_speed = 0.0  # 手动控制的目标速度
        self.manual_angle = 0.0  # 手动控制的方向偏移
        self.key_pressed = set()  # 存储当前按下的键
        self.listener = None  # 键盘监听器

        # 帧率控制相关变量
        self.target_fps = 20  # 设置 20 FPS
        self.last_update_time = 0.0  # 上次刷新时间

    def _get_env_properties(self):
        """自动获取env对象内支持getter的全部属性"""
        props = {}
        for attr_name in dir(self.env):
            attr = getattr(type(self.env), attr_name, None)
            # 检查是否为property，且具有getter
            if isinstance(attr, property) and attr.fget:
                props[attr_name] = attr
        return props

    def setup_gui(self):
        dpg.create_context()

        # 创建主窗口
        with dpg.window(label="ENV Controller", pos=(0, 0), width=800, height=400):
            with dpg.group(horizontal=True):  # 分两列布局
                # 左侧：控制（滑块、开关等）
                with dpg.group():
                    dpg.add_text("Manual Controls", color=(0, 255, 0))

                    # 速度滑动条
                    dpg.add_slider_float(
                        label="Target Speed",
                        min_value=0.0,
                        max_value=30.0,
                        default_value=0.0,
                        callback=self.update_manual_speed
                    )

                    # 方向滑动条
                    dpg.add_slider_float(
                        label="Steering Angle",
                        default_value=0.0,
                        min_value=-10.0,
                        max_value=10.0,
                        callback=self.update_manual_angle
                    )
                    
                # 右侧：读取属性显示
                with dpg.group():
                    dpg.add_text("Properties:", color=(255, 255, 0))

                    # 遍历 ENV 对象中的所有属性动态显示
                    self.property_texts = {}
                    for name, _ in self.properties.items():
                        prop_label = dpg.add_text(f"{name}: ")
                        prop_value = dpg.add_text("")  # 初始化值为空
                        self.property_texts[name] = prop_value  # 存储 UI 元素

        # 设置 viewport
        dpg.create_viewport(title="ENV Parameter Controller", width=800, height=400)
        dpg.setup_dearpygui()
        dpg.show_viewport()

    def on_press(self, key):
        """键盘按键按下事件"""
        try:
            if key.char == "w":  # 加速
                self.manual_speed = min(30.0, self.manual_speed + 1.0)
            elif key.char == "s":  # 减速
                self.manual_speed = max(0.0, self.manual_speed - 1.0)
        except AttributeError:
            if key == keyboard.Key.left:  # 左转
                self.manual_angle = max(-10.0, self.manual_angle - 1.0)
            elif key == keyboard.Key.right:  # 右转
                self.manual_angle = min(10.0, self.manual_angle + 1.0)
        self.manual_override = True  # 键盘按下时开启手动模式

    def on_release(self, key):
        """键盘按键松开事件"""
        if key in [keyboard.Key.left, keyboard.Key.right]:  # 松开方向键
            self.manual_angle = 0.0

        # 如果没有任何键被按下，关闭手动控制模式
        if key not in self.key_pressed:
            self.manual_override = False

    def start_keyboard_listener(self):
        """启动键盘监听器"""
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def stop_keyboard_listener(self):
        """停止键盘监听器"""
        if self.listener:
            self.listener.stop()

    def update_manual_speed(self, sender, value):
        """更新手动速度值"""
        self.manual_speed = value
        if not self.manual_override:  # 如果未手动干预，实时写入 ENV 对象
            self.env.target_speed = value

    def update_manual_angle(self, sender, value):
        """更新手动角度值"""
        self.manual_angle = value
        if not self.manual_override:  # 如果未手动干预，实时写入 ENV 对象
            self.env.wheel_angle = value

    def update_property_display(self):
        """实时刷新右侧的属性值显示"""
        if self.manual_override:  # 如果是键盘控制
            self.env.target_speed = self.manual_speed
            self.env.wheel_angle = self.manual_angle

        for name, text_id in self.property_texts.items():
            if hasattr(self.env, name):
                new_value = getattr(self.env, name)
                dpg.set_value(text_id, f"{name}: {new_value}")

    def start(self):
        """开始 GUI 主循环"""
        print("Starting GUI Controller...")
        self.start_keyboard_listener()
        while dpg.is_dearpygui_running():
            current_time = time.time()
            if current_time - self.last_update_time >= (1.0 / self.target_fps):
                self.update_property_display()  # 定时刷新属性显示
                self.last_update_time = current_time
            dpg.render_dearpygui_frame()

        self.cleanup()

    def cleanup(self):
        self.stop_keyboard_listener()
        dpg.destroy_context()


if __name__ == "__main__":
    env_instance = Env()
    gui_controller = ENVGuiController(env_instance)
    gui_controller.start()
