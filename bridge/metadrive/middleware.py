import socket
import threading
import time
import struct
import sys
from collections import defaultdict
import traceback
import serial  # 导入串口通信库
import math


SOCKET_PATH = "/tmp/can_read_socket"


SEND_PATH = "/tmp/can_socket"


class KeyboardPoller(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.running = True

    def run(self):
        while self.running:
            if sys.stdin.read(1) == "q":
                self.running = False
            time.sleep(0.1)

    def stop(self):
        self.running = False


class Env:
    def __init__(self):
        # 初始化数据存储，使用线程锁保护数据
        self.data = defaultdict(lambda: None)  # 存储解析后的 CAN 数据
        self.lock = threading.Lock()
        self.running = True  # 控制线程的运行
        self._target_speed = 0
        self._wheel_angle = 0.5
        self.brake = False

        # 启动后台线程收集数据
        sp = serial.Serial()
        sp.port = "/dev/ttyUSB0"  # 设置串口通信接口
        sp.baudrate = 115200  # 设置波特率
        sp.bytesize = serial.EIGHTBITS  # 数据位
        sp.parity = serial.PARITY_NONE  # 校验位
        sp.stopbits = serial.STOPBITS_ONE  # 停止位
        sp.timeout = 0  # 设置超时时间，单位：秒
        self.sp = sp
        self.sp.open()
        print(f"成功打开惯导串口：{sp.port}")
        self.socket_thread = threading.Thread(target=self._socket_reader, daemon=True)
        self.socket_thread.start()
        self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.serial_thread.start()
        self.keyboard_override = False  # 是否由键盘优先控制
        self.keyboard_speed_offset = 0  # 键盘控制的目标速度偏移值
        self.keyboard_angle_offset = 0  # 键盘控制的方向盘偏移值

        # 初始化键盘监听器
        self.keyboard_poller = KeyboardPoller()
        self.keyboard_poller.start()
        self.parse_rules = {
            "0x1806a0b0": [
                {
                    "name": "vehicle_speed",
                    "start_bit": 16,
                    "length": 8,
                    "factor": 1,
                    "offset": -50,
                },
                {
                    "name": "battery",
                    "start_bit": 24,
                    "length": 8,
                    "factor": 0.5,
                    "offset": 0,
                },
            ],
            "0x1802a0b0": [
                {
                    "name": "steering_angle",
                    "start_bit": 32,
                    "length": 8,
                    "factor": 1,
                    "offset": -42,
                },
            ],
        }
        try:
            self.send_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.send_socket.connect(SEND_PATH)
            print(f"成功连接到发送 Socket: {SEND_PATH}")
        except socket.error as e:
            print(f"发送 Socket 连接失败: {e}")
            self.send_socket = None

    def _extract_signal(
        self, data_bits, start_bit, length, factor, offset, round_digits=None
    ):
        """提取并解析CAN信号"""
        # 直接将hex字符串转换为整数，避免字符串操作
        value = int(data_bits, 16)
        # 通过位运算提取目标比特
        mask = (1 << length) - 1
        value = (value >> (start_bit)) & mask
        # 应用因子和偏移
        result = value * factor + offset
        return round(result, round_digits) if round_digits is not None else result

    def _hex_to_bitstream(self, data_bits):
        """
        将十六进制字符串转为二进制流。
        """
        return "".join(
            f"{int(data_bits[i:i+2], 16):08b}" for i in range(0, len(data_bits), 2)
        )

    def _extract_value(self, bitstream, start_bit, length, factor, offset):
        """
        从二进制流中提取并计算值。
        """
        raw_bits = bitstream[start_bit : start_bit + length]
        value = int(raw_bits, 2)  # 将二进制转为整数
        return value * factor + offset

    def parse_message(self, can_id, data_bits):
        """
        根据 can_id 和 data_bits 解析消息。
        """
        if can_id not in self.parse_rules:
            return {}  # 如果 can_id 不在规则中，返回空字典

        parsed_data = {}
        bitstream = self._hex_to_bitstream(data_bits)  # 转为二进制流
        for rule in self.parse_rules[can_id]:
            name = rule["name"]
            start_bit = rule["start_bit"]
            length = rule["length"]
            factor = rule["factor"]
            offset = rule["offset"]
            parsed_data[name] = round(
                self._extract_value(bitstream, start_bit, length, factor, offset), 2
            )

        return parsed_data

    def send(self, vc):
        """
        发送控制指令到环境中。vc = [steer, gas]
        """
        try:
            steer, gas = vc

            # 优先发送手动控制指令
            if self.keyboard_override:
                steer = self._wheel_angle  # 来自 GUI 或键盘的方向
                gas = self._target_speed  # 来自 GUI 或键盘的速度调整
            else:
                # 使用自动指令（来自车辆状态）
                gas = self.vehicle_speed

            # 生成控制消息并发送
            msg_steer = f"send 402763936 32 8 {format(abs(int(-(steer - 0.4) * 2 + 42)), '08b')}\n"
            msg_gas = f"send 402895008 0 8 {format(abs(int(gas * 2)), '08b')}\n"
            self.send_socket.send(msg_steer.encode())
            self.send_socket.send(msg_gas.encode())

        except Exception as e:
            print(f"发送控制指令失败: {e}")

    def _socket_reader(self):
        """
        从 Unix Socket 中读取数据并更新到内存中。
        """
        try:
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client.connect(SOCKET_PATH)
            client.setblocking(False)
        except socket.error as e:
            print(f"Socket error: {e}")
            return
        buffer_str = ""
        while self.running:
            try:
                chunk = client.recv(128)
                if not chunk:
                    # 对端关闭或无数据
                    time.sleep(0.01)
                    continue

                buffer_str += chunk.decode(errors="ignore")

                # 如果 buffer 中包含换行符，则仅解析最后一行
                if "\n" in buffer_str:
                    # rsplit一次即可把底部剩余全部分成两段，_ 忽略前面的所有内容，只取 last_line
                    *_, last_line = buffer_str.strip().rsplit("\n", 1)
                    # 清空 buffer，避免下次又重复处理前面内容
                    buffer_str = ""
                    # print(f"Received: {line}")
                    parts = last_line.split()

                    if len(parts) < 3:
                        continue
                    if len(parts[2]) < 21:
                        continue
                    can_id = parts[1].split(":")[1]
                    raw_data = parts[2].split(":")[1]
                    parsed_data = self.parse_message(can_id, raw_data)
                    with self.lock:
                        for key, value in parsed_data.items():
                            self.data[key] = value

            except socket.error as e:
                # print(f"Socket read error: {e}")
                # traceback.print_exc()
                continue
            except Exception as e:
                print(f"Error parsing message: {e}")
                print(parts)
                traceback.print_exc()

                continue

            time.sleep(0.01)

        client.close()

    def _serial_reader(self):
        """
        从串口读取最新的一行 IMU 数据并更新到内存中，采用非阻塞方式，忽略除最后一行之外的所有行。
        """
        buffer_str = ""
        while self.running:
            try:
                n = self.sp.in_waiting
                if n > 0:
                    # 读取所有可用数据并转为字符串
                    raw_data = self.sp.read(n).decode(errors="ignore")
                    buffer_str += raw_data
                    # 如果出现换行，则只解析最后一行
                    if "\n" in buffer_str:
                        # 使用 rsplit 将缓冲区按行分割，只取最后一行
                        *_, last_line = buffer_str.strip().rsplit("\n", 1)
                        # 清空 buffer，让下次读取时不重复处理老数据
                        buffer_str = ""
                        imu_data = last_line.split(",")

                        if len(imu_data) == 16:
                            imu_data = {
                                "heading": float(imu_data[3]),
                                "pitch": float(imu_data[4]),
                                "roll": float(imu_data[5]),
                                "lat": float(imu_data[6]),
                                "lon": float(imu_data[7]),
                                "alti": float(imu_data[8]),
                                "ve": float(imu_data[9]),
                                "vn": float(imu_data[10]),
                                "vu": float(imu_data[11]),
                            }
                            # vx = vn * math.cos(heading_rad) + ve * math.sin(heading_rad)
                            # vy = vn * math.sin(heading_rad) - ve * math.cos(heading_rad)
                            vx = imu_data["vn"] * math.cos(
                                math.radians(imu_data["heading"])
                            ) + imu_data["ve"] * math.sin(
                                math.radians(imu_data["heading"])
                            )
                            vy = imu_data["vn"] * math.sin(
                                math.radians(imu_data["heading"])
                            ) - imu_data["ve"] * math.cos(
                                math.radians(imu_data["heading"])
                            )
                            imu_data["vx"] = vx
                            imu_data["vy"] = vy
                            with self.lock:
                                self.data.update(imu_data)
            except Exception as e:
                print(f"Serial read error: {e}")
                traceback.print_exc()
            # time.sleep(0.01)

    def stop(self):
        """
        停止所有后台线程。
        """
        self.running = False
        self.keyboard_poller.stop()
        self.socket_thread.join()
        self.serial_thread.join()
        self.keyboard_poller.join()

    def is_running(self):
        """
        检查程序是否应该继续运行
        """
        return self.keyboard_poller.running

    @property
    def target_speed(self):
        with self.lock:
            return self._target_speed

    @target_speed.setter
    def target_speed(self, speed):
        with self.lock:
            self._target_speed = speed

    @property
    def wheel_angle(self):
        with self.lock:
            return self._target_speed

    @wheel_angle.setter
    def wheel_angle(self, limit):
        with self.lock:
            self._wheel_angle = limit

    @property
    def vehicle_speed(self):
        with self.lock:
            return self.data.get("vehicle_speed", 0) * 5

    @property
    def steering_angle(self):
        with self.lock:
            return self.data.get("steering_angle", 0)

    @property
    def battery(self):
        with self.lock:
            return self.data.get("battery", 0)

    @property
    def heading(self):
        with self.lock:
            return self.data.get("heading", 0)

    @property
    def postion(self):
        with self.lock:
            return [self.data.get("lat", 32.199403), self.data.get("lon", 119.515293)]

    @property
    def vector_velocity(self):
        vx = self.data.get("vx", 0)
        vy = self.data.get("vy", 0)
        if abs(vy) < 0.2:
            vy = 0
        else:
            vy = vy - 0.03
        with self.lock:
            return [vx * 5, vy]


####### TEST #######
if __name__ == "__main__":
    print("程序已启动，按 'q' 键退出...")

    # 将标准输入设置为非阻塞模式
    import os
    import termios
    import atexit

    # 保存终端设置
    old_settings = termios.tcgetattr(sys.stdin)

    def cleanup():
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    # 注册退出时的清理函数
    atexit.register(cleanup)

    # 设置终端为非阻塞模式
    tty = termios.tcgetattr(sys.stdin)
    tty[3] = tty[3] & ~(termios.ICANON | termios.ECHO)
    termios.tcsetattr(sys.stdin, termios.TCSANOW, tty)

    env = Env()

    try:
        while env.is_running():
            speed = env.vehicle_speed
            angle = env.steering_angle
            battery = env.battery
            heading = env.heading
            position = env.postion
            vector_velocity = env.vector_velocity
            print(
                f"Vehicle Speed: {speed} km/h, Steering Angle: {angle}°, battery: {battery}V, heading: {heading}°, position: {position}, vector_velocity: {vector_velocity}"
            )
            time.sleep(0.01)
    except Exception as e:
        print(f"发生错误: {e}")
        traceback.print_exc()
    finally:
        print("\n正在退出程序...")
        env.stop()
        cleanup()
