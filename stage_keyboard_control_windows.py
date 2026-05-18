"""
stage_keyboard_control_windows.py

用途：
    Windows 控制台上用键盘控制三轴样品台。
    按一下键，走一小步。
    带通信测试、IO 限位检查、停止、软件急停、实时状态刷新、运行时调速、运行时调步长。

运行前：
    py -m pip install -r requirements.txt
或
    python -m pip install -r requirements.txt

运行：
    py stage_keyboard_control_windows.py
或
    python stage_keyboard_control_windows.py

注意：
    软件急停不能替代物理急停。
    第一次真实测试请使用小步长和低速度。
"""

import os
import msvcrt
import threading
import time
from dataclasses import dataclass

import serial
import serial.tools.list_ports


# ==========================
# 用户需要根据现场修改的参数
# ==========================

PORT = ""  # 留空时，程序会让你输入。例如 "COM3" 或 "COM4"

BAUDRATE = 115200
TIMEOUT = 0.25

# 初次测试建议极小。
STEP_PULSES = 10

# 速度百分比。初次测试建议 5 或 10。
SPEED_PERCENT = 5

# 每次运动后，最多等多久等待到位反馈。
MOVE_DONE_TIMEOUT = 2.0

# 最保守限位策略：
# True: 只要 D7 读到任何限位 bit 有效，就禁止所有运动。
# False: 不因为限位 bit 自动禁止运动，但会显示警告。
# 初次测试强烈建议 True。
BLOCK_MOTION_IF_ANY_LIMIT_ACTIVE = True

# 每次运动前是否读取 IO。
CHECK_IO_BEFORE_EACH_MOVE = True


# ==========================
# 协议基础
# ==========================

class Axis:
    X = 0x01
    Y = 0x02
    Z = 0x04
    A = 0x08
    ALL = 0xFF


AXIS_VALUE_TO_NAME = {
    Axis.X: "X",
    Axis.Y: "Y",
    Axis.Z: "Z",
    Axis.A: "A",
    Axis.ALL: "ALL",
}


class Direction:
    POSITIVE = 0x00
    NEGATIVE = 0x01


class StopMode:
    EMERGENCY = 0x49
    DECELERATE = 0x4A


class FunctionCode:
    TEST_CONNECTION = 0x55
    READ_POSITION = 0xCB
    READ_IO = 0xD7
    MOVE_RELATIVE = 0xFA
    STOP = 0xFC
    HOME = 0xD0


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def int_to_4bytes_big_endian(value: int) -> bytes:
    if not (0 <= value <= 0xFFFFFFFF):
        raise ValueError("value 必须在 0 到 0xFFFFFFFF 之间")
    return value.to_bytes(4, byteorder="big", signed=False)


def make_12byte_frame(function_code: int, axis: int = 0x00, data: bytes = b"") -> bytes:
    if len(data) > 6:
        raise ValueError("普通 12 字节帧的数据区最多 6 字节")

    header = bytes([0x3A])
    body = bytes([function_code, axis]) + data.ljust(6, b"\x00")
    check = checksum(header + body)
    return header + body + bytes([check, 0x0D, 0x0A])


def make_test_connection_command() -> bytes:
    return make_12byte_frame(FunctionCode.TEST_CONNECTION, axis=0x00)


def make_read_position_command(axis: int) -> bytes:
    return make_12byte_frame(FunctionCode.READ_POSITION, axis=axis)


def make_read_io_command() -> bytes:
    return make_12byte_frame(FunctionCode.READ_IO, axis=0x00)


def make_move_relative_command(axis: int, direction: int, pulses: int, speed_percent: int) -> bytes:
    """
    相对定位：
        3A FA 轴号 方向 脉冲4字节 速度百分比 校验和 0D 0A
    """
    if direction not in (Direction.POSITIVE, Direction.NEGATIVE):
        raise ValueError("direction 必须是 0 或 1")
    if not (0 <= speed_percent <= 100):
        raise ValueError("speed_percent 必须在 0 到 100 之间")

    data = bytes([direction]) + int_to_4bytes_big_endian(pulses) + bytes([speed_percent])
    return make_12byte_frame(FunctionCode.MOVE_RELATIVE, axis=axis, data=data)


def make_stop_command(axis: int = Axis.ALL, mode: int = StopMode.DECELERATE) -> bytes:
    return make_12byte_frame(FunctionCode.STOP, axis=axis, data=bytes([mode]))


# ==========================
# 反馈解析
# ==========================

@dataclass
class PositionStatus:
    axis: str
    is_running: bool
    sign: str
    pulses: int
    raw_hex: str


@dataclass
class IOStatus:
    home_active_axes: list[str]
    limit_active_bits: list[int]
    input_active_channels: list[int]
    output_active_channels: list[int]
    raw_hex: str


def validate_response_frame(frame: bytes, expected_function_code: int | None = None) -> None:
    if len(frame) != 12:
        raise ValueError(f"反馈帧长度不是 12 字节，实际 {len(frame)}：{hex_bytes(frame)}")
    if frame[0] != 0xA3:
        raise ValueError(f"反馈帧头不是 A3，实际 {frame[0]:02X}")
    if frame[-2:] != b"\x0D\x0A":
        raise ValueError(f"反馈帧尾不是 0D 0A，实际 {hex_bytes(frame[-2:])}")
    if expected_function_code is not None and frame[1] != expected_function_code:
        raise ValueError(f"功能码不匹配，期望 {expected_function_code:02X}，实际 {frame[1]:02X}")

    calculated = checksum(frame[:9])
    received = frame[9]
    if calculated != received:
        raise ValueError(f"校验和不匹配，计算 {calculated:02X}，收到 {received:02X}")


def parse_position_response(frame: bytes) -> PositionStatus:
    validate_response_frame(frame, expected_function_code=FunctionCode.READ_POSITION)

    axis_value = frame[2]
    running_flag = frame[3]
    sign_flag = frame[4]
    pulses = int.from_bytes(frame[5:9], byteorder="big", signed=False)

    return PositionStatus(
        axis=AXIS_VALUE_TO_NAME.get(axis_value, f"UNKNOWN({axis_value:02X})"),
        is_running=(running_flag != 0x00),
        sign="positive" if sign_flag == 0x00 else "negative",
        pulses=pulses,
        raw_hex=hex_bytes(frame),
    )


def active_bits_from_byte(value: int, start_index: int = 1) -> list[int]:
    return [start_index + bit for bit in range(8) if value & (1 << bit)]


def active_bits_from_16bit(value: int, start_index: int = 1) -> list[int]:
    return [start_index + bit for bit in range(16) if value & (1 << bit)]


def parse_io_response(frame: bytes) -> IOStatus:
    validate_response_frame(frame, expected_function_code=FunctionCode.READ_IO)

    home_byte = frame[2]
    limit_16bit = int.from_bytes(frame[3:5], byteorder="big", signed=False)
    input_byte = frame[5]
    output_byte = frame[6]

    home_axes = []
    if home_byte & 0x01:
        home_axes.append("X")
    if home_byte & 0x02:
        home_axes.append("Y")
    if home_byte & 0x04:
        home_axes.append("Z")
    if home_byte & 0x08:
        home_axes.append("A")

    return IOStatus(
        home_active_axes=home_axes,
        limit_active_bits=active_bits_from_16bit(limit_16bit),
        input_active_channels=active_bits_from_byte(input_byte),
        output_active_channels=active_bits_from_byte(output_byte),
        raw_hex=hex_bytes(frame),
    )


# ==========================
# 串口与控制器
# ==========================

def list_serial_ports() -> None:
    ports = list(serial.tools.list_ports.comports())

    print("\n当前检测到的串口：")
    if not ports:
        print("  没有检测到串口设备。")
        return

    for idx, port in enumerate(ports, start=1):
        print(f"  [{idx}] {port.device}")
        print(f"      描述: {port.description}")
        print(f"      硬件: {port.hwid}")


def get_port_from_user() -> str:
    if PORT.strip():
        return PORT.strip()

    list_serial_ports()
    print("\n请输入要使用的串口名。")
    print("Windows 常见格式：COM3、COM4、COM5")
    return input("串口名: ").strip()


class StageController:
    def __init__(self, port: str):
        self.port = port
        self.ser: serial.Serial | None = None
        self.serial_lock = threading.RLock()
        self.state_lock = threading.Lock()
        self.motion_busy = False
        self.last_command_hex = ""
        self.last_response_hex = ""
        self.last_message = ""
        self.last_error = ""
        self.latest_positions: dict[str, PositionStatus | None] = {
            "X": None,
            "Y": None,
            "Z": None,
        }
        self.latest_io_status: IOStatus | None = None
        self.status_thread_running = False
        self.status_thread: threading.Thread | None = None
        self.current_speed_percent = SPEED_PERCENT
        self.step_presets = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000]
        self.current_step_index = (
            self.step_presets.index(STEP_PULSES)
            if STEP_PULSES in self.step_presets
            else 3
        )
        self.current_step_pulses = self.step_presets[self.current_step_index]

    def open(self) -> None:
        self.ser = serial.Serial(
            port=self.port,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self) -> None:
        self.stop_status_thread()
        if self.ser and self.ser.is_open:
            self.ser.close()

    def write(self, command: bytes) -> None:
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("串口未打开")
        with self.serial_lock:
            self.last_command_hex = hex_bytes(command)
            self.ser.write(command)
            self.ser.flush()

    def read_frame(self, timeout: float = 1.0) -> bytes:
        """
        尝试读取一个 12 字节反馈帧。
        """
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("串口未打开")

        with self.serial_lock:
            old_timeout = self.ser.timeout
            self.ser.timeout = timeout
            try:
                frame = self.ser.read(12)
                if frame:
                    self.last_response_hex = hex_bytes(frame)
                return frame
            finally:
                self.ser.timeout = old_timeout

    def send_command(
        self,
        command: bytes,
        expect_response: bool = False,
        timeout: float = 1.0,
        clear_input: bool = True,
    ) -> bytes | None:
        """
        所有串口写入/读取都通过这个函数，并用 serial_lock 保护。
        更新 last_command_hex 和 last_response_hex。
        """
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("串口未打开")

        with self.serial_lock:
            if clear_input:
                self.ser.reset_input_buffer()
            self.write(command)
            self.last_response_hex = ""
            if expect_response:
                time.sleep(0.05)
                return self.read_frame(timeout=timeout)
            return None

    def test_connection(self) -> bool:
        expected = bytes.fromhex("A3 AA 00 00 00 00 00 00 00 4D 0D 0A")
        response = self.send_command(make_test_connection_command(), expect_response=True, timeout=1.0)
        return response == expected

    def read_position(self, axis: int, timeout: float = 1.0) -> PositionStatus:
        response = self.send_command(make_read_position_command(axis), expect_response=True, timeout=timeout)
        if not response:
            raise RuntimeError("未收到位置反馈")
        return parse_position_response(response)

    def read_all_positions(self) -> list[PositionStatus]:
        return [
            self.read_position(Axis.X, timeout=0.5),
            self.read_position(Axis.Y, timeout=0.5),
            self.read_position(Axis.Z, timeout=0.5),
        ]

    def read_io(self, timeout: float = 0.5) -> IOStatus:
        response = self.send_command(make_read_io_command(), expect_response=True, timeout=timeout)
        if not response:
            raise RuntimeError("未收到 IO 反馈")
        return parse_io_response(response)

    def get_motion_settings(self) -> tuple[int, int]:
        with self.state_lock:
            return self.current_step_pulses, self.current_speed_percent

    def adjust_speed(self, delta: int) -> str:
        with self.state_lock:
            value = self.current_speed_percent + delta
            value = max(1, min(100, value))
            self.current_speed_percent = value
            self.last_message = f"当前速度百分比：{value}%"
            return self.last_message

    def set_speed_percent(self, value: int) -> str:
        with self.state_lock:
            self.current_speed_percent = max(1, min(100, value))
            self.last_message = f"当前速度百分比：{self.current_speed_percent}%"
            return self.last_message

    def adjust_step_index(self, delta: int) -> str:
        with self.state_lock:
            self.current_step_index = max(0, min(len(self.step_presets) - 1, self.current_step_index + delta))
            self.current_step_pulses = self.step_presets[self.current_step_index]
            self.last_message = f"当前步长：{self.current_step_pulses} pulses"
            return self.last_message

    def set_step_pulses(self, value: int) -> str:
        with self.state_lock:
            if value in self.step_presets:
                self.current_step_pulses = value
                self.current_step_index = self.step_presets.index(value)
            else:
                self.current_step_pulses = value
                if value < self.step_presets[0]:
                    self.current_step_index = 0
                else:
                    self.current_step_index = max(
                        i for i, preset in enumerate(self.step_presets) if preset <= value
                    )
            self.last_message = f"当前步长：{self.current_step_pulses} pulses"
            return self.last_message

    def stop_all(self) -> None:
        with self.state_lock:
            self.motion_busy = False
        self.send_command(make_stop_command(Axis.ALL, StopMode.DECELERATE), expect_response=False)

    def emergency_stop_all(self) -> None:
        with self.state_lock:
            self.motion_busy = False
        self.send_command(make_stop_command(Axis.ALL, StopMode.EMERGENCY), expect_response=False)

    def can_start_motion(self) -> bool:
        with self.state_lock:
            return not self.motion_busy

    def set_motion_busy(self, busy: bool) -> None:
        with self.state_lock:
            self.motion_busy = busy

    def refresh_status(self) -> None:
        positions = self.read_all_positions()
        io_status = self.read_io()
        with self.state_lock:
            self.latest_positions["X"] = positions[0]
            self.latest_positions["Y"] = positions[1]
            self.latest_positions["Z"] = positions[2]
            self.latest_io_status = io_status
            self.last_error = ""

    def status_update_loop(self) -> None:
        while self.status_thread_running:
            try:
                self.refresh_status()
            except Exception as e:
                with self.state_lock:
                    self.last_error = f"状态刷新失败：{e}"
            time.sleep(0.3)

    def start_status_thread(self) -> None:
        if self.status_thread:
            return
        self.status_thread_running = True
        self.status_thread = threading.Thread(target=self.status_update_loop, daemon=True)
        self.status_thread.start()

    def stop_status_thread(self) -> None:
        self.status_thread_running = False
        if self.status_thread:
            self.status_thread.join(timeout=1.0)
            self.status_thread = None

    def move_relative_guarded(self, axis: int, direction: int) -> str:
        if not self.can_start_motion():
            raise RuntimeError("上一个移动尚未完成，已忽略本次移动输入。")

        if CHECK_IO_BEFORE_EACH_MOVE:
            io_status = self.read_io()
            if io_status.limit_active_bits:
                message = (
                    f"检测到限位 bit 有效：{io_status.limit_active_bits}。"
                    f"原始 IO：{io_status.raw_hex}"
                )
                if BLOCK_MOTION_IF_ANY_LIMIT_ACTIVE:
                    raise RuntimeError(message + " 已阻止本次运动。")
                else:
                    with self.state_lock:
                        self.last_error = message

        self.set_motion_busy(True)
        self.motion_thread = threading.Thread(
            target=self._motion_worker,
            args=(axis, direction),
            daemon=True,
        )
        self.motion_thread.start()

        axis_name = AXIS_VALUE_TO_NAME.get(axis, f"UNKNOWN({axis:02X})")
        dir_text = "+" if direction == Direction.POSITIVE else "-"
        return f"已启动 {axis_name}{dir_text} 小步移动，正在等待完成。"

    def _motion_worker(self, axis: int, direction: int) -> None:
        try:
            with self.state_lock:
                pulses = self.current_step_pulses
                speed_percent = self.current_speed_percent

            command = make_move_relative_command(
                axis=axis,
                direction=direction,
                pulses=pulses,
                speed_percent=speed_percent,
            )
            self.send_command(command, expect_response=False)
            done_message = self.wait_motion_done(axis, timeout=MOVE_DONE_TIMEOUT)
            try:
                position = self.read_position(axis, timeout=0.5)
                with self.state_lock:
                    self.latest_positions[position.axis] = position
            except Exception:
                pass
            with self.state_lock:
                self.last_message = (
                    f"轴 {AXIS_VALUE_TO_NAME.get(axis, axis)} 移动完成：{done_message}"
                )
        except Exception as e:
            with self.state_lock:
                self.last_error = f"运动失败：{e}"
                self.last_message = f"运动失败：{e}"
        finally:
            with self.state_lock:
                self.motion_busy = False
            self.motion_thread = None

    def wait_motion_done(self, axis: int, timeout: float) -> str:
        end_time = time.time() + timeout
        while time.time() < end_time:
            frame = self.read_frame(timeout=0.1)
            if frame:
                if len(frame) == 12 and frame[0] == 0xA3 and frame[1] == 0xB5:
                    return f"收到到位反馈：{hex_bytes(frame)}"

            with self.state_lock:
                position = self.latest_positions.get(AXIS_VALUE_TO_NAME.get(axis, "X"))
            if position is not None and not position.is_running:
                return f"轴 {position.axis} 轮询停止，当前脉冲 {position.pulses}"

            time.sleep(0.1)

        self.stop_all()
        return "等待到位反馈超时，已发送停止命令。"


# ==========================
# 显示工具
# ==========================

def clear_console() -> None:
    os.system("cls")


def format_positions(positions: list[PositionStatus]) -> list[str]:
    lines = []
    for p in positions:
        lines.append(
            f"{p.axis}: {'RUN' if p.is_running else 'STOP'} | sign={p.sign} | pulses={p.pulses}"
        )
    return lines


def format_io(io_status: IOStatus) -> list[str]:
    return [
        f"Home active axes : {io_status.home_active_axes if io_status.home_active_axes else 'None'}",
        f"Limit active bits: {io_status.limit_active_bits if io_status.limit_active_bits else 'None'}",
        f"Input active ch. : {io_status.input_active_channels if io_status.input_active_channels else 'None'}",
        f"Output active ch.: {io_status.output_active_channels if io_status.output_active_channels else 'None'}",
        f"Raw IO frame     : {io_status.raw_hex}",
    ]


def console_loop(controller: StageController) -> None:
    last_message = "准备就绪。按 q 退出。"

    while True:
        with controller.state_lock:
            current_step_pulses = controller.current_step_pulses
            current_speed_percent = controller.current_speed_percent
            motion_busy = controller.motion_busy
            positions = [
                controller.latest_positions.get("X"),
                controller.latest_positions.get("Y"),
                controller.latest_positions.get("Z"),
            ]
            io_status = controller.latest_io_status
            last_command_hex = controller.last_command_hex
            last_response_hex = controller.last_response_hex
            last_error = controller.last_error

        clear_console()
        print("三轴样品台 Windows 控制")
        print("=" * 60)
        print(f"串口: {controller.port}")
        print(f"步长: {current_step_pulses} 脉冲 | 速度: {current_speed_percent}% | motion_busy: {'YES' if motion_busy else 'NO'}")
        print(f"限位保护: {'开启' if BLOCK_MOTION_IF_ANY_LIMIT_ACTIVE else '关闭'}")
        print()
        print("键位：")
        print("  a: X-   d: X+")
        print("  s: Y-   w: Y+")
        print("  f: Z-   r: Z+")
        print()
        print("  [: 速度-1%   ]: 速度+1%   1-4: 速度档位")
        print("  -: 步长减小   =/+: 步长增加   5-8: 步长档位")
        print("  p/i: 读取 X/Y/Z 位置和 IO")
        print("  空格: 全部轴减速停止")
        print("  e: 全部轴软件急停")
        print("  q: 停止并退出")
        print()
        print("实时状态：")

        for pos in positions:
            if pos is None:
                print("  未读取到位置。")
            else:
                print(
                    f"  {pos.axis}: {'RUN' if pos.is_running else 'STOP'} | pulses={pos.pulses} | sign={pos.sign}"
                )

        if io_status is None:
            print("IO 状态：未读取")
            print("  限位: -")
            print("  原点: -")
        else:
            print("IO 状态：")
            print(f"  限位 bit: {io_status.limit_active_bits if io_status.limit_active_bits else 'None'}")
            print(f"  原点轴: {io_status.home_active_axes if io_status.home_active_axes else 'None'}")

        print()
        print(f"最近命令: {last_command_hex or 'None'}")
        print(f"最近反馈: {last_response_hex or 'None'}")
        print(f"最近错误: {last_error or 'None'}")
        print()
        print("消息：")
        print(last_message)

        if msvcrt.kbhit():
            key = msvcrt.getwch()
            try:
                if key in ("q", "Q"):
                    controller.stop_all()
                    last_message = "已发送停止命令，退出。"
                    break

                if key == " ":
                    controller.stop_all()
                    last_message = "已发送全部轴减速停止。"
                    continue

                if key in ("e", "E"):
                    controller.emergency_stop_all()
                    last_message = "已发送全部轴软件急停。"
                    continue

                if key in ("p", "P", "i", "I"):
                    controller.refresh_status()
                    last_message = "已手动刷新位置和 IO 状态。"
                    continue

                if key in ("[", "{"):
                    last_message = controller.adjust_speed(-1 if key == "[" else -5)
                    continue

                if key in ("]", "}"):
                    last_message = controller.adjust_speed(1 if key == "]" else 5)
                    continue

                if key == "1":
                    last_message = controller.set_speed_percent(1)
                    continue
                if key == "2":
                    last_message = controller.set_speed_percent(5)
                    continue
                if key == "3":
                    last_message = controller.set_speed_percent(10)
                    continue
                if key == "4":
                    last_message = controller.set_speed_percent(20)
                    continue

                if key in ("-", "_"):
                    last_message = controller.adjust_step_index(-1)
                    continue
                if key in ("=", "+"):
                    last_message = controller.adjust_step_index(1)
                    continue
                if key == "5":
                    last_message = controller.set_step_pulses(1)
                    continue
                if key == "6":
                    last_message = controller.set_step_pulses(10)
                    continue
                if key == "7":
                    last_message = controller.set_step_pulses(100)
                    continue
                if key == "8":
                    last_message = controller.set_step_pulses(1000)
                    continue

                if key in ("a", "A"):
                    last_message = controller.move_relative_guarded(Axis.X, Direction.NEGATIVE)
                    continue
                if key in ("d", "D"):
                    last_message = controller.move_relative_guarded(Axis.X, Direction.POSITIVE)
                    continue
                if key in ("s", "S"):
                    last_message = controller.move_relative_guarded(Axis.Y, Direction.NEGATIVE)
                    continue
                if key in ("w", "W"):
                    last_message = controller.move_relative_guarded(Axis.Y, Direction.POSITIVE)
                    continue
                if key in ("f", "F"):
                    last_message = controller.move_relative_guarded(Axis.Z, Direction.NEGATIVE)
                    continue
                if key in ("r", "R"):
                    last_message = controller.move_relative_guarded(Axis.Z, Direction.POSITIVE)
                    continue

                last_message = f"未识别按键：{key}。"
            except Exception as e:
                try:
                    controller.stop_all()
                except Exception:
                    pass
                last_message = f"错误：{e}，已尝试发送全部轴停止命令。"

        time.sleep(0.2)

    clear_console()
    print("已退出 Windows 控制界面。")
    print(last_message)


def main() -> None:
    print("三轴样品台 Windows 键盘控制程序")
    print("启动前请确认：控制器供电正常、驱动器设置正确、物理急停可用、样品台附近无障碍。")

    port = get_port_from_user()
    controller = StageController(port)

    try:
        print(f"\n打开串口: {port}")
        controller.open()
        print("串口已打开。")

        print("发送通信测试命令...")
        if not controller.test_connection():
            print("通信测试失败。请先不要运动。检查串口名、RS232/RS485、接线、供电。")
            return

        print("通信测试成功。")

        print("读取 IO 状态...")
        controller.refresh_status()
        io_status = controller.latest_io_status
        if io_status is not None:
            print("当前 IO：")
            print("\n".join(format_io(io_status)))
        else:
            print("未能读取 IO 状态，继续进入界面但请注意。")

        if io_status and io_status.limit_active_bits and BLOCK_MOTION_IF_ANY_LIMIT_ACTIVE:
            print("\n检测到限位 bit 有效，当前安全策略会禁止运动。")
            print("请确认限位开关状态、接线和逻辑后再继续。")
            print("如果你确认这是正常状态，可在代码中暂时把 BLOCK_MOTION_IF_ANY_LIMIT_ACTIVE 改为 False。")
            return

        controller.start_status_thread()
        print("\n进入键盘控制界面。")
        time.sleep(1.0)
        console_loop(controller)

    except serial.SerialException as e:
        print(f"串口错误: {e}")

    except KeyboardInterrupt:
        print("\n用户 Ctrl+C 中断，尝试停止全部轴。")
        try:
            controller.stop_all()
        except Exception as stop_error:
            print(f"停止失败: {stop_error}")

    finally:
        try:
            controller.stop_all()
        except Exception:
            pass

        controller.close()
        print("串口已关闭。")


if __name__ == "__main__":
    main()
