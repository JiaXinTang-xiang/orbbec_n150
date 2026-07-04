"""
N150 (Jetson) ←→ STM32 UART6 串口通信
协议与 STM32 uart6_comm.c 对齐

帧格式: [0xAA] [CMD] [DATA...] [CRC]
  SOF: 0xAA
  CRC: XOR of all bytes from CMD to end of DATA

命令:
  0x01 CHASSIS_VEL   — 3×fp32: vx, vy, vw
  0x02 DRIVER_ANGLE  — u8(motor_id) + fp32(angle)
  0x03 RELAY          — u8: bit0~7=继电器1~8
  0x05 ECHO           — [len:1B] [data...]
  0x06 DRIVER_STOP    — u8(motor_id)

STM32 → N150 反馈帧 (每1ms, 33 bytes):
  [0xAA] [0x81] [x:i32] [y:i32] [w:i32] [relay:u8] [angle1:fp32] [angle7:fp32] [angle5:fp32] [angle8:fp32] [CRC]
"""

import struct
import serial
import time
from typing import Optional


# ================================================================
# 协议常量
# ================================================================
SOF            = 0xAA
CMD_CHASSIS    = 0x01
CMD_DRIVER     = 0x02
CMD_RELAY      = 0x03
CMD_ECHO       = 0x05
CMD_DRV_STOP   = 0x06

FB_CMD         = 0x81
ECHO_REPLY     = 0x85
FB_FRAME_LEN   = 33


class N150Serial:
    """Jetson N150 UART6 串口通信"""

    def __init__(self, port: str = "/dev/ttyTHS0", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self._relay_state = 0x00

    # ==================== CRC ====================
    @staticmethod
    def _crc(data: bytes) -> int:
        c = 0
        for b in data:
            c ^= b
        return c & 0xFF

    # ==================== 组帧 ====================
    def _frame(self, cmd: int, data: bytes = b"") -> bytes:
        payload = bytes([cmd]) + data
        crc = self._crc(payload)
        return bytes([SOF]) + payload + bytes([crc])

    # ==================== 发送接口 ====================
    def chassis_vel(self, vx: float, vy: float, vw: float) -> bool:
        """底盘速度控制"""
        data = struct.pack('<fff', vx, vy, vw)
        return self._write(self._frame(CMD_CHASSIS, data))

    def driver_angle(self, motor_id: int, angle: float) -> bool:
        """驱动器目标角度 (motor_id: 1~8)"""
        data = bytes([motor_id & 0xFF]) + struct.pack('<f', angle)
        return self._write(self._frame(CMD_DRIVER, data))

    def driver_stop(self, motor_id: int) -> bool:
        """停止指定驱动器"""
        return self._write(self._frame(CMD_DRV_STOP, bytes([motor_id & 0xFF])))

    def relay(self, flags: int) -> bool:
        """全量设置8路继电器 (bit0~7)"""
        self._relay_state = flags & 0xFF
        return self._write(self._frame(CMD_RELAY, bytes([self._relay_state])))

    def relay_on(self, num: int) -> bool:
        """打开第num路继电器 (1~8)"""
        self._relay_state |= (1 << (num - 1))
        return self.relay(self._relay_state)

    def relay_off(self, num: int) -> bool:
        """关闭第num路继电器 (1~8)"""
        self._relay_state &= ~(1 << (num - 1))
        return self.relay(self._relay_state)

    def relay_all_off(self) -> bool:
        return self.relay(0x00)

    def relay_all_on(self) -> bool:
        return self.relay(0xFF)

    def echo(self, data: bytes) -> Optional[bytes]:
        """ECHO测试, 返回应答数据"""
        frame = self._frame(CMD_ECHO, bytes([len(data)]) + data)
        if not self._write(frame):
            return None
        expected = 3 + 2 + len(data) + 1
        start = time.time()
        buf = b""
        while time.time() - start < 0.3:
            if self.ser and self.ser.in_waiting:
                buf += self.ser.read(self.ser.in_waiting)
            idx = buf.find(bytes([SOF, ECHO_REPLY]))
            if idx >= 0 and len(buf) >= idx + expected:
                reply = buf[idx:idx + expected]
                if self._crc(reply[:-1]) == reply[-1]:
                    # data starts after SOF+CMD+LEN+'OK' = 5 bytes, ends before CRC
                    return reply[5:-1]
            time.sleep(0.001)
        return None

    # ==================== 串口 ====================
    def open(self, port: Optional[str] = None) -> bool:
        if port:
            self.port = port
        try:
            self.ser = serial.Serial(self.port, self.baudrate,
                                     bytesize=8, parity='N', stopbits=1,
                                     timeout=0.1)
            print(f"N150 串口已打开: {self.port}")
            return True
        except Exception as e:
            print(f"N150 串口打开失败: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("N150 串口已关闭")

    @property
    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def _write(self, frame: bytes) -> bool:
        if self.ser and self.ser.is_open:
            return self.ser.write(frame) == len(frame)
        return False


# ================================================================
# 快速测试
# ================================================================
if __name__ == "__main__":
    n150 = N150Serial(port="/dev/ttyTHS0")
    if n150.open():
        # ECHO 测试
        print("ECHO 测试...")
        reply = n150.echo(b"TEST")
        print(f"  应答: {reply}")

        # 底盘前进
        print("发送底盘速度...")
        n150.chassis_vel(10.0, 0.0, 0.0)

        # 继电器
        print("开关继电器...")
        n150.relay(0x01)   # 开继电器1
        time.sleep(0.5)
        n150.relay(0x00)   # 全关

        n150.close()
