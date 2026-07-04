"""
视觉→STM32 USB 串口通信模块
协议: 0xA5 0x5A + payload(14B) + xor(1B) = 17 bytes

payload (14 bytes, 小端):
  [0]     class     uint8   类别ID
  [1-2]   cx        uint16  像素X     (0=无目标)
  [3-4]   cy        uint16  像素Y     (0=无目标)
  [5-6]   x_mm      int16   坐标X(mm) = 米×1000
  [7-8]   y_mm      int16   坐标Y(mm)
  [9-10]  z_mm      int16   坐标Z(mm)
  [11-12] dist_mm   uint16  距离(mm) = 米×1000, 0xFFFF=无深度
  [13]    reserved  uint8   保留

32 端判断:
  cx==0 && cy==0     → 无目标
  dist_mm == 0xFFFF  → 有目标无深度, 用 cx/cy 追方向
  其他               → 全数据有效
"""

import struct
import serial
import threading

SYNC1, SYNC2 = 0xA5, 0x5A
NO_DEPTH = 0xFFFF


def pack_frame(payload: bytes) -> bytes:
    frame = struct.pack('BB', SYNC1, SYNC2) + payload
    c = 0
    for b in frame:
        c ^= b
    return frame + bytes([c])


def pack_vision(has_target: bool, class_id: int,
                cx: int, cy: int,
                x_m: float, y_m: float, z_m: float, dist_m: float) -> bytes:
    if has_target:
        dist = NO_DEPTH if dist_m <= 0 else int(dist_m * 1000)
    else:
        cx = cy = 0
        x_m = y_m = z_m = 0
        dist = NO_DEPTH

    payload = struct.pack('<B', class_id & 0xff)
    payload += struct.pack('<HH', cx & 0xffff, cy & 0xffff)
    payload += struct.pack('<hhhH',
        int(x_m * 1000), int(y_m * 1000), int(z_m * 1000), dist)
    payload += b'\x00'  # reserved
    return pack_frame(payload)


class VisionSerial:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self._ser = None
        self._lock = threading.Lock()
        self._reconnect_skip = 0  # 重连冷却计数器

    def _do_open(self) -> bool:
        try:
            self._ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"[VisionSerial] 已连接 {self.port}")
            return True
        except serial.SerialException:
            return False

    def open(self) -> bool:
        return self._do_open()

    def _try_reconnect(self):
        """断线后尝试重连，每秒最多试一次"""
        if self._reconnect_skip > 0:
            self._reconnect_skip -= 1
            return
        self._reconnect_skip = 30  # 30 帧 ≈ 1 秒后再试
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        if self._do_open():
            print("[VisionSerial] 已重连")
        else:
            self._reconnect_skip = 60  # 失败则等 2 秒

    def send_vision(self, detections: list):
        with self._lock:
            if self._ser is None or not self._ser.is_open:
                self._try_reconnect()
                if self._ser is None:
                    return

            if detections:
                d = detections[0]
                cx, cy = d['center']
                dist = d.get('distance_m', 0)
                data = pack_vision(
                    has_target=True,
                    class_id=d.get('class_id', 0),
                    cx=cx, cy=cy,
                    x_m=d.get('x_m', 0),
                    y_m=d.get('y_m', 0),
                    z_m=d.get('z_m', 0),
                    dist_m=dist,
                )
            else:
                data = pack_vision(False, 0, 0, 0, 0, 0, 0, 0)
            try:
                self._ser.write(data)
            except serial.SerialException:
                self._ser = None  # 触发下次重连

    def read_feedback(self):
        """非阻塞读取32发来的反馈帧

        支持两种格式（4字节）:
          [0xAA][cmd][val][CRC]  CRC = XOR(0xAA, cmd, val)
          [0x26][0x01][lo][hi]   NRF遥控数据帧，表示32在线
        只要有数据就认为32在回复
        """
        frames = []
        if self._ser is None or not self._ser.is_open:
            return frames
        with self._lock:
            self._ser.timeout = 0
            raw = b''
            while True:
                b = self._ser.read(1)
                if not b:
                    break
                raw += b
            # 用最后收到的4字节帧判断
            if len(raw) >= 4:
                # 找最后一个 0xAA 或 0x26 帧头
                for i in range(len(raw) - 3, -1, -1):
                    h = raw[i]
                    if h in (0xAA, 0x26) and i + 3 < len(raw):
                        cmd = raw[i + 1]
                        val = raw[i + 2]
                        crc = raw[i + 3]
                        if h == 0xAA and crc == (0xAA ^ cmd ^ val):
                            frames.append({'cmd': cmd, 'flags': val})
                            break
                        elif h == 0x26:
                            # 0x26 帧头：只要有数据就认为32在线
                            frames.append({'cmd': cmd, 'flags': val | 0x04})
                            break
            self._ser.timeout = 0.1
        return frames

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
