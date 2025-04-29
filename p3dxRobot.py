import struct
import serial
import time
from arCosEnum import *  # Assumes constants like ENABLE = 0x83, SYNC0 = 0xFA, etc.

# Add if not already in arCosEnum.py
HEARTBEAT = 0xC9  # This is common for ARCOS, verify with your robot's docs

class P3DXRobot:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None

    def connect(self):
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            if self.serial_conn.is_open:
                print(f"[✓] Connected to {self.port} at {self.baudrate} baud.")
                return True
        except serial.SerialException as e:
            print(f"[✗] Serial connection failed: {e}")
        return False

    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("[x] Disconnected.")

    def build_packet(self, command: int, data: bytes = b'') -> bytes:
        length = 1 + len(data)
        body = bytes([length, command]) + data
        checksum = sum(body) & 0xFF
        return bytes([SYNC0, SYNC1]) + body + bytes([checksum])

    def send_packet(self, command: int, data: bytes = b''):
        if self.serial_conn and self.serial_conn.is_open:
            packet = self.build_packet(command, data)
            self.serial_conn.write(packet)
            print(f"[→] Sent: {packet.hex(' ')}")
        else:
            print("[!] Not connected to serial port.")

    def clamp_velocity(self, vel: int, min_val: int = -500, max_val: int = 500) -> int:
        return max(min_val, min(max_val, vel))

    def send_velocity(self, left_vel: int = 0, right_vel: int = 0):
        left = self.clamp_velocity(left_vel)
        right = self.clamp_velocity(right_vel)
        data = struct.pack('>hh', left, right)
        self.send_packet(VEL2, data)

    def enable_motors(self):
        self.send_packet(ENABLE)

    def disable_motors(self):
        self.send_packet(DISABLE)

    def stop(self):
        self.send_packet(STOP)

    def move_distance(self, mm: int):
        data = struct.pack('>h', mm)
        self.send_packet(MOVE_DISTANCE, data)

    def rotate_angle(self, deg: int):
        data = struct.pack('>h', deg)
        self.send_packet(ROTATE, data)

    def request_pose(self):
        self.send_packet(GET_POSE)

    def send_heartbeat(self):
        self.send_packet(HEARTBEAT)

    def read_response(self):
        if not self.serial_conn or not self.serial_conn.is_open:
            return None

        while True:
            b1 = self.serial_conn.read(1)
            if b1 == bytes([SYNC0]):
                b2 = self.serial_conn.read(1)
                if b2 == bytes([SYNC1]):
                    break
            if b1 == b'':
                return None

        length_byte = self.serial_conn.read(1)
        if not length_byte:
            return None
        length = length_byte[0]

        payload = self.serial_conn.read(length + 1)
        if len(payload) != length + 1:
            print("[!] Incomplete payload received.")
            return None

        command = payload[0]
        data = payload[1:-1]
        received_checksum = payload[-1]
        checksum_calc = (length + sum(payload[:-1])) & 0xFF

        if received_checksum != checksum_calc:
            print(f"[!] Checksum mismatch: got {received_checksum:#02x}, expected {checksum_calc:#02x}")
            return None

        return command, data

    def move_with_time(self, left_vel: int, right_vel: int, duration_sec: float, refresh_rate: float = 0.2):
        t_start = time.time()
        while time.time() - t_start < duration_sec:
            self.send_velocity(left_vel, right_vel)
            self.send_heartbeat()  # Keep motors alive
            time.sleep(refresh_rate)
        self.send_velocity(0, 0)

    def runRobot(self, func):
        if self.connect():
            try:
                func(self)
            finally:
                self.disconnect()
