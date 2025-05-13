# arcos_protocol.py

import struct
from old.arCosEnum import *

class ArcosProtocol:
    SYNC_BYTES = bytes([SYNC0, SYNC1])

    def __init__(self, sock):
        self.sock = sock

    def _build_packet(self, command, data=b''):
        length = 1 + len(data)
        body = bytes([length]) + bytes([command]) + data
        checksum = sum(body) & 0xFF
        return self.SYNC_BYTES + body + bytes([checksum])

    def _send_packet(self, command, data=b''):
        packet = self._build_packet(command, data)
        self.sock.write(packet)

    def _receive_packet(self):
        while True:
            b = self.sock.read(1)
            if b == bytes([SYNC0]) and self.sock.read(1) == bytes([SYNC1]):
                break
        length_byte = self.sock.read(1)
        if not length_byte:
            return None
        length = length_byte[0]
        payload = self.sock.read(length + 1)  # command + data + checksum
        return length_byte + payload

    # === High-level commands ===
    def send_velocity(self, left_vel, right_vel):
        data = struct.pack('>hh', left_vel, right_vel)
        self._send_packet(VEL2, data)

    def move_distance(self, mm):
        data = struct.pack('>h', mm)
        self._send_packet(MOVE_DISTANCE, data)

    def rotate_angle(self, deg):
        data = struct.pack('>h', deg)
        self._send_packet(ROTATE, data)

    def enable_motors(self):
        self._send_packet(ENABLE)

    def disable_motors(self):
        self._send_packet(DISABLE)

    def stop(self):
        self._send_packet(STOP)

    def get_pose(self):
        self._send_packet(GET_POSE)

    def send_heartbeat(self):
        self._send_packet(HEARTBEAT)
