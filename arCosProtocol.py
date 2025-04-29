# arcos_protocol.py

import struct
import time

class ArcosProtocol:
    SYNC_BYTES = b'\xFA\xFB'

    def __init__(self, sock):
        self.sock = sock

    def _build_packet(self, command, data=b''):
        length = 1 + len(data)
        body = bytes([length]) + bytes([command]) + data
        checksum = sum(body) & 0xFF
        return self.SYNC_BYTES + body + bytes([checksum])

    def _send_packet(self, command, data=b''):
        packet = self._build_packet(command, data)
        self.sock.sendall(packet)

    def _receive_packet(self):
        while True:
            b = self.sock.recv(1)
            if b == b'\xFA':
                if self.sock.recv(1) == b'\xFB':
                    break
        length_byte = self.sock.recv(1)
        length = length_byte[0]
        payload = self.sock.recv(length + 1)  # command + data + checksum
        return length_byte + payload

    # === Basic Movement Commands ===
    def send_velocity(self, left_vel, right_vel):
        data = struct.pack('>hh', left_vel, right_vel)
        self._send_packet(0x84, data)

    def move_distance(self, mm):
        data = struct.pack('>h', mm)
        self._send_packet(0x86, data)

    def rotate_angle(self, deg):
        data = struct.pack('>h', deg)
        self._send_packet(0
