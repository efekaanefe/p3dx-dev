import struct
import serial
import time
from arCosEnum import *  # Assumes constants like ENABLE = 0x83, SYNC0 = 0xFA, etc.
import threading



# Add if not already in arCosEnum.py
HEARTBEAT = 0xC9  # This is common for ARCOS, verify with your robot's docs

class P3DXRobot:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.heartbeat_thread = None
        self.heartbeat_running = False

    def _heartbeat_loop(self, interval=1.0):
        print("[~] Heartbeat thread started.")
        while self.heartbeat_running:
            try:
                self.send_pulse()
                time.sleep(interval)
            except Exception as e:
                print(f"[!] Heartbeat error: {e}")
                break
        print("[~] Heartbeat thread stopped.")

    def start_heartbeat(self, interval=1.0):
        if not self.heartbeat_running:
            self.heartbeat_running = True
            self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, args=(interval,), daemon=True)
            self.heartbeat_thread.start()

    def stop_heartbeat(self):
        self.heartbeat_running = False
        if self.heartbeat_thread:
            self.heartbeat_thread.join(timeout=2)
            self.heartbeat_thread = None

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
                if self.perform_sync_handshake():
                    if self.send_open():
                        self.send_pulse()
                        self.enable_motors()
                        self.start_heartbeat(interval=1.0)  # Start background pulse
                        return True
                    else:
                        print("[✗] OPEN command failed.")
                self.serial_conn.close()
        except serial.SerialException as e:
            print(f"[✗] Serial connection failed: {e}")
        return False

    def send_open(self):
        open_packet = bytes([250, 251, 3, 1, 0, 1])  # Same as SYNC1 structure but now acts as OPEN
        self.serial_conn.write(open_packet)
        print(f"[→] Sent OPEN command: {open_packet.hex(' ')}")

        # Expect an echo of OPEN packet
        echo = self.serial_conn.read(len(open_packet))
        if echo != open_packet:
            print(f"[✗] OPEN command failed. Expected echo: {open_packet.hex(' ')}, got: {echo.hex(' ')}")
            return False
        print("[✓] OPEN command echoed correctly.")
        return True


    def perform_sync_handshake(self):
        sync_packets = [
            bytes([250, 251, 3, 0, 0, 0]),  # SYNC0
            bytes([250, 251, 3, 1, 0, 1]),  # SYNC1
            bytes([250, 251, 3, 2, 0, 2])   # SYNC2
        ]

        for i, packet in enumerate(sync_packets):
            self.serial_conn.write(packet)
            print(f"[→] Sent SYNC{i}: {packet.hex(' ')}")

            echo = self.serial_conn.read(len(packet))
            if echo != packet:
                print(f"[✗] SYNC{i} failed. Expected echo: {packet.hex(' ')}, got: {echo.hex(' ')}")
                return False
            print(f"[✓] SYNC{i} echoed correctly.")

        # After SYNC2, wait for additional configuration strings (name, class, subclass)
        config_data = b""
        t_start = time.time()
        while time.time() - t_start < 2:  # Allow 2 seconds to receive config info
            chunk = self.serial_conn.read(1)
            if not chunk:
                break
            config_data += chunk
            if config_data.count(b'\x00') == 3:  # Expect three NULL-terminated strings
                break

        if config_data:
            parts = config_data.split(b'\x00')
            name = parts[0].decode(errors='ignore')
            robot_class = parts[1].decode(errors='ignore') if len(parts) > 1 else ''
            subclass = parts[2].decode(errors='ignore') if len(parts) > 2 else ''
            print(f"[✓] Robot Info: Name='{name}', Class='{robot_class}', Subclass='{subclass}'")
        else:
            print("[!] No config data received after SYNC2.")

        return True

    def send_pulse(self):
        pulse_packet = bytes([250, 251, 3, 0, 0, 0])
        self.serial_conn.write(pulse_packet)
        print(f"[→] Sent PULSE command: {pulse_packet.hex(' ')}")

    def send_close(self):
        close_packet = bytes([250, 251, 3, 2, 0, 2])
        self.serial_conn.write(close_packet)
        print(f"[→] Sent CLOSE command: {close_packet.hex(' ')}")

    def disconnect(self):
        self.stop_heartbeat()  # <-- stop background thread first
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.send_close()
                time.sleep(0.1)
            except Exception as e:
                print(f"[!] Error sending CLOSE command: {e}")
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

    def parse_sip(self, data: bytes):
        # Example based on known SIP structure. You may need to adjust fields.
        if len(data) < 20:
            print("[!] SIP packet too short.")
            return

        flags, battery, x_pos, y_pos, heading = struct.unpack('>Hhiii', data[:18])
        motors_enabled = bool(flags & 0x01)
        print(
            f"[SIP] Motors: {'ENABLED' if motors_enabled else 'DISABLED'}, Battery: {battery}, Position: ({x_pos}, {y_pos}), Heading: {heading}")


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
