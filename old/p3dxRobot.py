from arCosProtocol import ArcosProtocol
import serial
import struct
import time
import threading



class P3DXRobot:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.protocol = None
        self.heartbeat_thread = None
        self.heartbeat_running = False

    def _heartbeat_loop(self, interval=1.0):
        while self.heartbeat_running:
            try:
                self.protocol.send_heartbeat()
                time.sleep(interval)
            except Exception as e:
                print(f"[!] Heartbeat error: {e}")
                break

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
                print(f"[✓] Connected to {self.port}")
                self.protocol = ArcosProtocol(self.serial_conn)
                self.protocol.enable_motors()
                self.start_heartbeat()
                return True
        except serial.SerialException as e:
            print(f"[✗] Serial connection failed: {e}")
        return False

    def disconnect(self):
        self.stop_heartbeat()
        if self.protocol:
            try:
                self.protocol.stop()
            except:
                pass
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("[x] Disconnected.")

    def send_velocity(self, left_vel: int = 0, right_vel: int = 0):
        self.protocol.send_velocity(left_vel, right_vel)

    def move_distance(self, mm: int):
        self.protocol.move_distance(mm)

    def rotate_angle(self, deg: int):
        self.protocol.rotate_angle(deg)

    def stop(self):
        self.protocol.stop()

    def enable_motors(self):
        self.protocol.enable_motors()

    def disable_motors(self):
        self.protocol.disable_motors()

    def request_pose(self):
        self.protocol.get_pose()

    def move_with_time(self, left_vel: int, right_vel: int, duration_sec: float, refresh_rate: float = 0.2):
        t_start = time.time()
        while time.time() - t_start < duration_sec:
            self.send_velocity(left_vel, right_vel)
            self.protocol.send_heartbeat()
            time.sleep(refresh_rate)
        self.send_velocity(0, 0)

    def runRobot(self, func):
        if self.connect():
            try:
                func(self)
            finally:
                self.disconnect()
