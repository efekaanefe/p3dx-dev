import serial

class P3DXRobot:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1):
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

    def send_velocity(self, trans_vel=0, rot_vel=0):
        """
        Send velocity command in mm/s and deg/s.
        """
        if self.serial_conn and self.serial_conn.is_open:
            cmd = f"v {trans_vel} {rot_vel}\r"
            self.serial_conn.write(cmd.encode('ascii'))
            print(f"[→] Sent: {cmd.strip()}")
        else:
            print("[!] Not connected.")

    def read_response(self):
        """
        Read a single line from the robot (if available).
        """
        if self.serial_conn and self.serial_conn.is_open:
            if self.serial_conn.in_waiting:
                response = self.serial_conn.readline().decode('ascii').strip()
                print(f"[←] Received: {response}")
                return response
        return None

    def disconnect(self):
        if self.serial_conn:
            self.serial_conn.close()
            print("[x] Disconnected.")

    def runRobot(self,func):
        """
        This is the default run function that needs to be called to make sure no essential functions are forgotten
        :param func: Name of the function that takes our robot as a parameter
        :return: nothing
        """
        if self.connect():
            func(self)
            self.disconnect()
