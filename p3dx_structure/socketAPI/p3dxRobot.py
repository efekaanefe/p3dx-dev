import socket
import json
import math
import time
import threading
import keyboard
import tkinter as tk

MAX_LINEAR_SPEED = 6.28
MAX_ANGULAR_SPEED = 1.0


class p3dxRobot:
    def __init__(self, is_simulation=False, host='192.168.40.97', port=9090):
        self.is_simulation = is_simulation
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.curr_loc = [0.0, 0.0]
        self.curr_angle = 0.0

        self.keyboard_active = False
        self.keyboard_thread = None
        self.mode = "Idle"

        self.gui_thread = None
        self.canvas = None
        self.dot = None
        self.heading_line = None
        self.scale = 50  # 1m = 50px

        self.running = True
        self.velocity_thread = threading.Thread(target=self._velocity_loop)
        self.velocity_thread.daemon = True
        self.velocity_thread.start()

        if not self.is_simulation:
            print("Connecting to robot...")
            self.host = host
            self.port = port
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            print(f"Connected to robot at {self.host}:{self.port}")

        self.start_gui()

    def _velocity_loop(self):
        while self.running:
            self.send_curr_vel()
            if self.is_simulation:
                self.update_location_artificially(0.1)
            time.sleep(0.1)

    def send_curr_vel(self):
        cmd = {"linear_x": self.linear_x, "angular_z": self.angular_z}
        if not self.is_simulation:
            try:
                self.sock.sendall(json.dumps(cmd).encode('utf-8'))
                feedback = self.sock.recv(1024).decode()
                print("Feedback from robot:", feedback)
            except Exception as e:
                print("Socket send failed:", e)
        else:
            print(f"Simulated: linear_x={cmd['linear_x']:.2f}, angular_z={cmd['angular_z']:.2f}")

    def update_location_artificially(self, duration):
        dt = 0.1
        steps = int(duration / dt)
        for _ in range(steps):
            self.curr_loc[0] += self.linear_x * math.cos(self.curr_angle) * dt
            self.curr_loc[1] += self.linear_x * math.sin(self.curr_angle) * dt
            self.curr_angle += self.angular_z * dt
            self.update_gui()
            time.sleep(dt)

    def stop(self):
        self.set_velocity(0.0, 0.0)

    def set_velocity(self, linear_x, angular_z):
        self.linear_x = max(min(linear_x, MAX_LINEAR_SPEED), -MAX_LINEAR_SPEED)
        self.angular_z = max(min(angular_z, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)

    def go(self, target_x, target_y):
        self.mode = "Autonomous"
        goal_tolerance = 0.05  # meters
        angle_tolerance = 0.05  # radians

        max_forward_speed = 1.0
        max_turn_speed = MAX_ANGULAR_SPEED * 0.8

        while self.running:
            # Compute vector to goal
            dx = target_x - self.curr_loc[0]
            dy = target_y - self.curr_loc[1]
            distance = math.hypot(dx, dy)

            if distance < goal_tolerance:
                break  # Goal reached

            # Compute angle to target and angle difference
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.curr_angle
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            if abs(angle_diff) > angle_tolerance:
                # Rotate in place toward the goal
                angular_z = max_turn_speed if angle_diff > 0 else -max_turn_speed
                self.set_velocity(0.0, angular_z)
            else:
                # Move forward toward the goal
                self.set_velocity(max_forward_speed, 0.0)

            if self.is_simulation:
                self.update_location_artificially(0.1)
            else:
                time.sleep(0.1)

        self.set_velocity(0.0, 0.0)
        self.mode = "Idle"


    def close(self):
        self.running = False
        self.stop()
        if not self.is_simulation:
            self.sock.close()
        print("Connection closed.")

    def _keyboard_loop(self):
        self.mode = "Manual (Keyboard)"
        while self.keyboard_active:
            linear = 0.0
            angular = 0.0
            if keyboard.is_pressed("up"):
                linear += 1.0
            if keyboard.is_pressed("down"):
                linear -= 1.0
            if keyboard.is_pressed("left") or keyboard.is_pressed("q"):
                angular += MAX_ANGULAR_SPEED * 0.5
            if keyboard.is_pressed("right") or keyboard.is_pressed("e"):
                angular -= MAX_ANGULAR_SPEED * 0.5
            if keyboard.is_pressed("esc"):
                self.stopKeyboard()

            self.set_velocity(linear, angular)
            time.sleep(0.1)

        self.set_velocity(0.0, 0.0)
        self.mode = "Idle"

    def useKeyboard(self):
        if not self.keyboard_active:
            self.keyboard_active = True
            self.keyboard_thread = threading.Thread(target=self._keyboard_loop)
            self.keyboard_thread.start()

    def stopKeyboard(self):
        self.keyboard_active = False
        if self.keyboard_thread:
            self.keyboard_thread.join()

    def start_gui(self):
        self.gui_thread = threading.Thread(target=self._init_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()

    def _init_gui(self):
        self.root = tk.Tk()
        self.root.title("P3DX Robot Control")
        self.root.geometry("950x700")

        self.canvas_frame = tk.Frame(self.root)
        self.canvas_frame.pack(side=tk.LEFT)
        self.control_frame = tk.Frame(self.root, padx=15, pady=15)
        self.control_frame.pack(side=tk.RIGHT, fill=tk.BOTH)

        self.canvas = tk.Canvas(self.canvas_frame, width=600, height=600, bg="white")
        self.canvas.pack()
        self.dot = self.canvas.create_oval(295, 295, 305, 305, fill="red")
        self.heading_line = self.canvas.create_line(300, 300, 310, 300, fill="blue", width=2)

        self.mode_label = tk.Label(self.control_frame, text="Mode: Idle", font=("Arial", 14))
        self.mode_label.pack(anchor="w")

        self.pos_label = tk.Label(self.control_frame, text="Position: (0.00, 0.00)", font=("Arial", 14))
        self.pos_label.pack(anchor="w")

        self.angle_label = tk.Label(self.control_frame, text="Angle: 0°", font=("Arial", 14))
        self.angle_label.pack(anchor="w")

        self.vel_label = tk.Label(self.control_frame, text="Velocity: 0.0 m/s, 0.0 rad/s", font=("Arial", 14))
        self.vel_label.pack(anchor="w")

        self.toggle_btn = tk.Button(self.control_frame, text="Switch to Manual Mode", font=("Arial", 12),
                                    command=self.toggle_mode)
        self.toggle_btn.pack(pady=15)

        tk.Label(self.control_frame, text="Go to (x, y) [m]:", font=("Arial", 12)).pack()
        self.x_entry = tk.Entry(self.control_frame)
        self.x_entry.pack()
        self.y_entry = tk.Entry(self.control_frame)
        self.y_entry.pack()
        tk.Button(self.control_frame, text="Go There!", command=self.gui_go_to, font=("Arial", 12)).pack(pady=5)

        tk.Label(self.control_frame, text="Set Speed (linear, angular):", font=("Arial", 12)).pack()
        self.lin_entry = tk.Entry(self.control_frame)
        self.lin_entry.pack()
        self.ang_entry = tk.Entry(self.control_frame)
        self.ang_entry.pack()
        tk.Button(self.control_frame, text="Set Speed", command=self.gui_set_speed, font=("Arial", 12)).pack(pady=5)

        self.stop_btn = tk.Button(self.control_frame, text="STOP", font=("Arial", 16), fg="white", bg="red",
                                  command=self.stop)
        self.stop_btn.pack(pady=20)

        self._refresh_gui_loop()
        self.root.mainloop()

    def toggle_mode(self):
        if self.keyboard_active:
            self.stopKeyboard()
            self.toggle_btn.config(text="Switch to Manual Mode")
            self.mode = "Autonomous"
        else:
            self.useKeyboard()
            self.toggle_btn.config(text="Switch to Auto Mode")
            self.mode = "Manual (Keyboard)"

    def gui_go_to(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            threading.Thread(target=lambda: self.go(x, y)).start()
        except ValueError:
            print("Invalid input for go-to!")

    def gui_set_speed(self):
        try:
            lin = float(self.lin_entry.get())
            ang = float(self.ang_entry.get())
            self.set_velocity(lin, ang)
            self.mode = "Manual (SetSpeed)"
        except ValueError:
            print("Invalid input for speed!")

    def _refresh_gui_loop(self):
        self.mode_label.config(text=f"Mode: {self.mode}")
        self.pos_label.config(text=f"Position: ({self.curr_loc[0]:.2f}, {self.curr_loc[1]:.2f})")
        self.angle_label.config(text=f"Angle: {math.degrees(self.curr_angle) % 360:.1f}°")
        self.vel_label.config(text=f"Velocity: {self.linear_x:.2f} m/s, {self.angular_z:.2f} rad/s")
        self.update_gui()
        self.root.after(100, self._refresh_gui_loop)

    def update_gui(self):
        if self.canvas:
            x = 300 + self.curr_loc[0] * self.scale
            y = 300 - self.curr_loc[1] * self.scale
            self.canvas.coords(self.dot, x - 5, y - 5, x + 5, y + 5)
            heading_length = 20
            hx = x + heading_length * math.cos(self.curr_angle)
            hy = y - heading_length * math.sin(self.curr_angle)
            self.canvas.coords(self.heading_line, x, y, hx, hy)


def main():
    robot = p3dxRobot(is_simulation=True)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        robot.close()


if __name__ == "__main__":
    main()
