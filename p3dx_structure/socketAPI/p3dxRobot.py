import socket
import json
import math
import time
import threading
from pynput import keyboard
import tkinter as tk

MAX_LINEAR_SPEED = 6.28
MAX_ANGULAR_SPEED = 1.0

class p3dxRobot:
    def __init__(self, is_simulation=False, host='192.168.68.58', port=9090):
        self.is_simulation = is_simulation
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.curr_loc = [0.0, 0.0]
        self.curr_angle = 0.0

        self.force_stop = False
        self.autonomous_thread = None

        self.keyboard_active = False
        self.keyboard_thread = None
        self.mode = "Idle"
        self.key_states = {"w": False, "s": False, "a": False, "d": False}

        self.gui_thread = None
        self.canvas = None
        self.dot = None
        self.heading_line = None
        self.scale = 400  # 1m = 400px

        self.running = True
        self.set_speed_active = False

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
        dt = 0.1
        while self.running:
            self.curr_loc[0] += self.linear_x * math.cos(self.curr_angle) * dt
            self.curr_loc[1] += self.linear_x * math.sin(self.curr_angle) * dt
            self.curr_angle += self.angular_z * dt
            self.update_gui()

            if self.set_speed_active or self.keyboard_active or not self.is_simulation:
                self.send_curr_vel()

            time.sleep(dt)

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

    def stop(self):
        self.force_stop = True
        self.set_speed_active = False
        self.set_velocity(0.0, 0.0)
        self.mode = "Idle"
        print("STOP issued.")

    def set_velocity(self, linear_x, angular_z):
        self.linear_x = max(min(linear_x, MAX_LINEAR_SPEED), -MAX_LINEAR_SPEED)
        self.angular_z = max(min(angular_z, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)

    def go(self, target_x, target_y):
        self.mode = "Autonomous"
        goal_tolerance = 0.05
        angle_tolerance = 0.05
        max_forward_speed = 0.2
        max_turn_speed = MAX_ANGULAR_SPEED * 0.8
        self.force_stop = False

        while self.running and not self.force_stop:
            dx = target_x - self.curr_loc[0]
            dy = target_y - self.curr_loc[1]
            distance = math.hypot(dx, dy)

            if distance < goal_tolerance:
                break

            target_angle = math.atan2(dy, dx)
            angle_diff = (target_angle - self.curr_angle + math.pi) % (2 * math.pi) - math.pi

            if abs(angle_diff) > angle_tolerance:
                angular_z = max_turn_speed if angle_diff > 0 else -max_turn_speed
                self.set_velocity(0.0, angular_z)
            else:
                self.set_velocity(max_forward_speed, 0.0)

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
        self.key_states = {"w": False, "s": False, "a": False, "d": False}

        def on_press(key):
            try:
                k = key.char.lower()
                if k in self.key_states:
                    self.key_states[k] = True
            except AttributeError:
                if key == keyboard.Key.esc:
                    self.stopKeyboard()
                    return False

        def on_release(key):
            try:
                k = key.char.lower()
                if k in self.key_states:
                    self.key_states[k] = False
            except AttributeError:
                pass

        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        while self.keyboard_active:
            linear = 0.0
            angular = 0.0
            if self.key_states["w"]:
                linear += 0.1
            if self.key_states["s"]:
                linear -= 0.1
            if self.key_states["a"]:
                angular += 0.1
            if self.key_states["d"]:
                angular -= 0.1

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
        if self.keyboard_thread and self.keyboard_thread.is_alive():
            self.keyboard_thread.join()


    def start_gui(self):
        self.gui_thread = threading.Thread(target=self._init_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()

    def _init_gui(self):
        self.root = tk.Tk()
        self.root.title("P3DX Robot Control")
        self.root.configure(bg="#2e2e2e")
        self.root.geometry("1450x1100")

        self.scale = 400  # 1 meter = 80 pixels for canvas display

        # Left side - canvas (arena)
        self.canvas_frame = tk.Frame(self.root, bg="#2e2e2e")
        self.canvas_frame.pack(side=tk.LEFT, padx=20, pady=20)

        self.canvas = tk.Canvas(self.canvas_frame, width=800, height=800, bg="#3c3f41", highlightthickness=0)
        self.canvas.pack()
        self.dot = self.canvas.create_oval(390, 390, 410, 410, fill="yellow")  # 20px robot
        self.heading_line = self.canvas.create_line(400, 400, 440, 400, fill="skyblue", width=4)

        # Right side - scrollable controls
        control_canvas = tk.Canvas(self.root, width=900, height=860, bg="#2e2e2e", highlightthickness=0)
        scrollbar = tk.Scrollbar(self.root, orient="vertical", command=control_canvas.yview)
        self.control_frame = tk.Frame(control_canvas, bg="#2e2e2e")

        self.control_frame.bind(
            "<Configure>",
            lambda e: control_canvas.configure(scrollregion=control_canvas.bbox("all"))
        )
        control_canvas.create_window((0, 0), window=self.control_frame, anchor="nw")
        control_canvas.configure(yscrollcommand=scrollbar.set)
        control_canvas.pack(side=tk.RIGHT, fill=tk.Y, expand=False)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Styling helpers
        label_style = {"font": ("Arial", 16), "bg": "#2e2e2e", "fg": "white", "anchor": "w"}
        entry_style = {"font": ("Arial", 15)}

        self.mode_label = tk.Label(self.control_frame, text="Mode: Idle", **label_style)
        self.mode_label.pack(anchor="w", pady=6)

        self.pos_label = tk.Label(self.control_frame, text="Position: (0.00, 0.00)", **label_style)
        self.pos_label.pack(anchor="w", pady=6)

        self.angle_label = tk.Label(self.control_frame, text="Angle: 0°", **label_style)
        self.angle_label.pack(anchor="w", pady=6)

        self.vel_label = tk.Label(self.control_frame, text="Velocity: 0.0 m/s, 0.0 rad/s", **label_style)
        self.vel_label.pack(anchor="w", pady=6)

        self.toggle_btn = tk.Button(self.control_frame, text="Switch to Manual Mode",
                                    font=("Arial", 15), bg="#5c5c5c", fg="white",
                                    activebackground="#7a7a7a", command=self.toggle_mode)
        self.toggle_btn.pack(pady=20, fill=tk.X)

        section_label = lambda txt: tk.Label(self.control_frame, text=txt, **label_style)
        entry = lambda: tk.Entry(self.control_frame, **entry_style)

        section_label("Go to (x, y) [m]:").pack(anchor="w", pady=(12, 0))
        self.x_entry = entry();
        self.x_entry.pack(fill=tk.X, pady=2)
        self.y_entry = entry();
        self.y_entry.pack(fill=tk.X, pady=2)
        tk.Button(self.control_frame, text="Go There!", font=("Arial", 15),
                  command=self.gui_go_to, bg="#5c5c5c", fg="white",
                  activebackground="#7a7a7a").pack(pady=10, fill=tk.X)

        section_label("Set Speed (linear, angular):").pack(anchor="w", pady=(15, 0))
        self.lin_entry = entry();
        self.lin_entry.pack(fill=tk.X, pady=2)
        self.ang_entry = entry();
        self.ang_entry.pack(fill=tk.X, pady=2)
        tk.Button(self.control_frame, text="Set Speed", font=("Arial", 15),
                  command=self.gui_set_speed, bg="#5c5c5c", fg="white",
                  activebackground="#7a7a7a").pack(pady=10, fill=tk.X)

        self.stop_btn = tk.Button(self.control_frame, text="STOP", font=("Arial", 18, "bold"),
                                  fg="white", bg="red", activebackground="darkred",
                                  command=self.stop)
        self.stop_btn.pack(pady=30, fill=tk.X)

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
            if self.autonomous_thread and self.autonomous_thread.is_alive():
                print("Already navigating. Stop first.")
                return
            self.autonomous_thread = threading.Thread(target=lambda: self.go(x, y))
            self.autonomous_thread.start()
        except ValueError:
            print("Invalid input for go-to!")

    def gui_set_speed(self):
        try:
            lin = float(self.lin_entry.get())
            ang = float(self.ang_entry.get())
            self.set_velocity(lin, ang)
            self.set_speed_active = True
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
            x = 400 + self.curr_loc[0] * self.scale
            y = 400 - self.curr_loc[1] * self.scale
            self.canvas.coords(self.dot, x - 10, y - 10, x + 10, y + 10)  # 20px diameter
            heading_length = 40
            hx = x + heading_length * math.cos(self.curr_angle)
            hy = y - heading_length * math.sin(self.curr_angle)
            self.canvas.coords(self.heading_line, x, y, hx, hy)


def main():
    robot = p3dxRobot(is_simulation=False)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        robot.close()

if __name__ == "__main__":
    main()
