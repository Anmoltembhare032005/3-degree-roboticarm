#!/usr/bin/env python3
"""
3-DOF Robotic Arm Master GUI Controller
Controls:
  1. Gazebo simulation via /arm_controller/joint_trajectory
  2. RViz visualization via /joint_states
  3. Physical Arduino with 3 Servos via Serial (/dev/ttyACM* or /dev/ttyUSB*)
"""

import math
import glob
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

try:
    import serial
except ImportError:
    serial = None

JOINTS = ['joint1', 'joint2', 'joint3']
RAD_LIMIT = 1.5707963  # 90 degrees in radians

class ArmGuiController:
    def __init__(self, root):
        self.root = root
        self.root.title("3-DOF Robot Arm Controller (RViz + Gazebo + Arduino)")
        self.root.geometry("620x720")
        self.root.minsize(550, 650)

        # ROS 2 Node setup
        rclpy.init()
        self.node = Node('three_dof_gui_controller')
        self.js_pub = self.node.create_publisher(JointState, '/joint_states', 10)
        self.traj_pub = self.node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Serial setup
        self.ser = None
        self.serial_port = tk.StringVar(value="/dev/ttyACM0")
        self.baud_rate = 115200
        self.last_send_time = 0
        self.min_send_interval = 0.04  # max 25Hz serial rate

        # Angles in degrees (0 to 180, default home 90)
        self.angle_vars = [
            tk.IntVar(value=90),
            tk.IntVar(value=90),
            tk.IntVar(value=90)
        ]

        self.joint_labels = [
            "Joint 1 (Base / Waist) [Pin 9]",
            "Joint 2 (Shoulder)     [Pin 10]",
            "Joint 3 (Elbow)        [Pin 11]"
        ]

        self._build_ui()
        self._auto_connect_serial()

        # Start ROS spinning in a timer
        self._ros_spin_loop()

    def _build_ui(self):
        style = ttk.Style()
        style.theme_use('clam')

        # Title
        title_frame = tk.Frame(self.root, bg="#2c3e50", pady=10)
        title_frame.pack(fill=tk.X)
        tk.Label(title_frame, text="3-DOF Robotic Arm Control Panel", 
                 font=("Helvetica", 15, "bold"), fg="white", bg="#2c3e50").pack()
        tk.Label(title_frame, text="Synchronized: RViz 2 | Gazebo | Arduino Hardware", 
                 font=("Helvetica", 10), fg="#bdc3c7", bg="#2c3e50").pack()

        # Serial Connection Frame
        conn_frame = ttk.LabelFrame(self.root, text=" Arduino Serial Connection ", padding=10)
        conn_frame.pack(fill=tk.X, padx=15, pady=10)

        tk.Label(conn_frame, text="Serial Port:").grid(row=0, column=0, sticky=tk.W, padx=5)
        
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.serial_port, width=15)
        self._refresh_ports()
        self.port_combo.grid(row=0, column=1, padx=5)

        self.btn_refresh = ttk.Button(conn_frame, text="Scan", command=self._refresh_ports, width=8)
        self.btn_refresh.grid(row=0, column=2, padx=5)

        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self._toggle_serial, width=10)
        self.btn_connect.grid(row=0, column=3, padx=5)

        self.lbl_serial_status = tk.Label(conn_frame, text="Disconnected (Simulation only)", 
                                          fg="orange", font=("Helvetica", 9, "bold"))
        self.lbl_serial_status.grid(row=1, column=0, columnspan=4, sticky=tk.W, pady=(5, 0))

        # Sliders Frame
        sliders_frame = ttk.LabelFrame(self.root, text=" Joint Angle Sliders (0 - 180 deg) ", padding=15)
        sliders_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=5)

        self.sliders = []
        self.lbl_values = []

        colors = ["#3498db", "#e67e22", "#2ecc71"]
        for i in range(3):
            sf = tk.Frame(sliders_frame, pady=8)
            sf.pack(fill=tk.X)

            header_f = tk.Frame(sf)
            header_f.pack(fill=tk.X)

            tk.Label(header_f, text=self.joint_labels[i], font=("Helvetica", 11, "bold"), fg=colors[i]).pack(side=tk.LEFT)
            val_lbl = tk.Label(header_f, text="90 deg (0.00 rad)", font=("Helvetica", 10, "bold"), fg="#333")
            val_lbl.pack(side=tk.RIGHT)
            self.lbl_values.append(val_lbl)

            s = tk.Scale(sf, from_=0, to=180, orient=tk.HORIZONTAL, 
                         variable=self.angle_vars[i], length=520,
                         showvalue=False, command=lambda v, idx=i: self._on_slider_change(idx))
            s.pack(fill=tk.X, pady=4)
            self.sliders.append(s)

        # Presets Frame
        preset_frame = ttk.LabelFrame(self.root, text=" Quick Presets ", padding=10)
        preset_frame.pack(fill=tk.X, padx=15, pady=8)

        btn_grid = tk.Frame(preset_frame)
        btn_grid.pack(fill=tk.X)

        ttk.Button(btn_grid, text="Home (90, 90, 90)", command=lambda: self.set_angles(90, 90, 90)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)
        ttk.Button(btn_grid, text="Upright (90, 90, 0)", command=lambda: self.set_angles(90, 90, 0)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)
        ttk.Button(btn_grid, text="Reach (90, 135, 45)", command=lambda: self.set_angles(90, 135, 45)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)
        ttk.Button(btn_grid, text="Auto Sweep Demo", command=self._start_sweep).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)

        # Status & Log Frame
        log_frame = ttk.LabelFrame(self.root, text=" Serial & Controller Logs ", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=(5, 10))

        self.txt_log = tk.Text(log_frame, height=5, font=("Monospace", 9), bg="#1e1e1e", fg="#00ff66")
        self.txt_log.pack(fill=tk.BOTH, expand=True)

        self._log("Controller initialized. Gazebo & RViz publishers ready.")

    def _log(self, text):
        self.txt_log.insert(tk.END, text + "\n")
        self.txt_log.see(tk.END)

    def _refresh_ports(self):
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        self.port_combo['values'] = ports
        if ports and self.serial_port.get() not in ports:
            self.serial_port.set(ports[0])

    def _auto_connect_serial(self):
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        if ports:
            self.serial_port.set(ports[0])
            self._connect_serial()

    def _connect_serial(self):
        if serial is None:
            self._log("Error: pyserial library not available")
            return
        port = self.serial_port.get()
        try:
            self.ser = serial.Serial(port, self.baud_rate, timeout=0.1)
            self.lbl_serial_status.config(text=f"Connected: {port} @ {self.baud_rate} baud", fg="green")
            self.btn_connect.config(text="Disconnect")
            self._log(f"Arduino connected on {port}")
            # Start background serial reader
            t = threading.Thread(target=self._serial_read_loop, daemon=True)
            t.start()
            # Send initial home position
            self.root.after(1500, self.publish_current_state)
        except Exception as e:
            self.ser = None
            self.lbl_serial_status.config(text=f"Failed to connect to {port}", fg="red")
            self._log(f"Serial connection error: {e}")

    def _disconnect_serial(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.lbl_serial_status.config(text="Disconnected (Simulation only)", fg="orange")
        self.btn_connect.config(text="Connect")
        self._log("Serial disconnected")

    def _toggle_serial(self):
        if self.ser and self.ser.is_open:
            self._disconnect_serial()
        else:
            self._connect_serial()

    def _serial_read_loop(self):
        while self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.root.after(0, lambda l=line: self._log(f"[Arduino] {l}"))
            except Exception:
                break

    def _on_slider_change(self, idx):
        self.publish_current_state()

    def set_angles(self, a1, a2, a3):
        self.angle_vars[0].set(a1)
        self.angle_vars[1].set(a2)
        self.angle_vars[2].set(a3)
        self.publish_current_state()

    def publish_current_state(self):
        a1 = self.angle_vars[0].get()
        a2 = self.angle_vars[1].get()
        a3 = self.angle_vars[2].get()

        # Update Labels: servo degrees (0..180) to robot radians (-1.57..+1.57)
        # rad = (deg - 90) * (pi / 180)
        r1 = math.radians(a1 - 90.0)
        r2 = math.radians(a2 - 90.0)
        r3 = math.radians(a3 - 90.0)

        self.lbl_values[0].config(text=f"{a1:3d} deg ({r1:+.2f} rad)")
        self.lbl_values[1].config(text=f"{a2:3d} deg ({r2:+.2f} rad)")
        self.lbl_values[2].config(text=f"{a3:3d} deg ({r3:+.2f} rad)")

        # 1. Publish to RViz (/joint_states)
        js = JointState()
        js.header.stamp = self.node.get_clock().now().to_msg()
        js.name = JOINTS
        js.position = [r1, r2, r3]
        js.velocity = [0.0, 0.0, 0.0]
        js.effort = [0.0, 0.0, 0.0]
        self.js_pub.publish(js)

        # 2. Publish to Gazebo (/arm_controller/joint_trajectory)
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [r1, r2, r3]
        pt.time_from_start = Duration(sec=0, nanosec=250000000)  # 0.25s
        traj.points = [pt]
        self.traj_pub.publish(traj)

        # 3. Send to Arduino via Serial
        now = time.time()
        if self.ser and self.ser.is_open and (now - self.last_send_time >= self.min_send_interval):
            cmd = f"<{a1},{a2},{a3}>\n"
            try:
                self.ser.write(cmd.encode())
                self.last_send_time = now
            except Exception as e:
                self._log(f"Serial write error: {e}")

    def _start_sweep(self):
        t = threading.Thread(target=self._sweep_thread, daemon=True)
        t.start()

    def _sweep_thread(self):
        self._log("Starting sweep demo...")
        # Step through angles smoothly
        steps = [
            (90, 90, 90),
            (45, 90, 90),
            (135, 90, 90),
            (90, 90, 90),
            (90, 45, 120),
            (90, 135, 60),
            (90, 90, 90)
        ]
        for (t1, t2, t3) in steps:
            c1 = self.angle_vars[0].get()
            c2 = self.angle_vars[1].get()
            c3 = self.angle_vars[2].get()

            num_substeps = 20
            for i in range(1, num_substeps + 1):
                interp1 = int(c1 + (t1 - c1) * (i / num_substeps))
                interp2 = int(c2 + (t2 - c2) * (i / num_substeps))
                interp3 = int(c3 + (t3 - c3) * (i / num_substeps))
                self.root.after(0, lambda a=interp1, b=interp2, c=interp3: self.set_angles(a, b, c))
                time.sleep(0.04)
            time.sleep(0.4)
        self._log("Sweep demo finished.")

    def _ros_spin_loop(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.root.after(20, self._ros_spin_loop)

    def close(self):
        self._disconnect_serial()
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = ArmGuiController(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()

if __name__ == '__main__':
    main()
