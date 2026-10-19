#!/usr/bin/env python3
"""
3-DOF Serial Bridge Node
Subscribes to ROS 2 /joint_states and streams the 3 servo angles to Arduino via Serial.
Use this when controlling via ROS 2 CLI or joint_state_publisher_gui.
"""

import math
import glob
import sys
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState

RATE_HZ = 25.0
JOINTS = ['joint1', 'joint2', 'joint3']

class Bridge3DOF(Node):
    def __init__(self, port=None, baud=115200):
        super().__init__('three_dof_bridge')

        if port is None:
            ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
            if not ports:
                self.get_logger().error("No Arduino serial port found! Check USB cable.")
                raise RuntimeError("No serial port found")
            port = ports[0]

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {port} @ {baud} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Cannot open serial port {port}: {e}")
            raise

        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 10)
        self.last_angles = None
        self.last_send_time = self.get_clock().now()
        self.min_dt = 1.0 / RATE_HZ

    def cb(self, msg):
        angles = []
        for joint in JOINTS:
            if joint in msg.name:
                rad = msg.position[msg.name.index(joint)]
                # Radian (-1.57..+1.57) to Degree (0..180)
                deg = int(round(math.degrees(rad) + 90.0))
                deg = max(0, min(180, deg))
                angles.append(deg)
            else:
                angles.append(90)  # default center

        now = self.get_clock().now()
        dt = (now - self.last_send_time).nanoseconds * 1e-9

        if angles != self.last_angles and dt >= self.min_dt:
            cmd = f"<{angles[0]},{angles[1]},{angles[2]}>\n"
            try:
                self.ser.write(cmd.encode())
                self.last_angles = angles
                self.last_send_time = now
                self.get_logger().info(f"Sent: {cmd.strip()}")
            except Exception as e:
                self.get_logger().error(f"Serial write error: {e}")

def main(args=None):
    rclpy.init(args=args)
    port = sys.argv[1] if len(sys.argv) > 1 else None
    try:
        node = Bridge3DOF(port)
        rclpy.spin(node)
    except Exception as e:
        print(f"Bridge exit: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
