#!/usr/bin/env python3

"""
Live Debug Dashboard for LRAUV Altitude Mission
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import csv
import time
from collections import deque
import numpy as np


WINDOW = 500  # number of samples to show


class AltitudeDebugger(Node):

    def __init__(self):
        super().__init__('altitude_debugger')

        self.start_time = time.time()

        # Data buffers
        self.time_buf = deque(maxlen=WINDOW)
        self.alt_error = deque(maxlen=WINDOW)
        self.z_d = deque(maxlen=WINDOW)
        self.slope = deque(maxlen=WINDOW)
        self.lookahead = deque(maxlen=WINDOW)
        self.theta_d = deque(maxlen=WINDOW)
        self.pitch = deque(maxlen=WINDOW)
        self.horiz_fin = deque(maxlen=WINDOW)

        # Subscribers
        self.create_subscription(Float64, '/debug/altitude_error', self.alt_cb, 10)
        self.create_subscription(Float64, '/debug/z_d', self.zd_cb, 10)
        self.create_subscription(Float64, '/debug/slope', self.slope_cb, 10)
        self.create_subscription(Float64, '/debug/lookahead', self.look_cb, 10)
        self.create_subscription(Float64, '/debug/theta_d', self.theta_cb, 10)
        self.create_subscription(Float64, '/debug/pitch', self.pitch_cb, 10)
        self.create_subscription(Float64, '/debug/horiz_fin', self.fin_cb, 10)

        # CSV file
        self.csv_file = open("altitude_debug_log.csv", "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "time", "alt_error", "z_d", "slope",
            "lookahead", "theta_d", "pitch", "horiz_fin"
        ])

        # Setup plot
        plt.ion()
        self.fig, self.ax = plt.subplots(4, 2, figsize=(12, 10))
        self.timer = self.create_timer(0.1, self.update_plot)

    # Callbacks
    def record_time(self):
        return time.time() - self.start_time

    def alt_cb(self, msg):
        t = self.record_time()
        self.time_buf.append(t)
        self.alt_error.append(msg.data)

    def zd_cb(self, msg):
        self.z_d.append(msg.data)

    def slope_cb(self, msg):
        self.slope.append(msg.data)

    def look_cb(self, msg):
        self.lookahead.append(msg.data)

    def theta_cb(self, msg):
        self.theta_d.append(msg.data)

    def pitch_cb(self, msg):
        self.pitch.append(msg.data)

    def fin_cb(self, msg):
        self.horiz_fin.append(msg.data)

        # Write to CSV
        if len(self.time_buf) > 0:
            self.writer.writerow([
                self.time_buf[-1],
                self.alt_error[-1] if self.alt_error else 0,
                self.z_d[-1] if self.z_d else 0,
                self.slope[-1] if self.slope else 0,
                self.lookahead[-1] if self.lookahead else 0,
                self.theta_d[-1] if self.theta_d else 0,
                self.pitch[-1] if self.pitch else 0,
                self.horiz_fin[-1] if self.horiz_fin else 0
            ])

    # Live plotting
    def update_plot(self):

        if len(self.time_buf) < 10:
            return

        t = list(self.time_buf)

        self.ax[0,0].cla()
        self.ax[0,0].plot(t, list(self.alt_error))
        self.ax[0,0].set_title("Altitude Error")

        self.ax[0,1].cla()
        self.ax[0,1].plot(t, list(self.z_d))
        self.ax[0,1].set_title("Desired Depth z_d")

        self.ax[1,0].cla()
        self.ax[1,0].plot(t, list(self.slope))
        self.ax[1,0].set_title("Estimated Slope")

        self.ax[1,1].cla()
        self.ax[1,1].plot(t, list(self.lookahead))
        self.ax[1,1].set_title("Adaptive Lookahead")

        self.ax[2,0].cla()
        self.ax[2,0].plot(t, list(self.theta_d))
        self.ax[2,0].set_title("Desired Pitch θ_d")

        self.ax[2,1].cla()
        self.ax[2,1].plot(t, list(self.pitch))
        self.ax[2,1].set_title("Actual Pitch θ")

        self.ax[3,0].cla()
        self.ax[3,0].plot(t, list(self.horiz_fin))
        self.ax[3,0].set_title("Horizontal Fin")

        self.fig.tight_layout()
        plt.pause(0.001)

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AltitudeDebugger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()