#!/usr/bin/env python3
"""
Mecanum Robot Teleop GUI
Publishes geometry_msgs/Twist to /cmd_vel  (or /cmd_vel_nav when nav2 is running)

Usage:
  ros2 run mecanum4_description teleop_gui.py                          # plain sim
  ros2 run mecanum4_description teleop_gui.py --ros-args -p topic:=/cmd_vel_nav  # with nav2
"""

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TeleopGUI(Node):
    def __init__(self):
        super().__init__("teleop_gui")
        self.declare_parameter("topic", "/cmd_vel")
        topic = self.get_parameter("topic").get_parameter_value().string_value
        self.get_logger().info(f"Publishing cmd_vel to: {topic}")
        self._pub = self.create_publisher(Twist, topic, 10)
        self._topic = topic
        self._timer = self.create_timer(0.05, self._publish_cmd)  # 20 Hz

        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._active_keys: set[str] = set()

    # ── called by GUI buttons (press / release) ───────────────────────────────
    def set_motion(self, vx, vy, wz):
        # explicit float() — Twist msg rejects ints strictly
        self._vx = float(vx)
        self._vy = float(vy)
        self._wz = float(wz)

    def stop(self):
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

    # ── keyboard accumulation ─────────────────────────────────────────────────
    def key_press(self, key: str, speed: float, angular: float):
        self._active_keys.add(key)
        self._recalc(speed, angular)

    def key_release(self, key: str, speed: float, angular: float):
        self._active_keys.discard(key)
        self._recalc(speed, angular)

    def _recalc(self, speed: float, angular: float):
        vx = vy = wz = 0.0
        if "w" in self._active_keys or "Up" in self._active_keys:
            vx += speed
        if "s" in self._active_keys or "Down" in self._active_keys:
            vx -= speed
        if "a" in self._active_keys or "Left" in self._active_keys:
            vy += speed
        if "d" in self._active_keys or "Right" in self._active_keys:
            vy -= speed
        if "q" in self._active_keys:
            wz += angular
        if "e" in self._active_keys:
            wz -= angular
        self._vx, self._vy, self._wz = vx, vy, wz

    # ── ROS timer ─────────────────────────────────────────────────────────────
    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x = float(self._vx)
        msg.linear.y = float(self._vy)
        msg.angular.z = float(self._wz)
        self._pub.publish(msg)


# ── GUI ───────────────────────────────────────────────────────────────────────

BTN_STYLE = {
    "width": 5,
    "height": 2,
    "font": ("Helvetica", 14, "bold"),
    "relief": "raised",
    "bd": 3,
}

STOP_STYLE = {
    "width": 5,
    "height": 2,
    "font": ("Helvetica", 14, "bold"),
    "bg": "#e74c3c",
    "activebackground": "#c0392b",
    "fg": "white",
    "relief": "raised",
    "bd": 3,
}


def build_gui(node: TeleopGUI):
    root = tk.Tk()
    root.title(f"Mecanum Teleop  [{node._topic}]")
    root.resizable(False, False)
    root.configure(bg="#2c3e50")

    # ── speed sliders ─────────────────────────────────────────────────────────
    slider_frame = tk.Frame(root, bg="#2c3e50", pady=6)
    slider_frame.pack(fill="x", padx=12)

    tk.Label(slider_frame, text="Linear speed (m/s)", bg="#2c3e50", fg="white",
             font=("Helvetica", 10)).grid(row=0, column=0, sticky="w")
    linear_var = tk.DoubleVar(value=0.3)
    linear_sl = ttk.Scale(slider_frame, from_=0.05, to=1.0, variable=linear_var,
                          orient="horizontal", length=200)
    linear_sl.grid(row=0, column=1, padx=6)
    linear_lbl = tk.Label(slider_frame, textvariable=linear_var, width=4,
                           bg="#2c3e50", fg="#1abc9c", font=("Helvetica", 10))
    linear_lbl.grid(row=0, column=2)

    tk.Label(slider_frame, text="Angular speed (rad/s)", bg="#2c3e50", fg="white",
             font=("Helvetica", 10)).grid(row=1, column=0, sticky="w")
    angular_var = tk.DoubleVar(value=0.8)
    angular_sl = ttk.Scale(slider_frame, from_=0.1, to=2.0, variable=angular_var,
                           orient="horizontal", length=200)
    angular_sl.grid(row=1, column=1, padx=6)
    angular_lbl = tk.Label(slider_frame, textvariable=angular_var, width=4,
                            bg="#2c3e50", fg="#1abc9c", font=("Helvetica", 10))
    angular_lbl.grid(row=1, column=2)

    # round display values
    def fmt_slider(*_):
        linear_var.set(round(linear_var.get(), 2))
        angular_var.set(round(angular_var.get(), 2))

    linear_sl.configure(command=fmt_slider)
    angular_sl.configure(command=fmt_slider)

    # ── direction pad ─────────────────────────────────────────────────────────
    pad = tk.Frame(root, bg="#2c3e50", pady=4)
    pad.pack()

    def spd():
        return linear_var.get()

    def ang():
        return angular_var.get()

    def make_btn(parent, text, row, col, color,
                 press_fn, release_fn=None, **kwargs):
        btn = tk.Button(parent, text=text,
                        bg=color, activebackground=color,
                        fg="white", **BTN_STYLE, **kwargs)
        btn.grid(row=row, column=col, padx=4, pady=4)
        btn.bind("<ButtonPress-1>",   lambda e: press_fn())
        btn.bind("<ButtonRelease-1>", lambda e: (release_fn() if release_fn else node.stop()))
        return btn

    # Forward (↑)
    make_btn(pad, "▲\nFwd",    0, 1, "#2980b9",
             lambda: node.set_motion(spd(), 0, 0))

    # Rotate Left (↺)
    make_btn(pad, "↺\nR.L",   1, 0, "#8e44ad",
             lambda: node.set_motion(0, 0,  ang()))

    # STOP
    stop_btn = tk.Button(pad, text="■\nSTOP", **STOP_STYLE)
    stop_btn.grid(row=1, column=1, padx=4, pady=4)
    stop_btn.bind("<ButtonPress-1>", lambda e: node.stop())

    # Rotate Right (↻)
    make_btn(pad, "↻\nR.R",   1, 2, "#8e44ad",
             lambda: node.set_motion(0, 0, -ang()))

    # Strafe Left (←)
    make_btn(pad, "◄\nLeft",  2, 0, "#27ae60",
             lambda: node.set_motion(0,  spd(), 0))

    # Back (↓)
    make_btn(pad, "▼\nBack",  2, 1, "#2980b9",
             lambda: node.set_motion(-spd(), 0, 0))

    # Strafe Right (→)
    make_btn(pad, "►\nRight", 2, 2, "#27ae60",
             lambda: node.set_motion(0, -spd(), 0))

    # ── keyboard legend ───────────────────────────────────────────────────────
    legend = tk.Frame(root, bg="#2c3e50", pady=4)
    legend.pack(fill="x", padx=12)

    keys_text = (
        "Keyboard:  W/↑ Fwd   S/↓ Back   A/← Strafe-L   D/→ Strafe-R   Q Rot-L   E Rot-R"
    )
    tk.Label(legend, text=keys_text, bg="#2c3e50", fg="#95a5a6",
             font=("Helvetica", 9)).pack()

    # ── status bar ────────────────────────────────────────────────────────────
    status_var = tk.StringVar(value="vx: 0.00  vy: 0.00  ωz: 0.00")
    tk.Label(root, textvariable=status_var, bg="#1a252f", fg="#1abc9c",
             font=("Courier", 10), pady=4).pack(fill="x")

    def update_status():
        status_var.set(
            f"vx: {node._vx:+.2f}  vy: {node._vy:+.2f}  ωz: {node._wz:+.2f}"
        )
        root.after(100, update_status)

    update_status()

    # ── keyboard bindings ─────────────────────────────────────────────────────
    MOVE_KEYS = {"w", "s", "a", "d", "q", "e", "Up", "Down", "Left", "Right"}

    def on_key_press(event):
        k = event.keysym
        if k in MOVE_KEYS:
            node.key_press(k, spd(), ang())

    def on_key_release(event):
        k = event.keysym
        if k in MOVE_KEYS:
            node.key_release(k, spd(), ang())

    root.bind("<KeyPress>",   on_key_press)
    root.bind("<KeyRelease>", on_key_release)
    root.focus_set()

    # ── on close: stop robot ──────────────────────────────────────────────────
    def on_close():
        node.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    return root


def main():
    rclpy.init()
    node = TeleopGUI()

    # spin ROS in background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    root = build_gui(node)
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
