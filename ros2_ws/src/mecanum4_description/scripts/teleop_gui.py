#!/usr/bin/env python3
"""
Mecanum Robot Command Center (Teleop GUI)
Features:
  - 8-Way Translation + Rotation
  - Snap-to-Heading (Alignment)
  - Speed Multipliers (Turbo)
  - Keyboard/Mouse Support
"""

import threading
import tkinter as tk
from tkinter import ttk
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class TeleopGUI(Node):
    def __init__(self):
        super().__init__("teleop_gui")
        self.declare_parameter("topic", "/cmd_vel")
        topic = self.get_parameter("topic").get_parameter_value().string_value
        
        self._pub = self.create_publisher(Twist, topic, 10)
        self._topic = topic
        
        # Subscribe to odom for Heading/Alignment feedback
        self._current_yaw = 0.0
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, qos)

        self._timer = self.create_timer(0.05, self._publish_cmd)

        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._active_keys: set[str] = set()
        
        # Alignment state
        self._target_yaw = None
        self._aligning = False

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self._current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def set_motion(self, vx, vy, wz, align_stop=True):
        if align_stop:
            self._target_yaw = None
            self._aligning = False
        self._vx = float(vx)
        self._vy = float(vy)
        self._wz = float(wz)

    def align_to(self, yaw_deg):
        self._target_yaw = math.radians(yaw_deg)
        self._aligning = True
        self.get_logger().info(f"Aligning to {yaw_deg} degrees...")

    def stop(self):
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._target_yaw = None
        self._aligning = False
        self._active_keys.clear()

    def key_press(self, key: str, speed: float, angular: float):
        self._active_keys.add(key)
        self._recalc(speed, angular)

    def key_release(self, key: str, speed: float, angular: float):
        if key in self._active_keys:
            self._active_keys.remove(key)
        self._recalc(speed, angular)

    def _recalc(self, speed: float, angular: float):
        # If manual keys are pressed, cancel alignment
        if self._active_keys:
            self._target_yaw = None
            self._aligning = False

        vx = vy = wz = 0.0
        if "w" in self._active_keys or "Up" in self._active_keys:
            vx += speed
        if "s" in self._active_keys or "Down" in self._active_keys:
            vx -= speed
        if "a" in self._active_keys:
            vy += speed
        if "d" in self._active_keys:
            vy -= speed
        if "q" in self._active_keys or "Left" in self._active_keys:
            wz += angular
        if "e" in self._active_keys or "Right" in self._active_keys:
            wz -= angular
        if "space" in self._active_keys:
            vx = vy = wz = 0.0

        self._vx, self._vy, self._wz = vx, vy, wz

    def _publish_cmd(self):
        msg = Twist()
        
        # Automatic Alignment Logic (P controller)
        if self._aligning and self._target_yaw is not None:
            error = self._target_yaw - self._current_yaw
            # Normalize to [-pi, pi]
            while error > math.pi: error -= 2 * math.pi
            while error < -math.pi: error += 2 * math.pi
            
            if abs(error) < 0.02:
                self._wz = 0.0
                self._aligning = False
                self._target_yaw = None
            else:
                kp = 2.5
                self._wz = kp * error
                # Cap speed
                self._wz = max(-1.5, min(1.5, self._wz))

        msg.linear.x = self._vx
        msg.linear.y = self._vy
        msg.angular.z = self._wz
        self._pub.publish(msg)


# ── GUI Redesign ──────────────────────────────────────────────────────────────

THEME = {
    "bg": "#1e272e",
    "fg": "#d2dae2",
    "accent": "#05c46b",
    "warn": "#ff5e57",
    "btn_fwd": "#0fbcf9",
    "btn_back": "#3c40c6",
    "btn_strafe": "#00d8d6",
    "btn_rotate": "#ef5777",
    "btn_align": "#ffa801",
    "status_bg": "#000000"
}

def build_gui(node: TeleopGUI):
    root = tk.Tk()
    root.title("Mecanum Command Center")
    root.configure(bg=THEME["bg"])
    root.resizable(False, False)

    # ── Speed Controls ────────────────────────────────────────────────────────
    ctrl_frame = tk.Frame(root, bg=THEME["bg"], padx=20, pady=10)
    ctrl_frame.pack(fill="x")

    tk.Label(ctrl_frame, text="TRANSLATION SPEED", bg=THEME["bg"], fg=THEME["fg"], font=("Impact", 10)).pack(anchor="w")
    lin_var = tk.DoubleVar(value=0.5)
    lin_scale = ttk.Scale(ctrl_frame, from_=0.1, to=1.5, variable=lin_var, orient="horizontal")
    lin_scale.pack(fill="x", pady=2)

    tk.Label(ctrl_frame, text="ROTATION SPEED", bg=THEME["bg"], fg=THEME["fg"], font=("Impact", 10)).pack(anchor="w", pady=(10,0))
    ang_var = tk.DoubleVar(value=1.0)
    ang_scale = ttk.Scale(ctrl_frame, from_=0.1, to=3.0, variable=ang_var, orient="horizontal")
    ang_scale.pack(fill="x", pady=2)

    def spd(): return lin_var.get()
    def ang(): return ang_var.get()

    # ── Control Grid ──────────────────────────────────────────────────────────
    main_pad = tk.Frame(root, bg=THEME["bg"], pady=10)
    main_pad.pack()

    btn_opt = {"font": ("Arial Black", 10), "width": 8, "height": 2, "relief": "flat", "bd": 0}

    def mk_btn(txt, r, c, color, vx, vy, wz, **kwargs):
        # Merge btn_opt and kwargs, with kwargs taking precedence
        opts = btn_opt.copy()
        opts.update(kwargs)
        b = tk.Button(main_pad, text=txt, bg=color, activebackground=color, fg="white", **opts)
        b.grid(row=r, column=c, padx=4, pady=4)
        b.bind("<ButtonPress-1>", lambda e: node.set_motion(vx, vy, wz))
        b.bind("<ButtonRelease-1>", lambda e: node.stop())
        return b

    # Layout
    mk_btn("↖", 0, 0, THEME["btn_strafe"],  spd(),  spd(), 0)
    mk_btn("FORWARD", 0, 1, THEME["btn_fwd"],  spd(),  0,     0, width=10)
    mk_btn("↗", 0, 2, THEME["btn_strafe"],  spd(), -spd(), 0)

    mk_btn("STRAFE L", 1, 0, THEME["btn_strafe"], 0, spd(), 0, width=10)
    
    # STOP button fix
    stop_opts = btn_opt.copy()
    stop_opts.update({"width": 10, "bg": THEME["warn"], "activebackground": THEME["warn"]})
    stop_btn = tk.Button(main_pad, text="STOP", fg="white", **stop_opts)
    stop_btn.grid(row=1, column=1)
    stop_btn.bind("<ButtonPress-1>", lambda e: node.stop())
    
    mk_btn("STRAFE R", 1, 2, THEME["btn_strafe"], 0, -spd(), 0, width=10)

    mk_btn("↙", 2, 0, THEME["btn_strafe"], -spd(),  spd(), 0)
    mk_btn("BACKWARD", 2, 1, THEME["btn_back"], -spd(),  0,     0, width=10)
    mk_btn("↘", 2, 2, THEME["btn_strafe"], -spd(), -spd(), 0)

    # ── Rotation & Alignment ──────────────────────────────────────────────────
    extra_frame = tk.Frame(root, bg=THEME["bg"], pady=10)
    extra_frame.pack()

    # Rotation
    rot_frame = tk.LabelFrame(extra_frame, text="ROTATION", bg=THEME["bg"], fg=THEME["fg"], font=("Impact", 8))
    rot_frame.grid(row=0, column=0, padx=10)
    
    # Left Rotation
    rot_l = tk.Button(rot_frame, text="↺ LEFT", bg=THEME["btn_rotate"], fg="white", font=("Arial Black", 8), width=8)
    rot_l.pack(side="left", padx=5, pady=5)
    rot_l.bind("<ButtonPress-1>", lambda e: node.set_motion(0, 0, ang()))
    rot_l.bind("<ButtonRelease-1>", lambda e: node.stop())

    # Right Rotation
    rot_r = tk.Button(rot_frame, text="RIGHT ↻", bg=THEME["btn_rotate"], fg="white", font=("Arial Black", 8), width=8)
    rot_r.pack(side="left", padx=5, pady=5)
    rot_r.bind("<ButtonPress-1>", lambda e: node.set_motion(0, 0, -ang()))
    rot_r.bind("<ButtonRelease-1>", lambda e: node.stop())

    # Alignment (Snap)
    align_frame = tk.LabelFrame(extra_frame, text="ALIGN HEADING", bg=THEME["bg"], fg=THEME["fg"], font=("Impact", 8))
    align_frame.grid(row=0, column=1, padx=10)

    tk.Button(align_frame, text="0°",   width=3, bg=THEME["btn_align"], command=lambda: node.align_to(0)).pack(side="left", padx=2)
    tk.Button(align_frame, text="90°",  width=3, bg=THEME["btn_align"], command=lambda: node.align_to(90)).pack(side="left", padx=2)
    tk.Button(align_frame, text="180°", width=3, bg=THEME["btn_align"], command=lambda: node.align_to(180)).pack(side="left", padx=2)
    tk.Button(align_frame, text="-90°", width=3, bg=THEME["btn_align"], command=lambda: node.align_to(-90)).pack(side="left", padx=2)

    # ── Status Display ────────────────────────────────────────────────────────
    stat_frame = tk.Frame(root, bg=THEME["status_bg"], pady=5)
    stat_frame.pack(fill="x")

    v_stat = tk.StringVar(value="V: 0.0, 0.0  W: 0.0")
    h_stat = tk.StringVar(value="HEADING: 0.0°")

    tk.Label(stat_frame, textvariable=v_stat, bg=THEME["status_bg"], fg=THEME["accent"], font=("Courier", 10, "bold")).pack()
    tk.Label(stat_frame, textvariable=h_stat, bg=THEME["status_bg"], fg="#f1c40f", font=("Courier", 10, "bold")).pack()

    def update_ui():
        v_stat.set(f"VX: {node._vx:+.2f}  VY: {node._vy:+.2f}  WZ: {node._wz:+.2f}")
        h_stat.set(f"HEADING: {math.degrees(node._current_yaw):.1f}°")
        root.after(100, update_ui)
    update_ui()

    # ── Keyboard Bindings ─────────────────────────────────────────────────────
    def on_key_press(e):
        k = e.keysym
        node.key_press(k, spd(), ang())

    def on_key_release(e):
        k = e.keysym
        node.key_release(k, spd(), ang())

    root.bind("<KeyPress>", on_key_press)
    root.bind("<KeyRelease>", on_key_release)
    root.focus_set()

    return root

def main():
    rclpy.init()
    node = TeleopGUI()
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    
    root = build_gui(node)
    root.mainloop()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
