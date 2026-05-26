#!/usr/bin/env python3
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from pynput import keyboard

JOINT_ORDER = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
               'wrist_flex', 'wrist_roll', 'gripper']
NUM_JOINTS = len(JOINT_ORDER)

KEY_MAP: dict[str, tuple[int, float]] = {
    'a': (0, -1.0), 'd': (0, +1.0),
    's': (1, -1.0), 'w': (1, +1.0),
    'e': (2, -1.0), 'q': (2, +1.0),
    'f': (3, -1.0), 'r': (3, +1.0),
    'z': (4, -1.0), 'x': (4, +1.0),
    'g': (5, -1.0), 'h': (5, +1.0),
}

JOINT_LIMITS_LO = np.array([-2.0, -2.5, -2.5, -2.5, -3.0, -0.2], dtype=np.float64)
JOINT_LIMITS_HI = np.array([+2.0, +2.5, +2.5, +2.5, +3.0, +2.5], dtype=np.float64)

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('fast_mult', 2.0)
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('mode', 'full')

        self._speed = float(self.get_parameter('speed').value)
        self._fastm = float(self.get_parameter('fast_mult').value)
        rate = float(self.get_parameter('rate').value)
        self._dt = 1.0 / rate
        self._mode = self.get_parameter('mode').value
        if self._mode not in ('full', 'gripper_only'):
            raise ValueError(f"mode must be 'full' or 'gripper_only', got {self._mode!r}")

        self._target: np.ndarray | None = None
        self._held: set[str] = set()
        self._fast = False
        self._lock = threading.Lock()

        self._arm_pub: rclpy.publisher.Publisher | None = None
        self._grp_pub: rclpy.publisher.Publisher | None = None
        if self._mode == 'full':
            self._arm_pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        else:
            self._grp_pub = self.create_publisher(Float64, '/gripper_pos_cmd', 10)

        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.create_timer(self._dt, self._step)

        self._listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.start()

        if self._mode == 'full':
            self.get_logger().info(
                'Keyboard teleop ready (full).\n'
                '  shoulder_pan  : A / D\n'
                '  shoulder_lift : S / W\n'
                '  elbow_flex    : E / Q\n'
                '  wrist_flex    : F / R\n'
                '  wrist_roll    : Z / X\n'
                '  gripper       : G (close) / H (open)\n'
                '  Shift = fast (×%.1f)\n'
                '  Ctrl-C to quit.' % self._fastm)
        else:
            self.get_logger().info(
                'Keyboard teleop ready (gripper-only, arm backdrives freely).\n'
                '  gripper : G (close) / H (open)\n'
                '  Shift = fast (×%.1f)\n'
                '  Ctrl-C to quit.' % self._fastm)

    def _js_cb(self, msg: JointState) -> None:
        if self._target is not None:
            return
        lut = dict(zip(msg.name, msg.position))
        qpos = np.array([lut.get(n, 0.0) for n in JOINT_ORDER], dtype=np.float64)
        with self._lock:
            self._target = qpos.copy()
        self.get_logger().info(f'Seeded target from /joint_states: {np.round(qpos, 3).tolist()}')

    def _key_char(self, key) -> str | None:
        try:
            return key.char.lower() if getattr(key, 'char', None) else None
        except Exception:
            return None

    def _on_press(self, key) -> None:
        ch = self._key_char(key)
        valid = (ch in ('g', 'h')) if self._mode == 'gripper_only' else (ch in KEY_MAP)
        if ch and valid:
            with self._lock:
                self._held.add(ch)
        if key in (keyboard.Key.shift, keyboard.Key.shift_r):
            self._fast = True

    def _on_release(self, key) -> None:
        ch = self._key_char(key)
        if ch:
            with self._lock:
                self._held.discard(ch)
        if key in (keyboard.Key.shift, keyboard.Key.shift_r):
            self._fast = False

    def _step(self) -> None:
        if self._target is None:
            return
        with self._lock:
            held = set(self._held)
            fast = self._fast

        speed = self._speed * (self._fastm if fast else 1.0)

        if self._mode == 'gripper_only':
            delta = 0.0
            if 'g' in held: delta -= speed * self._dt
            if 'h' in held: delta += speed * self._dt
            self._target[5] = np.clip(self._target[5] + delta,
                                      JOINT_LIMITS_LO[5], JOINT_LIMITS_HI[5])
            msg = Float64()
            msg.data = float(self._target[5])
            self._grp_pub.publish(msg)
            return

        if held:
            delta = np.zeros(NUM_JOINTS, dtype=np.float64)
            for ch in held:
                idx, sign = KEY_MAP[ch]
                delta[idx] += sign * speed * self._dt
            self._target = np.clip(self._target + delta, JOINT_LIMITS_LO, JOINT_LIMITS_HI)

        msg = Float64MultiArray()
        msg.data = self._target.tolist()
        self._arm_pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self._listener.stop()
        except Exception:
            pass
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
