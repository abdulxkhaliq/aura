#!/usr/bin/env python3
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64
import h5py
import numpy as np
import threading
from pathlib import Path

JOINT_ORDER = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
               'wrist_flex', 'wrist_roll', 'gripper']
NUM_JOINTS   = len(JOINT_ORDER)
GRIPPER_STEP = 0.05

class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')
        self.declare_parameter('output_dir',        str(Path.home() / 'aura_demos'))
        self.declare_parameter('max_frames',        3000)
        self.declare_parameter('gripper_open_rad',   1.74533)
        self.declare_parameter('gripper_close_rad', -0.174533)

        self.output_dir       = Path(self.get_parameter('output_dir').value)
        self.max_frames       = self.get_parameter('max_frames').value
        self.gripper_open_rad  = self.get_parameter('gripper_open_rad').value
        self.gripper_close_rad = self.get_parameter('gripper_close_rad').value
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.current_joints = np.zeros(NUM_JOINTS, dtype=np.float32)
        self.current_image  = None
        self.recording      = False
        self.gripper_rad    = self.gripper_open_rad
        self.qpos_buf: list = []
        self.img_buf:  list = []

        existing = list(self.output_dir.glob('episode_*.hdf5'))
        self.episode_idx = len(existing)

        self.create_subscription(JointState, 'joint_states',     self.joint_cb, 10)
        self.create_subscription(Image,      'camera/image_raw', self.image_cb, 10)

        self.gripper_pub = self.create_publisher(Float64, 'gripper_pos_cmd', 10)

        threading.Thread(target=self.keyboard_loop, daemon=True).start()

        self.get_logger().info(f'Output dir : {self.output_dir}')
        self.get_logger().info(f'Next episode: {self.episode_idx:04d}')
        self._print_help()

    def joint_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        for i, name in enumerate(JOINT_ORDER):
            if name in name_to_pos:
                self.current_joints[i] = name_to_pos[name]

    def image_cb(self, msg: Image):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        self.current_image = frame.copy()

        if self.recording:
            self.qpos_buf.append(self.current_joints.copy())
            self.img_buf.append(self.current_image.copy())

            n = len(self.qpos_buf)
            if n % 30 == 0:
                print(f'  ... {n} frames', flush=True)

            if n >= self.max_frames:
                self.recording = False
                print(f'Max frames ({self.max_frames}) reached, auto-saving.')
                self.save_episode()

    def _get_key(self) -> str:
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def keyboard_loop(self):
        while rclpy.ok():
            ch = self._get_key()

            if ch in ('\x03', '\x04', 'q', 'Q'):
                print('\nQuitting.', flush=True)
                break

            elif ch in ('g', 'G'):
                self._step_gripper(close=True)

            elif ch in ('f', 'F'):
                self._step_gripper(close=False)

            elif ch in ('\r', '\n'):
                if not self.recording:
                    self.qpos_buf.clear()
                    self.img_buf.clear()
                    self.recording = True
                    print(f'\nRecording episode {self.episode_idx:04d}  '
                          f'[gripper: {self._gripper_pct():.0f}% closed]  '
                          f'— ENTER to save  g=close  f=open  d=discard', flush=True)
                else:
                    self.recording = False
                    self.save_episode()

            elif ch in ('d', 'D'):
                if self.recording:
                    self.recording = False
                    self.qpos_buf.clear()
                    self.img_buf.clear()
                    print('Discarded.', flush=True)
                    self._print_help()
                else:
                    print('(not recording — nothing to discard)', flush=True)

    def _step_gripper(self, close: bool):
        lo = min(self.gripper_open_rad, self.gripper_close_rad)
        hi = max(self.gripper_open_rad, self.gripper_close_rad)
        step = GRIPPER_STEP if self.gripper_close_rad > self.gripper_open_rad else -GRIPPER_STEP
        if not close:
            step = -step
        self.gripper_rad = float(np.clip(self.gripper_rad + step, lo, hi))
        msg      = Float64()
        msg.data = float(self.gripper_rad)
        self.gripper_pub.publish(msg)
        print(f'\rGripper {self._gripper_pct():.0f}% closed '
              f'({self.gripper_rad:.2f} rad)  ',
              end='', flush=True)

    def _gripper_pct(self) -> float:
        span = abs(self.gripper_close_rad - self.gripper_open_rad)
        return abs(self.gripper_rad - self.gripper_open_rad) / span * 100.0

    def save_episode(self):
        T = len(self.qpos_buf)
        if T < 10:
            print(f'Too short ({T} frames), discarded.', flush=True)
            self.qpos_buf.clear()
            self.img_buf.clear()
            self._print_help()
            return

        qpos = np.array(self.qpos_buf, dtype=np.float32)
        imgs = np.array(self.img_buf,  dtype=np.uint8)

        path = self.output_dir / f'episode_{self.episode_idx:04d}.hdf5'
        with h5py.File(path, 'w') as f:
            obs = f.create_group('observations')
            obs.create_dataset('qpos', data=qpos, compression='gzip')
            obs.create_dataset('qvel', data=np.zeros_like(qpos), compression='gzip')
            imgs_grp = obs.create_group('images')
            imgs_grp.create_dataset(
                'wrist_cam', data=imgs, compression='gzip',
                chunks=(1, *imgs.shape[1:]))
            f.create_dataset('action', data=qpos, compression='gzip')

            f.attrs['episode_idx'] = self.episode_idx
            f.attrs['num_frames']  = T
            f.attrs['joint_order'] = JOINT_ORDER

        sz_mb = path.stat().st_size / 1e6
        print(f'Saved  episode_{self.episode_idx:04d}.hdf5  '
              f'({T} frames, {sz_mb:.1f} MB)', flush=True)
        self.episode_idx += 1
        self.qpos_buf.clear()
        self.img_buf.clear()
        self._print_help()

    def _print_help(self):
        total = list(self.output_dir.glob('episode_*.hdf5'))
        print(
            f'[{len(total)} episodes saved | gripper: {self._gripper_pct():.0f}% closed]  '
            f'ENTER=record/save  g=close  f=open  d=discard  q=quit\n',
            flush=True,
        )

def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
