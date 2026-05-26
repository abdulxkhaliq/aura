#!/usr/bin/env python3
import shutil
import sys
import threading
import termios
import tty
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray

from lerobot.datasets.lerobot_dataset import LeRobotDataset

JOINT_ORDER = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
               'wrist_flex', 'wrist_roll', 'gripper']
NUM_JOINTS = len(JOINT_ORDER)

def _decode_image(msg: Image) -> np.ndarray:
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == 'bgr8':
        arr = arr[:, :, ::-1]
    return np.ascontiguousarray(arr[:, :, :3])

class SmolVLARecorderNode(Node):
    def __init__(self):
        super().__init__('smolvla_recorder')

        self.declare_parameter('repo_id',       'local/aura_pickup')
        self.declare_parameter('root',          str(Path.home() / 'lerobot_demos'))
        self.declare_parameter('fps',           30)
        self.declare_parameter('task',          'Pick up the red cube')
        self.declare_parameter('img_h',         480)
        self.declare_parameter('img_w',         640)
        self.declare_parameter('action_source', 'commands')

        self._repo = self.get_parameter('repo_id').value
        self._root = Path(self.get_parameter('root').value).expanduser()
        fps        = int(self.get_parameter('fps').value)
        self._task = self.get_parameter('task').value
        self._h    = int(self.get_parameter('img_h').value)
        self._w    = int(self.get_parameter('img_w').value)
        self._action_source = self.get_parameter('action_source').value
        if self._action_source not in ('commands', 'states'):
            raise ValueError(
                f"action_source must be 'commands' or 'states', got {self._action_source!r}")

        self._qpos:   np.ndarray | None = None
        self._action: np.ndarray | None = None
        self._front:  np.ndarray | None = None
        self._wrist:  np.ndarray | None = None
        self._lock = threading.Lock()
        self._recording = False
        self._frames = 0

        joints_meta = {'dtype': 'float32', 'shape': (NUM_JOINTS,), 'names': JOINT_ORDER}
        cam_meta    = {'dtype': 'video',   'shape': (self._h, self._w, 3),
                       'names': ['height', 'width', 'channels']}
        features = {
            'observation.state':         joints_meta,
            'action':                    joints_meta,
            'observation.images.front':  cam_meta,
            'observation.images.wrist':  cam_meta,
        }

        ds_root = self._root / self._repo
        info_json = ds_root / 'meta' / 'info.json'
        tasks_parquet = ds_root / 'meta' / 'tasks.parquet'
        data_dir = ds_root / 'data'
        has_episodes = data_dir.exists() and any(data_dir.rglob('*.parquet'))

        if info_json.exists() and tasks_parquet.exists() and has_episodes:
            self.get_logger().info(f'Resuming finalized dataset at {ds_root}')
            self._dataset = LeRobotDataset.resume(
                self._repo, root=str(ds_root), image_writer_threads=4)
        elif info_json.exists() and not has_episodes:
            self.get_logger().warning(
                f'Found empty stillborn dataset at {ds_root} (info.json but no episodes). '
                f'Removing and recreating.')
            shutil.rmtree(ds_root)
            self._dataset = LeRobotDataset.create(
                self._repo, fps, root=str(ds_root),
                features=features, robot_type='so_arm101',
                use_videos=True, image_writer_threads=4)
        elif info_json.exists() and has_episodes and not tasks_parquet.exists():
            raise RuntimeError(
                f'{ds_root} has episodes but was never finalized (no meta/tasks.parquet). '
                f'A previous session crashed before pressing Q. Either:\n'
                f'  (a) keep the episodes by manually finalizing — re-record one trivial '
                f'episode, then press Q (this will write tasks.parquet); or\n'
                f'  (b) discard the data: rm -rf "{ds_root}"')
        elif ds_root.exists():
            raise RuntimeError(
                f'{ds_root} exists but has no meta/info.json. Looks like a corrupt '
                f'leftover. Remove it (rm -rf "{ds_root}") or pick a different repo_id.')
        else:
            self.get_logger().info(f'Creating new dataset at {ds_root}')
            self._dataset = LeRobotDataset.create(
                self._repo, fps, root=str(ds_root),
                features=features, robot_type='so_arm101',
                use_videos=True, image_writer_threads=4)

        self.create_subscription(JointState,        '/joint_states',     self._js_cb,    10)
        self.create_subscription(Float64MultiArray, '/joint_commands',   self._cmd_cb,   10)
        self.create_subscription(Image,             '/front/image_raw',  self._front_cb, 10)
        self.create_subscription(Image,             '/wrist/image_raw',  self._wrist_cb, 10)
        self.create_timer(1.0 / fps, self._step)

        threading.Thread(target=self._kb_loop, daemon=True).start()

        n = self._dataset.num_episodes if hasattr(self._dataset, 'num_episodes') else '?'
        self.get_logger().info(
            f'Recorder ready ({fps} Hz, action_source={self._action_source}). '
            f'Existing episodes: {n}.\n'
            f'Task: {self._task!r}\n'
            f'  ENTER  start/stop episode\n'
            f'  D      discard current\n'
            f'  Q      finalize and quit')

    def _js_cb(self, msg):
        lut = dict(zip(msg.name, msg.position))
        with self._lock:
            self._qpos = np.array([lut.get(n, 0.0) for n in JOINT_ORDER], dtype=np.float32)

    def _cmd_cb(self, msg):
        if len(msg.data) >= NUM_JOINTS:
            with self._lock:
                self._action = np.array(msg.data[:NUM_JOINTS], dtype=np.float32)

    def _front_cb(self, msg):
        with self._lock:
            self._front = _decode_image(msg)

    def _wrist_cb(self, msg):
        with self._lock:
            self._wrist = _decode_image(msg)

    def _step(self):
        if not self._recording:
            return
        with self._lock:
            qpos   = None if self._qpos   is None else self._qpos.copy()
            front  = None if self._front  is None else self._front.copy()
            wrist  = None if self._wrist  is None else self._wrist.copy()
            cmd    = None if self._action is None else self._action.copy()

        action = cmd if self._action_source == 'commands' else qpos

        if any(x is None for x in (qpos, action, front, wrist)):
            self.get_logger().warning(
                'waiting for all inputs before recording starts',
                throttle_duration_sec=2.0)
            return
        if front.shape[0] != self._h or front.shape[1] != self._w:
            self.get_logger().error(
                f'front image is {front.shape[:2]} but dataset expects '
                f'({self._h},{self._w}). Pass img_h:= img_w:= matching the camera.',
                throttle_duration_sec=5.0)
            return
        if wrist.shape[0] != self._h or wrist.shape[1] != self._w:
            self.get_logger().error(
                f'wrist image is {wrist.shape[:2]} but dataset expects '
                f'({self._h},{self._w})', throttle_duration_sec=5.0)
            return

        frame = {
            'observation.state':         qpos,
            'action':                    action,
            'observation.images.front':  front,
            'observation.images.wrist':  wrist,
            'task':                      self._task,
        }
        try:
            self._dataset.add_frame(frame)
            self._frames += 1
        except Exception as e:
            self.get_logger().error(f'add_frame failed: {e}')

    def _get_key(self) -> str:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _kb_loop(self):
        while rclpy.ok():
            try:
                ch = self._get_key()
            except Exception:
                break
            if ch in ('\r', '\n'):
                if self._recording:
                    self._stop(save=True)
                else:
                    self._start()
            elif ch.lower() == 'd':
                if self._recording:
                    self._stop(save=False)
                else:
                    print('\n[discard: not currently recording]', flush=True)
            elif ch.lower() == 'q':
                if self._recording:
                    self._stop(save=False)
                self.get_logger().info('Finalizing dataset ...')
                try:
                    self._dataset.finalize()
                    self.get_logger().info('Dataset finalized.')
                except Exception as e:
                    self.get_logger().error(f'finalize: {e}')
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
                return

    def _start(self):
        self._frames = 0
        self._recording = True
        self.get_logger().info('>>> RECORDING — press ENTER to stop, D to discard.')

    def _stop(self, save: bool):
        self._recording = False
        n = self._frames
        if save and n > 0:
            try:
                self._dataset.save_episode()
                self.get_logger().info(f'Saved episode ({n} frames).')
            except Exception as e:
                self.get_logger().error(f'save_episode failed: {e}')
        else:
            try:
                self._dataset.writer.clear_episode_buffer()
            except Exception:
                pass
            self.get_logger().info(f'Discarded episode ({n} frames).')

def main(args=None):
    rclpy.init(args=args)
    node = SmolVLARecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node._recording:
                node._stop(save=False)
        except Exception:
            pass
        try:
            node.get_logger().info('Finalizing dataset (shutdown)...')
            node._dataset.finalize()
            node.get_logger().info('Dataset finalized.')
        except Exception as e:
            try:
                node.get_logger().warning(f'finalize on shutdown: {e}')
            except Exception:
                pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
