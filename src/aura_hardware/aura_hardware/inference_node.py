#!/usr/bin/env python3
import sys
from collections import deque
from pathlib import Path
import cv2
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray

ACT_DIR = Path.home() / 'aura_ws' / 'act'
sys.path.insert(0, str(ACT_DIR))

from model import CNNMLPPolicy
from config import (JOINT_ORDER, NUM_JOINTS, CHUNK_SIZE,
                    OBS_HISTORY, IMG_SIZE, EXP_WEIGHT, MAX_DELTA)

CHECKPOINT = ACT_DIR / 'checkpoints' / 'best.pth'

class InferenceNode(Node):
    def __init__(self):
        super().__init__('inference_node')
        self.declare_parameter('checkpoint', str(CHECKPOINT))
        ckpt_path = self.get_parameter('checkpoint').value

        ckpt = torch.load(ckpt_path, map_location='cpu', weights_only=False)
        self._stats      = ckpt['stats']
        self._chunk_size = ckpt['chunk_size']

        self._model = CNNMLPPolicy()
        self._model.load_state_dict(ckpt['model_state'])
        self._model.eval()
        self.get_logger().info(
            f'Loaded checkpoint: {ckpt_path} '
            f'(val_loss={ckpt["val_loss"]:.6f}  epoch={ckpt["epoch"]})')

        self._history: deque[np.ndarray] = deque(
            [np.zeros(NUM_JOINTS, dtype=np.float32)] * (OBS_HISTORY + 1),
            maxlen=OBS_HISTORY + 1,
        )

        blank = np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)
        self._img: np.ndarray = blank

        self._ensemble: list[tuple[np.ndarray, np.ndarray]] = []

        self._pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.create_subscription(JointState, '/joint_states',   self._js_cb,  10)
        self.create_subscription(Image,      'camera/image_raw', self._img_cb, 10)
        self.create_timer(1.0 / 30.0, self._step)
        self.get_logger().info('Inference node ready.')

    def _js_cb(self, msg: JointState) -> None:
        lut  = dict(zip(msg.name, msg.position))
        qpos = np.array([lut.get(n, 0.0) for n in JOINT_ORDER], dtype=np.float32)
        self._history.append(qpos)

    def _img_cb(self, msg: Image) -> None:
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self._img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))

    _IMG_MEAN = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
    _IMG_STD  = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)

    def _step(self) -> None:
        img_rgb = self._img[:, :, ::-1].copy()
        img_t   = torch.from_numpy(img_rgb).permute(2, 0, 1).float() / 255.0
        img_t   = (img_t - self._IMG_MEAN) / self._IMG_STD
        img_t   = img_t.unsqueeze(0)

        obs     = np.stack(self._history, axis=0)
        obs_n   = (obs - self._stats['qpos_mean']) / self._stats['qpos_std']
        obs_t   = torch.from_numpy(obs_n.reshape(1, -1).astype(np.float32))

        with torch.no_grad():
            chunk_n = self._model(img_t, obs_t).squeeze(0).numpy()

        chunk   = chunk_n * self._stats['action_std'] + self._stats['action_mean']
        weights = EXP_WEIGHT ** np.arange(self._chunk_size, dtype=np.float32)
        self._ensemble.append((chunk.copy(), weights.copy()))

        num, den    = np.zeros(NUM_JOINTS, np.float32), 0.0
        next_ens    = []
        for actions, w in self._ensemble:
            num += w[0] * actions[0]
            den += w[0]
            if actions.shape[0] > 1:
                next_ens.append((actions[1:], w[1:]))
        self._ensemble = next_ens

        action = num / (den + 1e-8)

        current_qpos = self._history[-1]
        delta  = np.clip(action - current_qpos, -MAX_DELTA, MAX_DELTA)
        action = current_qpos + delta

        msg      = Float64MultiArray()
        msg.data = action.tolist()
        self._pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
