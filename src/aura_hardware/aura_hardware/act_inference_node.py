#!/usr/bin/env python3
import json
import numpy as np
import torch
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray

from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.act.processor_act import make_act_pre_post_processors
from huggingface_hub import hf_hub_download
from safetensors.torch import load_file

JOINT_ORDER = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
               'wrist_flex', 'wrist_roll', 'gripper']
NUM_JOINTS = len(JOINT_ORDER)
IMG_PREFIX = 'observation.images.'

HEURISTIC = {
    'wrist':   'wrist', 'on_robot': 'wrist', 'gripper_cam': 'wrist', 'low': 'wrist',
    'front':   'front', 'side':     'front', 'phone':       'front',
    'up':      'front', 'top':      'front', 'high':        'front',
    'external':'front',
    'camera1': 'front', 'camera3':  'wrist', 'camera2':     'zero',
}

class ACTInferenceNode(Node):
    def __init__(self):
        super().__init__('act_inference_node')

        self.declare_parameter('checkpoint', '')
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('max_delta', 0.02)
        self.declare_parameter('image_map_overrides', '{}')
        self.declare_parameter('img_size', 256)
        self.declare_parameter('joint_units', 'auto')

        ckpt = self.get_parameter('checkpoint').value
        if not ckpt:
            raise ValueError(
                "Required parameter 'checkpoint' is empty. Pass the path to a "
                "pretrained_model dir, e.g. outputs/aura_act/checkpoints/last/pretrained_model")
        self._dev   = torch.device(self.get_parameter('device').value)
        rate        = float(self.get_parameter('publish_rate').value)
        self._mxd   = float(self.get_parameter('max_delta').value)
        self._img_hw = int(self.get_parameter('img_size').value)
        units_param = self.get_parameter('joint_units').value
        overrides_str = self.get_parameter('image_map_overrides').value
        try:
            overrides = json.loads(overrides_str) if overrides_str else {}
        except json.JSONDecodeError as e:
            raise ValueError(f'image_map_overrides is not valid JSON: {e}')

        self.get_logger().info(f'Loading ACT policy from {ckpt} on {self._dev} ...')
        self._policy = ACTPolicy.from_pretrained(ckpt).to(self._dev)
        self._policy.eval()
        self._policy.reset()

        self.get_logger().info('Building pre/post processors (normalizer) ...')
        legacy_state_mean: torch.Tensor | None = None
        try:
            self._pre, self._post = make_pre_post_processors(
                self._policy.config, pretrained_path=ckpt)
        except FileNotFoundError as e:
            if 'policy_preprocessor.json' not in str(e):
                raise
            self.get_logger().warning(
                f'Checkpoint {ckpt} is in legacy format (no policy_preprocessor.json). '
                f'Reconstructing preprocessor from in-model normalize_*.buffer_* stats.')
            sf = ckpt + '/model.safetensors'
            try:
                sd = load_file(sf)
            except FileNotFoundError:
                sf = hf_hub_download(ckpt, 'model.safetensors')
                sd = load_file(sf)
            stats = {
                'observation.state': {
                    'mean': sd['normalize_inputs.buffer_observation_state.mean'],
                    'std':  sd['normalize_inputs.buffer_observation_state.std'],
                },
                'action': {
                    'mean': sd['unnormalize_outputs.buffer_action.mean'],
                    'std':  sd['unnormalize_outputs.buffer_action.std'],
                },
            }
            legacy_state_mean = stats['observation.state']['mean']
            self._pre, self._post = make_act_pre_post_processors(
                self._policy.config, dataset_stats=stats)

        if units_param == 'auto':
            mag = float(legacy_state_mean.abs().max()) if legacy_state_mean is not None else 0.0
            self._units = 'deg' if mag > 6.5 else 'rad'
            if legacy_state_mean is not None:
                self.get_logger().info(
                    f'Auto-detected joint units = {self._units} '
                    f'(state mean max |·| = {mag:.2f}).')
        else:
            if units_param not in ('rad', 'deg'):
                raise ValueError(f"joint_units must be 'auto', 'rad', or 'deg' — got {units_param!r}")
            self._units = units_param
            self.get_logger().info(f'Joint units forced to {self._units}.')

        all_inputs = list(self._policy.config.input_features.keys())
        self._image_keys = [k for k in all_inputs if k.startswith(IMG_PREFIX)]
        if not self._image_keys:
            raise RuntimeError(f'Checkpoint {ckpt} has no observation.images.* inputs.')

        self._image_src: dict[str, str] = {}
        for full in self._image_keys:
            short = full[len(IMG_PREFIX):].lower()
            if short in overrides:
                self._image_src[full] = overrides[short]
            elif short in HEURISTIC:
                self._image_src[full] = HEURISTIC[short]
            else:
                self._image_src[full] = 'front'

        self.get_logger().info('Loaded. Image routing for this checkpoint:')
        for k, src in self._image_src.items():
            self.get_logger().info(f'  {k}  ←  {src}')

        self._qpos:  np.ndarray | None = None
        self._front: np.ndarray | None = None
        self._wrist: np.ndarray | None = None

        self._pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.create_subscription(JointState, '/joint_states',     self._js_cb,    10)
        self.create_subscription(Image,      '/front/image_raw',  self._front_cb, 10)
        self.create_subscription(Image,      '/wrist/image_raw',  self._wrist_cb, 10)
        self.create_timer(1.0 / rate, self._step)
        self.get_logger().info(f'ACT inference node ready, control rate {rate:.1f} Hz.')

    def _js_cb(self, msg: JointState) -> None:
        lut = dict(zip(msg.name, msg.position))
        self._qpos = np.array([lut.get(n, 0.0) for n in JOINT_ORDER], dtype=np.float32)

    def _img_to_chw01(self, msg: Image) -> np.ndarray:
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'bgr8':
            frame = frame[:, :, ::-1]
        frame = cv2.resize(frame[:, :, :3], (self._img_hw, self._img_hw),
                           interpolation=cv2.INTER_AREA)
        return frame.astype(np.float32).transpose(2, 0, 1) / 255.0

    def _front_cb(self, msg: Image) -> None:
        self._front = self._img_to_chw01(msg)

    def _wrist_cb(self, msg: Image) -> None:
        self._wrist = self._img_to_chw01(msg)

    def _step(self) -> None:
        needs_front = any(s == 'front' for s in self._image_src.values())
        needs_wrist = any(s == 'wrist' for s in self._image_src.values())
        missing = []
        if self._qpos is None:                  missing.append('/joint_states')
        if needs_front and self._front is None: missing.append('/front/image_raw')
        if needs_wrist and self._wrist is None: missing.append('/wrist/image_raw')
        if missing:
            self.get_logger().warning(
                f'waiting for: {", ".join(missing)}', throttle_duration_sec=2.0)
            return

        qpos_for_model = self._qpos * (180.0 / np.pi) if self._units == 'deg' else self._qpos
        state_t = torch.from_numpy(qpos_for_model.astype(np.float32))
        front_t = torch.from_numpy(self._front) if self._front is not None else None
        wrist_t = torch.from_numpy(self._wrist) if self._wrist is not None else None
        zero_t  = torch.zeros(3, self._img_hw, self._img_hw, dtype=torch.float32)
        src_to_tensor = {'front': front_t, 'wrist': wrist_t, 'zero': zero_t}

        batch: dict[str, object] = {'observation.state': state_t}
        for full, src in self._image_src.items():
            t = src_to_tensor[src]
            batch[full] = t if t is not None else zero_t

        with torch.inference_mode():
            batch  = self._pre(batch)
            action = self._policy.select_action(batch)
            action = self._post(action)

        action = action.squeeze(0).detach().cpu().numpy()

        if action.shape[0] != NUM_JOINTS:
            self.get_logger().warning(
                f'ACT returned action of shape {action.shape}, expected ({NUM_JOINTS},)',
                throttle_duration_sec=5.0)
            action = action[:NUM_JOINTS]

        if self._units == 'deg':
            action = action * (np.pi / 180.0)

        delta = np.clip(action - self._qpos, -self._mxd, self._mxd)
        cmd = (self._qpos + delta).astype(float)

        out = Float64MultiArray()
        out.data = cmd.tolist()
        self._pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = ACTInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
