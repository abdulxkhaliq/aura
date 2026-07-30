# AURA

**Autonomous Unified Robotic Arm.** A 6-DOF arm that learns pick-and-place tasks from human demonstrations instead of being manually programmed.

Built on the SO-ARM101 with STS3215 servos. Integrates ROS 2, lerobot, and imitation learning (ACT / SmolVLA) into one workspace.

---

## Demos & dataset

[![AURA wrist view](https://img.youtube.com/vi/-fd4ih16TK0/maxresdefault.jpg)](https://www.youtube.com/watch?v=-fd4ih16TK0)

- **Recorded dataset** (~70 episodes, parquet + MP4) — [aki1107/aura-pickup on HuggingFace](https://huggingface.co/datasets/aki1107/aura-pickup)
- **Trained ACT checkpoint** — [aki1107/aura-act on HuggingFace](https://huggingface.co/aki1107/aura-act)
- **Trained SmolVLA checkpoint** — [aki1107/aura-smolvla on HuggingFace](https://huggingface.co/aki1107/aura-smolvla)

---

## What it does

- Records human demonstrations through keyboard teleop or by physically moving the arm
- Trains an imitation-learning policy (ACT or SmolVLA) on those demos
- Runs the trained policy in a closed loop — cameras + joint states → policy → motor commands
- Visualizes the arm in RViz, with optional slider teleop into the real hardware

## Hardware

- **SO-ARM101** (6-DOF arm with parallel gripper, STS3215 servos)
- **USB serial bus** to the servos (typically `/dev/ttyACM0`)
- **Two USB cameras** — one front view of the workspace, one mounted on the wrist
- **Linux PC** with an NVIDIA GPU (8 GB VRAM is enough for ACT; SmolVLA wants more)

## Quick start

```bash
git clone https://github.com/<your-username>/aura.git ~/aura_ws
cd ~/aura_ws

# Install Python dependencies
pip install --user --break-system-packages 'lerobot[smolvla]' 'setuptools<80'

# Build the workspace
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Confirm hardware
v4l2-ctl --list-devices       # note the camera /dev/videoN values
ls /dev/ttyACM*               # should show /dev/ttyACM0
```

## Workflow

The full pipeline is **record → train → run**. Pick a model (ACT for small datasets, SmolVLA for larger / language-conditioned).

### 1. Record demos

Backdrive mode — push the arm by hand, control the gripper from the keyboard:

```bash
# Terminal 1
ros2 launch aura_hardware collect_backdrive.launch.py \
    front_device:=0 wrist_device:=4

# Terminal 2 (this terminal needs keyboard focus for ENTER/D/Q)
ros2 run aura_hardware smolvla_recorder_node --ros-args \
    -p task:='Pick up the cube' \
    -p repo_id:='local/aura_pickup' \
    -p action_source:='states'
```

Press **ENTER** to start a demo, **ENTER** to save it, **D** to discard, **Q** to finalize and quit. Gripper keys while recording: **G** close, **H** open. Aim for 30–100 demos.

### 2. Train

Pick one. Both produce a checkpoint at `outputs/<name>/checkpoints/last/pretrained_model/`.

**ACT** — small model, 3–5 hours on a consumer GPU, best for 30–100 demos:

```bash
lerobot-train \
  --policy.type=act \
  --policy.push_to_hub=false \
  --policy.device=cuda \
  --dataset.repo_id=local/aura_pickup \
  --dataset.root=~/lerobot_demos/local/aura_pickup \
  --batch_size=8 --steps=100000 --save_freq=10000 \
  --output_dir=outputs/aura_act
```

**SmolVLA** — large vision-language-action model, fine-tunes a pretrained base, overnight job:

```bash
lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --policy.push_to_hub=false \
  --policy.empty_cameras=1 \
  --dataset.repo_id=local/aura_pickup \
  --dataset.root=~/lerobot_demos/local/aura_pickup \
  --rename_map='{observation.images.front: observation.images.camera1, observation.images.wrist: observation.images.camera3}' \
  --batch_size=4 --steps=20000 --save_freq=2500 \
  --output_dir=outputs/aura_smolvla
```

### 3. Run inference

```bash
# ACT
ros2 launch aura_hardware act_inference.launch.py \
    checkpoint:=outputs/aura_act/checkpoints/last/pretrained_model \
    front_device:=0 wrist_device:=4 max_delta:=0.1

# SmolVLA
ros2 launch aura_hardware smolvla_inference.launch.py \
    checkpoint:=outputs/aura_smolvla/checkpoints/last/pretrained_model \
    task:='Pick up the cube' \
    front_device:=0 wrist_device:=4 max_delta:=0.1
```

`max_delta` caps how far each joint moves per control step (default 0.02 rad ≈ 0.6 rad/s peak). Raise to 0.1 for more decisive motion.

## Visualize the arm

```bash
# Standalone — drag joint sliders, see the URDF move
ros2 launch aura_description display_rviz.launch.py run_rviz:=true

# Mirror the real arm (run alongside any launch that starts servo_driver)
ros2 launch aura_description display_rviz.launch.py run_rviz:=true standalone:=false
```

## Project structure

```
aura_ws/
└── src/
    ├── aura_description/      URDF, meshes, RViz config
    ├── aura_hardware/         servo driver, cameras, recorders, inference nodes, teleop
    ├── aura_moveit_config/    MoveIt 2 config (motion planning)
    └── servo_examples/        standalone scservo_sdk scripts (no ROS)
```

The `aura_hardware` package contains every runtime node:

| Node | Role |
|---|---|
| `servo_driver` | Talks to the STS3215 servo bus. Publishes `/joint_states`, accepts `joint_commands`. |
| `camera_node` | Bridges one v4l2 camera to a ROS Image topic. Used twice (front + wrist). |
| `smolvla_recorder_node` | Writes a `LeRobotDataset` (parquet + MP4) compatible with `lerobot-train`. |
| `smolvla_inference_node` | Runs a SmolVLA policy in a 30 Hz closed loop. |
| `act_inference_node` | Same, for ACT policies. |
| `keyboard_teleop_node` | pynput-based teleop. Full mode (all joints) or gripper-only mode. |
| `twin_teleop_node` | Bridges RViz joint sliders to the real arm. |



