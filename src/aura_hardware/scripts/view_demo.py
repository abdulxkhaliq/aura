#!/usr/bin/env python3
import sys
from pathlib import Path
import h5py
import numpy as np
import cv2
import matplotlib.pyplot as plt

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
          'wrist_flex',   'wrist_roll',    'gripper']

def list_all(d: Path):
    eps = sorted(d.glob('episode_*.hdf5'))
    print(f'\n{len(eps)} episodes in {d}\n')
    for ep in eps:
        with h5py.File(ep) as f:
            T = f['observations/qpos'].shape[0]
        print(f'  {ep.name}  —  {T} frames  ({T/30:.1f}s)')
    print()

def view(path: Path):
    with h5py.File(path) as f:
        qpos = f['observations/qpos'][:]
        imgs = f['observations/images/wrist_cam'][:]
    T = len(qpos)
    print(f'{path.name}: {T} frames ({T/30:.1f}s)')

    fig, axes = plt.subplots(2, 3, figsize=(12, 5), sharex=True)
    fig.suptitle(path.name)
    t = np.arange(T) / 30.0
    for ax, name, col in zip(axes.flat, JOINTS, range(6)):
        ax.plot(t, np.degrees(qpos[:, col]))
        ax.set_title(name); ax.set_ylabel('deg'); ax.grid(alpha=0.3)
    plt.tight_layout()
    plt.show(block=False)

    print('Video: SPACE=pause/resume  ←/→=step  q=quit')
    paused, i = False, 0
    while i < T:
        frame = imgs[i].copy()
        cv2.putText(frame, f'{i}/{T-1}  gripper:{np.degrees(qpos[i,5]):.0f}deg',
                    (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,0), 1)
        cv2.imshow(path.name, frame)
        key = cv2.waitKey(1 if paused else 33) & 0xFF
        if   key == ord('q'):            break
        elif key == ord(' '):            paused = not paused
        elif key == 81 and i > 0:        i -= 1
        elif key == 83:                  i += 1
        elif not paused:                 i += 1
    cv2.destroyAllWindows()
    plt.close('all')

p = Path(sys.argv[1]).expanduser()
if p.is_dir():
    list_all(p)
else:
    view(p)
