# Generated Stonefish scenarios

These are the exact scenarios used for the 13-episode sample dataset.
All scenes use **cruise depth 2.2 m** (0.8 m above the seabed) and are generated so the
charging target board is visible in the first camera frame: 3/3 board samples in some
episodes, 2/3 (partial rock occlusion) in others.

```
scenes/
├── train/episode_000000 .. episode_000009   # 10 training scenes
└── test/episode_000000  .. episode_000002   # 3 test scenes
```

Each directory contains:

| File | Purpose |
|---|---|
| `scene.scn` | Complete Stonefish scenario: lake, rocks, containers, pipeline, charging dock, BlueROV2 include |
| `path.csv` | Dense A* path (`x,y,z,yaw`, ~0.25 m spacing) |
| `obstacles.csv` | Circular obstacle approximation used by A* and collision checks |
| `episode.cfg` | Task, seed, start/goal pose, depth, success radius/yaw, target visibility, timeout |
| `preview.png` | Top-down view of obstacles, path, start, goal, dock |

## Run one scene

```bash
cd /path/to/uuv_charge_nav

# One command: copies the scene to episodes_raw/run_<split>_<episode> and launches it
./scripts/run_scene.sh scenes/train/episode_000000

# Another terminal, for ROS 2 / RViz:
./ros2/run_camera_view.sh "$PWD/episodes_raw/run_train_episode_000000"
```

Or manually:

```bash
export LD_LIBRARY_PATH=/path/to/stonefish/install_stonefish/lib:$LD_LIBRARY_PATH
mkdir -p episodes_raw/demo_run
cp scenes/train/episode_000000/* episodes_raw/demo_run/
./build/uuv_nav \
  --scene "$PWD/episodes_raw/demo_run/scene.scn" \
  --episode-dir "$PWD/episodes_raw/demo_run"
```

The app writes `images/ego`, `images/wrist`, `samples.csv` and `result.txt` into
`episodes_raw/demo_run`. The Stonefish data directory must contain the assets in
`data/` and the robot include `data/scenes/robots/bluerov2_usim_nav.scn`; the
build compiles the data path, so build the app from this repository after cloning.

For parallel collection of new scenes, `scripts/collect.py` supports `--jobs 3`
(each job opens its own Stonefish window on the same GPU):

```bash
python scripts/collect.py --split train --count 10 --jobs 3
```

To also stream both cameras to ROS 2 / RViz:

```bash
./ros2/run_camera_view.sh "$PWD/episodes_raw/demo_run"
```

## Relationship to the dataset

`datasets/usim/train` and `datasets/usim/test` are the LeRobot v2.1 conversions of
these scenes. The raw `episodes_raw/` PNG sequences are intentionally excluded
from Git (about 1.2 GB); they can be regenerated with `scripts/collect.py` or by
running the scenes above.
