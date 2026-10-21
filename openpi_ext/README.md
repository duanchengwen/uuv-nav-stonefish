# OpenPI π0.5 integration

The generated dataset follows the **USIM LeRobot v2.1 schema exactly**, so it can
be consumed by the USIM π0.5 baseline without any data conversion:

- repository: <https://github.com/VincentGu2000/u0-openpi>
- train config: `pi05_u0bot`
- policy: `src/openpi/policies/u0bot_policy.py`
- data config: `LeRobotU0BotDataConfig`

The baseline expects the dataset at:

```
$DATA_BASE_DIR/usim/train    # training split
$DATA_BASE_DIR/usim/test     # test split
```

`scripts/collect_and_convert.sh` writes exactly that layout. The dataset keys,
shapes, fps, action dimension and state dimension are all checked by
`scripts/validate_lerobot.py`.

## Option A — use u0-openpi (recommended, no code changes)

```bash
git clone https://github.com/VincentGu2000/u0-openpi.git
cd u0-openpi
# follow README section 1 (conda env + uv sync)
cp .env.example .env
# edit .env so that DATA_BASE_DIR points to the parent of the "usim" folder
# e.g. DATA_BASE_DIR=/media/ai/DATA1/dcw/stonefish/uuv_charge_nav/datasets
source .env

# one-time norm stats
python scripts/compute_norm_stats.py \
  --config_name pi05_u0bot \
  --repo_id $DATA_BASE_DIR/usim/train

# fine-tune π0.5
python scripts/train.py pi05_u0bot \
  --exp-name=uuv_charge_nav_pi05 \
  --overwrite
```

The config trains with absolute actions (the dataset stores normalized PWM
commands directly), action horizon 16, batch 64, and a frozen PaliGemma LLM.

> The included sample dataset is small (10 train / 3 test episodes) and is meant
> to validate the scene + data + training plumbing. For a real policy, collect
> hundreds to thousands of episodes with `scripts/collect.py`; otherwise 22k
> steps will simply overfit the sample. For a quick smoke test, reduce
> `num_train_steps` and `save_interval` in the config.

## Option B — patch the upstream openpi repository

1. Copy `u0bot_policy.py` into `openpi/src/openpi/policies/`.
2. Append the contents of `config_snippet.py` to
   `openpi/src/openpi/training/config.py` (before `_CONFIGS`).
3. Add `TRAIN_CONFIG_SNIPPET` to the `_CONFIGS` list.
4. Follow the official openpi installation/training instructions with
   `pi05_uuv_charge_nav` as the config name.

## Dataset contract

| Key | Shape | Notes |
|---|---|---|
| `observation.images.ego` | 240×320×3 video | USIM ego (right stereo camera) |
| `observation.images.wrist` | 240×320×3 video | USIM wrist (hand/left stereo camera; body-mounted for navigation) |
| `observation.state` | 29 | joint_pos(5), pwm(8), joint_v(5), dvl_v(3), imu_av(3), imu_la(3), pressure(1), dvl_h(1) |
| `action` | 13 | joint_pos(5), pwm(8) |
| `target_pos` | 6 | next waypoint in body frame (x,y,z,roll,pitch,yaw) |
| task | text | `"Go to the charge station"` |

The arm-related dimensions (joint_pos/joint_v) are zero because this scene uses
the USIM navigation configuration (BlueROV2 without manipulator), exactly like
the `goto_charge_station` episodes in USIM.

## Attribution

`u0bot_policy.py` and the data-config pattern are adapted from
`VincentGu2000/u0-openpi` (Apache-2.0), which is a fork of Physical
Intelligence's openpi (Apache-2.0).
