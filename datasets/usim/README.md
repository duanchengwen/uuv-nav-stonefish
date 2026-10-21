# UUV charge-station navigation dataset (USIM-compatible)

Synthetic **Underwater Vehicle** navigation dataset generated in Stonefish v1.5 for
vision-language-action (VLA) training, in **LeRobot v2.1** format and matching the
USIM `goto_charge_station` schema.

## Task

- Instruction: **"Go to the charge station"**
- Robot: BlueROV2, 8 thrusters, no manipulator (USIM navigation configuration)
- Scene: 50 m x 30 m shallow lake with a serpentine rock field, containers, a pipeline,
  and a solar-powered charging dock; the UUV must avoid obstacles and dock.
- Expert: A* global path + pure-pursuit tracking + 90 deg multibeam local avoidance.

## Splits

| Split | Episodes | Frames | Average episode |
|---|---:|---:|---:|
| train | 10 | 8338 | 84.6 s |
| test | 3 | 2498 | 84.5 s |

All episodes are successful expert demonstrations (collection failures are discarded
before conversion). Each episode is 10 Hz.

## Observations / actions

| Key | Shape | Description |
|---|---|---|
| `observation.images.ego` | 240 x 320 x 3 (AV1, 10 fps) | front-right stereo camera |
| `observation.images.wrist` | 240 x 320 x 3 (AV1, 10 fps) | front-left / hand camera (body-mounted in the nav configuration) |
| `observation.state` | 29 | joint_pos(5), pwm(8), joint_v(5), dvl_v(3), imu_av(3), imu_la(3), pressure(1), dvl_h(1) |
| `action` | 13 | joint_pos(5), normalized thruster PWM(8) |
| `target_pos` | 6 | next waypoint in body frame (x,y,z,roll,pitch,yaw) |

The arm dimensions are zero (no manipulator). Pressure is stored as
`fluid_pressure / 10000` (approximately depth in metres), exactly as in USIM.
Videos use AV1 / yuv420p / 320x240 / 10 fps, with one parquet episode file and two
video files per episode.

## LeRobot usage

The dataset is consumed directly by the pinned LeRobot v2.1 loader used in
u0-openpi (`uv sync` installs it); no conversion to RLDS/v3 is required.

## π0.5 training

Use the USIM π0.5 baseline config `pi05_u0bot` from
<https://github.com/VincentGu2000/u0-openpi> (or the patch in `../../openpi_ext/`).
Set `DATA_BASE_DIR` to the directory containing `usim/train` and `usim/test`.

The sample is intended to validate the data/training plumbing. For a useful policy,
collect hundreds to thousands of episodes with `../../scripts/collect.py` and additional
domain randomization.

## License / attribution

Scene assets are derived from USIM/u0env (Apache-2.0); Stonefish is GPLv3. See the
project `../../NOTICE.md` and the USIM paper (arXiv:2510.07869).
