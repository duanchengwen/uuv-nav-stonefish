# Third-party notices

This project reuses simulation assets and code from the following open-source
projects. Keep these notices when redistributing the scene or derived data.

## Stonefish

- https://github.com/patrykcieslak/stonefish
- License: GNU GPL v3
- Used for the physics/rendering engine and the BlueROV2/thruster runtime.

## u0env (USIM simulation environment)

- https://github.com/VincentGu2000/u0env
- License: Apache-2.0
- Reused assets: BlueROV2 meshes/textures, `charge0/charge1` charging pads,
  solar-panel meshes/textures, `stone.obj`, `grass.obj`, container/pipe meshes,
  and the USIM robot/sensor layout reference.
- The `data/bluerov2`, `data/charge`, `data/solar_panel`, `data/stone`,
  `data/grassstone`, `data/container`, `data/pipe`, `data/flat.obj` and
  `data/SoilBeach0078_1_350.jpg` directories are copied from that repository.

## USIM dataset and U0

- https://huggingface.co/datasets/Vincent2025hello/usim
- https://arxiv.org/abs/2510.07869
- License: Apache-2.0 (dataset)
- The LeRobot v2.1 schema, 29-dim state layout, 13-dim action layout,
  dual-camera naming and 10 Hz/320x240 settings follow the released USIM
  `goto_charge_station` episodes.

## u0-openpi

- https://github.com/VincentGu2000/u0-openpi
- License: Apache-2.0
- `openpi_ext/u0bot_policy.py` and the data-config pattern in
  `openpi_ext/config_snippet.py` are adapted from this repository.

## OpenPI

- https://github.com/Physical-Intelligence/openpi
- License: Apache-2.0
- The π0.5 training stack used by u0-openpi.
