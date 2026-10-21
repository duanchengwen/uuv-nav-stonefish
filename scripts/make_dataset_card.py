#!/usr/bin/env python3
"""Write a dataset card for the converted USIM-compatible UUV navigation dataset."""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def read_kv(path: Path) -> dict:
    out = {}
    if path.exists():
        for line in path.read_text().splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                out[k] = v
    return out


def episode_stats(raw: Path) -> tuple[int, int, float, int, float]:
    ok = 0
    total = 0
    frames = 0
    duration = 0.0
    for d in sorted(raw.iterdir()):
        if not d.is_dir() or not (d / "result.txt").exists():
            continue
        total += 1
        res = read_kv(d / "result.txt")
        if res.get("success") == "1":
            ok += 1
            frames += int(res.get("frames", 0))
            duration += float(res.get("sim_time", 0.0))
    return ok, total, duration, frames, (duration / ok if ok else 0.0)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--datasets", default="datasets/usim")
    ap.add_argument("--raw-train", default="episodes_raw/train")
    ap.add_argument("--raw-test", default="episodes_raw/test")
    args = ap.parse_args()

    root = Path(args.datasets)
    train_info = json.loads((root / "train/meta/info.json").read_text())
    test_info = json.loads((root / "test/meta/info.json").read_text())
    tr_ok, tr_total, tr_dur, tr_frames, tr_avg = episode_stats(Path(args.raw_train))
    te_ok, te_total, te_dur, te_frames, te_avg = episode_stats(Path(args.raw_test))

    text = f"""# UUV charge-station navigation dataset (USIM-compatible)

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
| train | {tr_ok} | {tr_frames} | {tr_avg:.1f} s |
| test | {te_ok} | {te_frames} | {te_avg:.1f} s |

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
"""
    out = root / "README.md"
    out.write_text(text)
    print(f"wrote {out}")
    print(f"train: {tr_ok}/{tr_total} episodes, {tr_frames} frames; test: {te_ok}/{te_total}, {te_frames} frames")
    print(f"info train frames={train_info['total_frames']} test frames={test_info['total_frames']}")


if __name__ == "__main__":
    main()
