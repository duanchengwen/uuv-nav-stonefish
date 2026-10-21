#!/usr/bin/env python3
"""Structural validation for a USIM-compatible LeRobot v2.1 dataset."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pyarrow.parquet as pq

EXPECTED_KEYS = [
    "observation.images.ego",
    "observation.images.wrist",
    "observation.state",
    "action",
    "target_pos",
    "timestamp",
    "frame_index",
    "episode_index",
    "index",
    "task_index",
]


def probe_video(path: Path) -> dict:
    out = subprocess.check_output(
        [
            "ffprobe", "-v", "error", "-select_streams", "v:0",
            "-show_entries", "stream=codec_name,width,height,pix_fmt,r_frame_rate,nb_frames",
            "-of", "json", str(path),
        ]
    )
    return json.loads(out)["streams"][0]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--max-episodes", type=int, default=0)
    args = ap.parse_args()
    root = Path(args.dataset)

    info = json.loads((root / "meta/info.json").read_text())
    errors = []
    if info.get("codebase_version") != "v2.1":
        errors.append(f"codebase_version={info.get('codebase_version')} (expected v2.1)")
    if info.get("fps") != 10:
        errors.append(f"fps={info.get('fps')}")
    keys = set(info["features"].keys())
    if keys != set(EXPECTED_KEYS):
        errors.append(f"feature keys mismatch: {sorted(keys)}")

    state_shape = info["features"]["observation.state"]["shape"]
    action_shape = info["features"]["action"]["shape"]
    target_shape = info["features"]["target_pos"]["shape"]
    if state_shape != [29]:
        errors.append(f"state shape {state_shape}")
    if action_shape != [13]:
        errors.append(f"action shape {action_shape}")
    if target_shape != [6]:
        errors.append(f"target_pos shape {target_shape}")
    for key in ("observation.images.ego", "observation.images.wrist"):
        feat = info["features"][key]
        if feat["shape"] != [240, 320, 3]:
            errors.append(f"{key} shape {feat['shape']}")
        if feat["info"].get("video.codec") != "av1":
            errors.append(f"{key} codec {feat['info'].get('video.codec')}")

    episodes = [json.loads(line) for line in (root / "meta/episodes.jsonl").read_text().splitlines() if line]
    if len(episodes) != info["total_episodes"]:
        errors.append("episode count mismatch")
    tasks = [json.loads(line) for line in (root / "meta/tasks.jsonl").read_text().splitlines() if line]
    if not tasks or tasks[0]["task"] != "Go to the charge station":
        errors.append("task description mismatch")

    total = 0
    checked = episodes[: args.max_episodes] if args.max_episodes > 0 else episodes
    for ep in checked:
        idx = ep["episode_index"]
        chunk = idx // info["chunks_size"]
        parquet = root / "data" / f"chunk-{chunk:03d}" / f"episode_{idx:06d}.parquet"
        if not parquet.exists():
            errors.append(f"missing parquet {parquet}")
            continue
        table = pq.read_table(parquet)
        cols = set(table.column_names)
        if cols != set(EXPECTED_KEYS) - {"observation.images.ego", "observation.images.wrist"}:
            errors.append(f"{parquet.name}: columns {sorted(cols)}")
            continue
        n = table.num_rows
        if n != ep["length"]:
            errors.append(f"{parquet.name}: rows {n} != length {ep['length']}")
        state = np.asarray(table["observation.state"].to_pylist(), dtype=np.float32)
        action = np.asarray(table["action"].to_pylist(), dtype=np.float32)
        target = np.asarray(table["target_pos"].to_pylist(), dtype=np.float32)
        if state.shape != (n, 29) or action.shape != (n, 13) or target.shape != (n, 6):
            errors.append(f"{parquet.name}: array shapes {state.shape} {action.shape} {target.shape}")
        if not (np.isfinite(state).all() and np.isfinite(action).all() and np.isfinite(target).all()):
            errors.append(f"{parquet.name}: non-finite values")
        if not np.allclose(action[:, 5:], state[:, 5:13], atol=1e-5):
            errors.append(f"{parquet.name}: action pwm != state pwm")
        if n > 0 and not np.all(np.diff(np.asarray(table["frame_index"])) == 1):
            errors.append(f"{parquet.name}: frame_index not contiguous")
        ts = np.asarray(table["timestamp"], dtype=np.float64)
        if n > 1 and not np.allclose(np.diff(ts), 1.0 / info["fps"], atol=1e-4):
            errors.append(f"{parquet.name}: timestamps not at {info['fps']} Hz")

        for key in ("observation.images.ego", "observation.images.wrist"):
            video = root / "videos" / f"chunk-{chunk:03d}" / key / f"episode_{idx:06d}.mp4"
            if not video.exists():
                errors.append(f"missing video {video}")
                continue
            meta = probe_video(video)
            if meta.get("codec_name") != "av1":
                errors.append(f"{video.name}: codec {meta.get('codec_name')}")
            if int(meta.get("width", 0)) != 320 or int(meta.get("height", 0)) != 240:
                errors.append(f"{video.name}: size {meta.get('width')}x{meta.get('height')}")
            if meta.get("pix_fmt") != "yuv420p":
                errors.append(f"{video.name}: pix_fmt {meta.get('pix_fmt')}")
            if meta.get("r_frame_rate") not in ("10/1", "10"):
                errors.append(f"{video.name}: fps {meta.get('r_frame_rate')}")
            if "nb_frames" in meta and int(meta["nb_frames"]) != n:
                errors.append(f"{video.name}: frames {meta['nb_frames']} != {n}")
        total += n
        print(f"  episode {idx:06d}: {n} frames ok")

    if len(checked) == len(episodes) and total != info["total_frames"]:
        errors.append(f"total frames {total} != {info['total_frames']}")

    stats_lines = (root / "meta/episodes_stats.jsonl").read_text().splitlines()
    if len(stats_lines) != info["total_episodes"]:
        errors.append("episodes_stats count mismatch")
    for line in stats_lines[:1]:
        rec = json.loads(line)
        for key in ("observation.state", "action", "target_pos", "observation.images.ego"):
            if key not in rec["stats"]:
                errors.append(f"stats missing {key}")

    print(f"\ndataset: {root}")
    print(f"episodes: {info['total_episodes']}  frames: {info['total_frames']}  fps: {info['fps']}")
    print(f"state {state_shape} action {action_shape} target {target_shape}")
    if errors:
        print("\nFAILED:")
        for e in errors:
            print("  -", e)
        sys.exit(1)
    print("\nOK: dataset is structurally compatible with USIM / LeRobot v2.1 / openpi pi05_u0bot")


if __name__ == "__main__":
    main()
