#!/usr/bin/env python3
"""Convert raw UUV episodes into a USIM-compatible LeRobot v2.1 dataset."""
from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

FPS = 10
CHUNKS_SIZE = 1000
TASK = "Go to the charge station"
VIDEO_KEYS = ("observation.images.ego", "observation.images.wrist")
STATE_DIM = 29
ACTION_DIM = 13
TARGET_DIM = 6

INFO_FEATURES = {
    "observation.images.ego": {
        "dtype": "video",
        "shape": [240, 320, 3],
        "names": ["height", "width", "channel"],
        "info": {
            "video.height": 240,
            "video.width": 320,
            "video.codec": "av1",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": False,
            "video.fps": FPS,
            "video.channels": 3,
            "has_audio": False,
        },
    },
    "observation.images.wrist": {
        "dtype": "video",
        "shape": [240, 320, 3],
        "names": ["height", "width", "channel"],
        "info": {
            "video.height": 240,
            "video.width": 320,
            "video.codec": "av1",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": False,
            "video.fps": FPS,
            "video.channels": 3,
            "has_audio": False,
        },
    },
    "observation.state": {
        "dtype": "float32",
        "shape": [STATE_DIM],
        "names": ["joint_pos", "pwm", "joint_v", "dvl_v", "imu_av", "imu_la", "pressure", "dvl_h"],
    },
    "action": {
        "dtype": "float32",
        "shape": [ACTION_DIM],
        "names": ["joint_pos", "pwm"],
    },
    "target_pos": {"dtype": "float32", "shape": [TARGET_DIM], "names": ["target_pos"]},
    "goal_rel": {"dtype": "float32", "shape": [2], "names": ["goal_dist", "goal_bearing"]},
    "route_rel": {"dtype": "float32", "shape": [3], "names": ["goal_dist", "route_dist", "route_bearing"]},
    "timestamp": {"dtype": "float32", "shape": [1], "names": None},
    "frame_index": {"dtype": "int64", "shape": [1], "names": None},
    "episode_index": {"dtype": "int64", "shape": [1], "names": None},
    "index": {"dtype": "int64", "shape": [1], "names": None},
    "task_index": {"dtype": "int64", "shape": [1], "names": None},
}

MODALITY = {
    "state": {
        "joint_pos": {"start": 0, "end": 5},
        "pwm": {"start": 5, "end": 13},
        "joint_v": {"start": 13, "end": 18},
        "dvl_v": {"start": 18, "end": 21},
        "imu_av": {"start": 21, "end": 24},
        "imu_la": {"start": 24, "end": 27},
        "pressure": {"start": 27, "end": 28},
        "dvl_h": {"start": 28, "end": 29},
    },
    "action": {"joint_pos": {"start": 0, "end": 5}, "pwm": {"start": 5, "end": 13}},
    "target_pos": {"target_pos": {"start": 0, "end": 6}},
    "goal_rel": {"goal_dist": {"start": 0, "end": 1}, "goal_bearing": {"start": 1, "end": 2}},
    "route_rel": {"goal_dist": {"start": 0, "end": 1}, "route_dist": {"start": 1, "end": 2},
                  "route_bearing": {"start": 2, "end": 3}},
    "video": {
        "ego": {"original_key": "observation.images.ego"},
        "wrist": {"original_key": "observation.images.wrist"},
    },
    "annotation": {"human.action.task_description": {"original_key": "task_index"}},
}


def read_result(episode: Path) -> dict:
    result = {}
    for line in (episode / "result.txt").read_text().splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            result[k] = v
    return result


def load_samples(episode: Path) -> np.ndarray:
    import csv

    rows = list(csv.reader((episode / "samples.csv").open()))
    if len(rows) < 2:
        raise RuntimeError("samples.csv is empty")
    header = rows[0]
    data = np.asarray(rows[1:], dtype=np.float64)
    if data.shape[1] != len(header):
        raise RuntimeError(f"malformed samples.csv in {episode}")
    return header, data


def route_relative(episode: Path, header, data, lookahead: float = 5.0) -> np.ndarray:
    """[goal_dist, dist/bearing to a look-ahead point on the planned route].

    The look-ahead point is found by walking ``lookahead`` metres along path.csv from the
    path point closest to the robot. It mirrors the global planner output that would be
    available at deployment and keeps the route bearing small, like the expert's own inputs.
    """
    import csv as _csv

    path = []
    with (episode / "path.csv").open() as f:
        for row in _csv.reader(f):
            try:
                path.append((float(row[0]), float(row[1])))
            except (ValueError, IndexError):
                continue
    if len(path) < 2:
        raise RuntimeError(f"path.csv missing/too short in {episode}")
    path = np.asarray(path, dtype=np.float64)

    yaw = data[:, header.index("yaw")]
    px = data[:, header.index("pos_x")]
    py = data[:, header.index("pos_y")]
    gd = data[:, header.index("goal_dist")]
    out = np.zeros((data.shape[0], 3), dtype=np.float32)
    for k in range(data.shape[0]):
        i = int(np.argmin(np.hypot(path[:, 0] - px[k], path[:, 1] - py[k])))
        acc = 0.0
        while i + 1 < len(path) and acc < lookahead:
            acc += float(np.hypot(path[i + 1, 0] - path[i, 0], path[i + 1, 1] - path[i, 1]))
            i += 1
        dx = path[i, 0] - px[k]
        dy = path[i, 1] - py[k]
        world = math.atan2(dy, dx)
        bearing = math.atan2(math.sin(world - yaw[k]), math.cos(world - yaw[k]))
        out[k] = (gd[k], float(np.hypot(dx, dy)), bearing)
    return out


def goal_relative(header, data) -> np.ndarray:
    """[distance, bearing] of the goal in the robot body frame (bearing + = right)."""
    return np.stack([data[:, header.index("goal_dist")],
                     data[:, header.index("goal_bearing")]], axis=1).astype(np.float32)


def encode_video(images: Path, out: Path, count: int, threads: int = 8) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
        "-framerate", str(FPS), "-start_number", "0",
        "-i", str(images / "%06d.png"),
        "-frames:v", str(count),
        "-c:v", "libaom-av1", "-crf", "28", "-b:v", "0",
        "-cpu-used", "8", "-row-mt", "1", "-threads", str(threads),
        "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(out),
    ]
    subprocess.run(cmd, check=True)


def image_stats(images: Path, count: int, stride: int = 5) -> dict:
    means = np.zeros(3, dtype=np.float64)
    sq = np.zeros(3, dtype=np.float64)
    mins = np.full(3, 255.0)
    maxs = np.zeros(3)
    n = 0
    for idx in range(0, count, stride):
        path = images / f"{idx:06d}.png"
        img = cv2.imread(str(path))
        if img is None:
            raise FileNotFoundError(path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float64) / 255.0
        flat = img.reshape(-1, 3)
        means += flat.mean(axis=0)
        sq += (flat * flat).mean(axis=0)
        mins = np.minimum(mins, flat.min(axis=0))
        maxs = np.maximum(maxs, flat.max(axis=0))
        n += 1
    if n == 0:
        raise RuntimeError(f"no images in {images}")
    mean = means / n
    std = np.sqrt(np.maximum(sq / n - mean * mean, 0.0))
    return {
        "min": [[[float(mins[c])]] for c in range(3)],
        "max": [[[float(maxs[c])]] for c in range(3)],
        "mean": [[[float(mean[c])]] for c in range(3)],
        "std": [[[float(std[c])]] for c in range(3)],
        "count": [count],
    }


def vector_stats(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=np.float64)
    return {
        "min": np.min(values, axis=0).tolist(),
        "max": np.max(values, axis=0).tolist(),
        "mean": np.mean(values, axis=0).tolist(),
        "std": np.std(values, axis=0).tolist(),
        "count": [int(values.shape[0])],
    }


def scalar_stats(values: np.ndarray, dtype=float) -> dict:
    values = np.asarray(values, dtype=np.float64)
    return {
        "min": [dtype(values.min())],
        "max": [dtype(values.max())],
        "mean": [float(values.mean())],
        "std": [float(values.std())],
        "count": [int(values.shape[0])],
    }


def make_parquet(episode: Path, out: Path, episode_index: int, global_index: int) -> tuple[int, np.ndarray]:
    header, data = load_samples(episode)
    n = data.shape[0]
    state = data[:, 1:30].astype(np.float32)
    action = data[:, 30:43].astype(np.float32)
    target = data[:, 43:49].astype(np.float32)
    goal_rel = goal_relative(header, data)
    route_rel = route_relative(episode, header, data)
    timestamp = (np.arange(n, dtype=np.float32) / float(FPS)).astype(np.float32)
    frame_index = np.arange(n, dtype=np.int64)
    ep_index = np.full(n, episode_index, dtype=np.int64)
    index = np.arange(global_index, global_index + n, dtype=np.int64)
    task_index = np.zeros(n, dtype=np.int64)

    schema = pa.schema(
        [
            pa.field("observation.state", pa.list_(pa.float32(), STATE_DIM)),
            pa.field("action", pa.list_(pa.float32(), ACTION_DIM)),
            pa.field("target_pos", pa.list_(pa.float32(), TARGET_DIM)),
            pa.field("goal_rel", pa.list_(pa.float32(), 2)),
            pa.field("route_rel", pa.list_(pa.float32(), 3)),
            pa.field("timestamp", pa.float32()),
            pa.field("frame_index", pa.int64()),
            pa.field("episode_index", pa.int64()),
            pa.field("index", pa.int64()),
            pa.field("task_index", pa.int64()),
        ]
    )
    hf_info = {
        "info": {
            "features": {
                "observation.state": {"feature": {"dtype": "float32", "_type": "Value"}, "length": STATE_DIM, "_type": "List"},
                "action": {"feature": {"dtype": "float32", "_type": "Value"}, "length": ACTION_DIM, "_type": "List"},
                "target_pos": {"feature": {"dtype": "float32", "_type": "Value"}, "length": TARGET_DIM, "_type": "List"},
                "goal_rel": {"feature": {"dtype": "float32", "_type": "Value"}, "length": 2, "_type": "List"},
                "route_rel": {"feature": {"dtype": "float32", "_type": "Value"}, "length": 3, "_type": "List"},
                "timestamp": {"dtype": "float32", "_type": "Value"},
                "frame_index": {"dtype": "int64", "_type": "Value"},
                "episode_index": {"dtype": "int64", "_type": "Value"},
                "index": {"dtype": "int64", "_type": "Value"},
                "task_index": {"dtype": "int64", "_type": "Value"},
            }
        }
    }
    schema = schema.with_metadata({"huggingface": json.dumps(hf_info)})

    table = pa.Table.from_arrays(
        [
            pa.FixedSizeListArray.from_arrays(pa.array(state.reshape(-1), type=pa.float32()), STATE_DIM),
            pa.FixedSizeListArray.from_arrays(pa.array(action.reshape(-1), type=pa.float32()), ACTION_DIM),
            pa.FixedSizeListArray.from_arrays(pa.array(target.reshape(-1), type=pa.float32()), TARGET_DIM),
            pa.FixedSizeListArray.from_arrays(pa.array(goal_rel.reshape(-1), type=pa.float32()), 2),
            pa.FixedSizeListArray.from_arrays(pa.array(route_rel.reshape(-1), type=pa.float32()), 3),
            pa.array(timestamp),
            pa.array(frame_index),
            pa.array(ep_index),
            pa.array(index),
            pa.array(task_index),
        ],
        schema=schema,
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, out, compression="snappy")
    return n, data


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw", required=True, help="directory containing episode_* raw dirs")
    parser.add_argument("--out", required=True, help="output LeRobot dataset root")
    parser.add_argument("--split", default="train")
    parser.add_argument("--include-failed", action="store_true")
    parser.add_argument("--jobs", type=int, default=1, help="parallel episode conversions")
    args = parser.parse_args()
    jobs = args.jobs

    raw_root = Path(args.raw)
    out_root = Path(args.out)
    if out_root.exists():
        shutil.rmtree(out_root)
    (out_root / "meta").mkdir(parents=True)
    (out_root / "data").mkdir(parents=True)
    (out_root / "videos").mkdir(parents=True)

    episodes = []
    candidates = [raw_root] if (raw_root / "samples.csv").exists() else sorted(raw_root.iterdir())
    for d in candidates:
        if not d.is_dir() or not (d / "samples.csv").exists():
            continue
        result = read_result(d) if (d / "result.txt").exists() else {}
        success = result.get("success") == "1"
        if not success and not args.include_failed:
            continue
        episodes.append((d, result))
    if not episodes:
        print("no successful episodes found", file=sys.stderr)
        sys.exit(1)

    total_frames = 0
    global_index = 0
    episode_lines = []
    stats_lines = []
    print(f"converting {len(episodes)} episodes -> {out_root}")

    lengths = [load_samples(ep)[1].shape[0] for ep, _ in episodes]
    offsets = []
    acc = 0
    for n_i in lengths:
        offsets.append(acc)
        acc += n_i
    total_frames = acc

    def work(out_idx: int):
        episode, _ = episodes[out_idx]
        n = lengths[out_idx]
        global_index = offsets[out_idx]
        chunk = out_idx // CHUNKS_SIZE
        data_path = out_root / "data" / f"chunk-{chunk:03d}" / f"episode_{out_idx:06d}.parquet"
        n_read, data = make_parquet(episode, data_path, out_idx, global_index)
        if n_read != n:
            raise RuntimeError("frame count mismatch")

        for key in VIDEO_KEYS:
            short = "ego" if key.endswith("ego") else "wrist"
            video_path = (
                out_root / "videos" / f"chunk-{chunk:03d}" / key / f"episode_{out_idx:06d}.mp4"
            )
            encode_video(episode / "images" / short, video_path, n, threads=video_threads)

        header, data = load_samples(episode)
        state = data[:, 1:30].astype(np.float32)
        action = data[:, 30:43].astype(np.float32)
        target = data[:, 43:49].astype(np.float32)
        goal_rel = goal_relative(header, data)
        route_rel = route_relative(episode, header, data)
        timestamp = (np.arange(n, dtype=np.float32) / float(FPS)).astype(np.float32)
        frame_index = np.arange(n, dtype=np.int64)
        ep_index = np.full(n, out_idx, dtype=np.int64)
        index = np.arange(global_index, global_index + n, dtype=np.int64)
        task_index = np.zeros(n, dtype=np.int64)

        stats = {
            "observation.images.ego": image_stats(episode / "images" / "ego", n),
            "observation.images.wrist": image_stats(episode / "images" / "wrist", n),
            "observation.state": vector_stats(state),
            "action": vector_stats(action),
            "target_pos": vector_stats(target),
            "goal_rel": vector_stats(goal_rel),
            "route_rel": vector_stats(route_rel),
            "timestamp": scalar_stats(timestamp),
            "frame_index": scalar_stats(frame_index, int),
            "episode_index": scalar_stats(ep_index, int),
            "index": scalar_stats(index, int),
            "task_index": scalar_stats(task_index, int),
        }
        print(f"  episode {out_idx:06d}: {n} frames ({episode.name})", flush=True)
        return (json.dumps({"episode_index": out_idx, "stats": stats}) + "\n",
                json.dumps({"episode_index": out_idx, "tasks": [TASK], "length": n}) + "\n")

    video_threads = max(2, 8 // max(1, jobs))
    with ThreadPoolExecutor(max_workers=max(1, jobs)) as pool:
        results = list(pool.map(work, range(len(episodes))))
    stats_lines = [r[0] for r in results]
    episode_lines = [r[1] for r in results]

    total_episodes = len(episodes)
    total_chunks = max(1, math.ceil(total_episodes / CHUNKS_SIZE))
    info = {
        "codebase_version": "v2.1",
        "robot_type": "AgileBot",
        "total_episodes": total_episodes,
        "total_frames": total_frames,
        "total_tasks": 1,
        "total_videos": total_episodes * len(VIDEO_KEYS),
        "total_chunks": total_chunks,
        "chunks_size": CHUNKS_SIZE,
        "fps": FPS,
        "splits": {args.split: f"0:{total_episodes}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": INFO_FEATURES,
    }
    (out_root / "meta" / "info.json").write_text(json.dumps(info, indent=4))
    (out_root / "meta" / "tasks.jsonl").write_text(
        json.dumps({"task_index": 0, "task": TASK}) + "\n"
    )
    (out_root / "meta" / "episodes.jsonl").write_text("".join(episode_lines))
    (out_root / "meta" / "episodes_stats.jsonl").write_text("".join(stats_lines))
    (out_root / "meta" / "modality.json").write_text(json.dumps(MODALITY, indent=4))
    print(f"done: {total_episodes} episodes, {total_frames} frames")


if __name__ == "__main__":
    main()
