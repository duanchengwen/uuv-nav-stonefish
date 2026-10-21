#!/usr/bin/env python3
"""Generate and run UUV charging-station navigation episodes."""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
from generate_episode import build_episode  # noqa: E402


def read_result(episode: Path) -> dict:
    path = episode / "result.txt"
    if not path.exists():
        return {}
    out = {}
    for line in path.read_text().splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            out[k] = v
    return out


def main() -> None:
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    ap = argparse.ArgumentParser()
    ap.add_argument("--split", default="train", choices=["train", "test", "debug"])
    ap.add_argument("--count", type=int, default=5)
    ap.add_argument("--start-index", type=int, default=0)
    ap.add_argument("--raw-root", default="")
    ap.add_argument("--base-seed", type=int, default=1000)
    ap.add_argument("--attempts", type=int, default=3)
    ap.add_argument("--timeout", type=float, default=240.0)
    ap.add_argument("--binary", default=str(ROOT / "build" / "uuv_nav"))
    ap.add_argument("--display", default=os.environ.get("DISPLAY", ":1"))
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--scene-only", action="store_true")
    ap.add_argument("--success-radius", type=float, default=0.80)
    ap.add_argument("--success-yaw", type=float, default=0.40)
    args = ap.parse_args()

    raw_root = Path(args.raw_root).resolve() if args.raw_root else ROOT / "episodes_raw" / args.split
    raw_root.mkdir(parents=True, exist_ok=True)
    binary = Path(args.binary)
    if not args.scene_only and not binary.exists():
        raise SystemExit(f"binary not found: {binary}; build with cmake/make first")

    env = os.environ.copy()
    env["DISPLAY"] = args.display
    env["LD_LIBRARY_PATH"] = str(ROOT.parent / "install_stonefish" / "lib") + ":" + env.get("LD_LIBRARY_PATH", "")

    successes = 0
    failures = 0
    for offset in range(args.count):
        idx = args.start_index + offset
        episode = raw_root / f"episode_{idx:06d}"
        if args.resume and read_result(episode).get("success") == "1":
            print(f"[collect] skip existing {episode.name}")
            successes += 1
            continue

        last_error = ""
        for attempt in range(args.attempts):
            seed = args.base_seed + idx * 100 + attempt
            if episode.exists():
                shutil.rmtree(episode)
            t0 = time.time()
            info = build_episode(episode, seed=seed, split=args.split, episode_index=idx,
                                 success_radius=args.success_radius, success_yaw=args.success_yaw)
            print(f"[collect] {episode.name} seed={seed} gen={time.time()-t0:.1f}s "
                  f"path={info['path_length']:.1f}m turn={info['turning']:.2f} "
                  f"clear={info['clearance']:.2f} rocks={info['rocks']}")
            if args.scene_only:
                successes += 1
                last_error = ""
                break

            cmd = [str(binary), "--scene", str(episode / "scene.scn"), "--episode-dir", str(episode)]
            log_path = episode / "run.log"
            try:
                t0 = time.time()
                with log_path.open("w") as log_file:
                    proc = subprocess.run(cmd, env=env, stdout=log_file, stderr=subprocess.STDOUT,
                                          timeout=args.timeout, text=True)
            except subprocess.TimeoutExpired:
                last_error = "process timeout"
                print(f"[collect] timeout ({args.timeout}s), retrying")
                continue
            result = read_result(episode)
            elapsed = time.time() - t0
            if not result:
                tail = "\n".join(log_path.read_text().splitlines()[-15:]) if log_path.exists() else ""
                print(f"[collect] no result (rc={proc.returncode}, {elapsed:.1f}s). Last output:\n{tail}")
            if result.get("success") == "1":
                successes += 1
                last_error = ""
                print(f"[collect] OK {episode.name} frames={result.get('frames')} "
                      f"t={result.get('sim_time')}s goal={result.get('goal_distance')} "
                      f"wall={elapsed:.1f}s")
                break
            last_error = result.get("reason", "no result")
            print(f"[collect] FAIL {episode.name}: {last_error} (attempt {attempt+1}/{args.attempts})")
            if attempt + 1 < args.attempts:
                failed = raw_root / f"failed_{idx:06d}_{attempt}"
                if failed.exists():
                    shutil.rmtree(failed)
                shutil.move(str(episode), str(failed))
        else:
            failures += 1
            print(f"[collect] giving up on episode {idx}: {last_error}")
            if episode.exists():
                failed = raw_root / f"failed_{idx:06d}"
                if failed.exists():
                    shutil.rmtree(failed)
                shutil.move(str(episode), str(failed))

    print(f"[collect] done: {successes} successes, {failures} failures -> {raw_root}")


if __name__ == "__main__":
    main()
