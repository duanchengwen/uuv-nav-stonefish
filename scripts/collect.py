#!/usr/bin/env python3
"""Generate and run UUV charging-station navigation episodes.

Episodes can be collected concurrently (``--jobs N``): each worker owns one episode
directory and one Stonefish window, so the scenes run in parallel on the same GPU.
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import generate_episode  # noqa: E402
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


def start_fraction(seed: int, max_frac: float) -> float:
    return ((seed * 1664525 + 1013904223) % 1000000) / 1000000.0 * max_frac


def collect_one(idx: int, args, raw_root: Path, binary: Path, env: dict) -> tuple[int, bool, str]:
    episode = raw_root / f"episode_{idx:06d}"
    tag = f"[{episode.name}]"
    if args.resume and read_result(episode).get("success") == "1":
        return idx, True, f"{tag} skipped (already complete)"

    last_error = ""
    for attempt in range(args.attempts):
        seed = args.base_seed + idx * 100 + attempt
        if episode.exists():
            shutil.rmtree(episode)
        t0 = time.time()
        info = build_episode(episode, seed=seed, split=args.split, episode_index=idx,
                             success_radius=args.success_radius, success_yaw=args.success_yaw,
                             start_frac=start_fraction(seed, args.start_frac_max))
        print(f"{tag} seed={seed} gen={time.time()-t0:.1f}s path={info['path_length']:.1f}m "
              f"turn={info['turning']:.2f} clear={info['clearance']:.2f} "
              f"rocks={info['rocks']} target_vis={info.get('target_visible', '?')}",
              flush=True)
        if args.scene_only:
            return idx, True, f"{tag} scene only"

        cmd = [str(binary), "--scene", str(episode / "scene.scn"), "--episode-dir", str(episode)]
        if args.expert == "direct":
            cmd.append("--direct-expert")
        log_path = episode / "run.log"
        try:
            t0 = time.time()
            with log_path.open("w") as log_file:
                proc = subprocess.run(cmd, env=env, stdout=log_file, stderr=subprocess.STDOUT,
                                      timeout=args.timeout, text=True)
        except subprocess.TimeoutExpired:
            last_error = "process timeout"
            print(f"{tag} timeout ({args.timeout}s), retrying", flush=True)
            continue

        result = read_result(episode)
        elapsed = time.time() - t0
        if not result:
            tail = "\n".join(log_path.read_text().splitlines()[-15:]) if log_path.exists() else ""
            print(f"{tag} no result (rc={proc.returncode}, {elapsed:.1f}s). Last output:\n{tail}",
                  flush=True)
        if result.get("success") == "1":
            return idx, True, (f"{tag} OK frames={result.get('frames')} t={result.get('sim_time')}s "
                               f"goal={result.get('goal_distance')} wall={elapsed:.1f}s")
        last_error = result.get("reason", "no result")
        print(f"{tag} FAIL {last_error} (attempt {attempt+1}/{args.attempts})", flush=True)
        if attempt + 1 < args.attempts:
            failed = raw_root / f"failed_{idx:06d}_{attempt}"
            if failed.exists():
                shutil.rmtree(failed)
            shutil.move(str(episode), str(failed))

    if episode.exists():
        failed = raw_root / f"failed_{idx:06d}"
        if failed.exists():
            shutil.rmtree(failed)
        shutil.move(str(episode), str(failed))
    return idx, False, f"{tag} giving up: {last_error}"


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
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--binary", default=str(ROOT / "build" / "uuv_nav"))
    ap.add_argument("--display", default=os.environ.get("DISPLAY", ":1"))
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--scene-only", action="store_true")
    ap.add_argument("--success-radius", type=float, default=0.30)
    ap.add_argument("--success-yaw", type=float, default=0.10)
    ap.add_argument("--start-frac-max", type=float, default=1.0,
                    help="randomize the start distance; 1.0 uses the full 8-15 m range")
    ap.add_argument("--jobs", type=int, default=1, help="number of concurrent simulator instances")
    ap.add_argument("--depth", type=float, default=None,
                    help="cruise depth in metres (NED z); generator default is 2.2")
    ap.add_argument("--expert", choices=["path", "direct"], default="path",
                    help="path: A* pure-pursuit expert; direct: goal-bearing + sonar detour expert")
    args = ap.parse_args()

    if args.depth is not None:
        generate_episode.DEPTH = args.depth
    raw_root = Path(args.raw_root).resolve() if args.raw_root else ROOT / "episodes_raw" / args.split
    raw_root.mkdir(parents=True, exist_ok=True)
    binary = Path(args.binary)
    if not args.scene_only and not binary.exists():
        raise SystemExit(f"binary not found: {binary}; build with cmake/make first")

    env = os.environ.copy()
    env["DISPLAY"] = args.display
    env["LD_LIBRARY_PATH"] = str(ROOT.parent / "install_stonefish" / "lib") + ":" + env.get("LD_LIBRARY_PATH", "")

    idxs = list(range(args.start_index, args.start_index + args.count))
    jobs = max(1, args.jobs)
    successes = 0
    failures = 0
    print(f"[collect] {len(idxs)} episodes, jobs={jobs}, display={args.display}", flush=True)
    with ThreadPoolExecutor(max_workers=jobs) as pool:
        futures = {pool.submit(collect_one, idx, args, raw_root, binary, env): idx for idx in idxs}
        for fut in as_completed(futures):
            idx, ok, msg = fut.result()
            print(msg, flush=True)
            successes += int(ok)
            failures += int(not ok)

    print(f"[collect] done: {successes} successes, {failures} failures -> {raw_root}", flush=True)
    raise SystemExit(0 if failures == 0 else 1)


if __name__ == "__main__":
    main()
