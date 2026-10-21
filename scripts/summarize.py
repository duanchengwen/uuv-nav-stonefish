#!/usr/bin/env python3
"""Summarize raw episode quality (success rate, lengths, clearances, seeds)."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def read_kv(path: Path) -> dict:
    out = {}
    if not path.exists():
        return out
    for line in path.read_text().splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            out[k] = v
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", required=True)
    ap.add_argument("--json", default="")
    args = ap.parse_args()
    root = Path(args.raw)

    rows = []
    for d in sorted(root.iterdir()):
        if not d.is_dir() or not (d / "samples.csv").exists():
            continue
        res = read_kv(d / "result.txt")
        meta = read_kv(d / "episode.cfg")
        path_len = 0.0
        p = d / "path.csv"
        if p.exists():
            pts = []
            with p.open() as f:
                for row in csv.DictReader(f):
                    pts.append((float(row["x"]), float(row["y"]), float(row["z"])))
            path_len = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        n = 0
        with (d / "samples.csv").open() as f:
            n = max(0, sum(1 for _ in f) - 1)
        rows.append(
            {
                "episode": d.name,
                "success": res.get("success") == "1",
                "reason": res.get("reason", ""),
                "frames": n,
                "duration_s": round(float(res.get("sim_time", 0.0)), 2),
                "goal_distance": round(float(res.get("goal_distance", 0.0)), 3),
                "min_clearance": round(float(res.get("min_clearance", 0.0)), 3),
                "path_length_m": round(path_len, 2),
                "seed": int(meta.get("seed", -1)),
            }
        )

    if not rows:
        raise SystemExit("no episodes found")
    print(f"{'episode':18s} {'ok':>3s} {'frames':>7s} {'sec':>7s} {'goal':>6s} {'clear':>6s} {'path':>6s} {'seed':>7s}")
    for r in rows:
        print(
            f"{r['episode']:18s} {('Y' if r['success'] else 'N'):>3s} {r['frames']:7d} "
            f"{r['duration_s']:7.2f} {r['goal_distance']:6.3f} {r['min_clearance']:6.3f} "
            f"{r['path_length_m']:6.1f} {r['seed']:7d}"
        )
    ok = sum(1 for r in rows if r["success"])
    frames = sum(r["frames"] for r in rows)
    print(f"\nepisodes={len(rows)} success={ok}/{len(rows)} frames={frames} "
          f"duration={sum(r['duration_s'] for r in rows)/60:.1f} min")
    if args.json:
        Path(args.json).write_text(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
