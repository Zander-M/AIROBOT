#!/usr/bin/env python3
from __future__ import annotations

import argparse
import pickle
from pathlib import Path

from airobot_common import STTrajectory

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="in_path", required=True, help="input traj.pkl")
    ap.add_argument("--out", dest="out_path", required=True, help="output traj_transformed.pkl")
    ap.add_argument("--space_scale", type=float, default=5.0)
    ap.add_argument("--time_scale", type=float, default=10.0)
    ap.add_argument("--x_offset", type=float, default=-8.0)
    ap.add_argument("--y_offset", type=float, default=-8.0)
    args = ap.parse_args()

    in_path = Path(args.in_path)
    out_path = Path(args.out_path)

    with in_path.open("rb") as f:
        obj = pickle.load(f)

    out = []
    for i, traj_d in enumerate(obj):
        tr = STTrajectory.from_dict(traj_d)
        tr2 = tr.transform_space_time(args.space_scale, args.time_scale, args.x_offset, args.y_offset)
        out.append(tr2.to_dict())

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("wb") as f:
        pickle.dump(out, f)

    print(f"Wrote {len(out)} transformed trajectories to {out_path}")


if __name__ == "__main__":
    main()
