"""
Preprocess Trajectories. Each robot now has a (x, y, yaw, t) trajectory with
a separate trajectory file.
"""
from __future__ import annotations

import argparse
import json
import numpy as np
from pathlib import Path
import pickle

from airobot_common import STTrajectory 

# ----- Utils -----

def wrap_to_pi(a: float) -> float:
    return (a + np.pi) % (2.0 * np.pi) - np.pi

def load_st_trajectory(pkl_path: str)-> list[STTrajectory]:
    """
    Docstring for load_st_trajectory
    
    :param pkl_path: Description
    :type pkl_path: str
    :return: Description
    :rtype: list[STTrajectory]
    """

    p = Path(pkl_path)
    assert p.exists(), "Path does not exist."

    with p.open("rb") as f:
        trajs = pickle.load(f)
    st_trajs = [STTrajectory.from_dict(traj) for traj in trajs]
    return st_trajs

def current_xy_yaw(traj: STTrajectory, t: float, dt: float) -> np.ndarray:
    """
    Return current x y yaw at timestep t by looking forward at the trajectory
    
    :param traj: STTrajectory for current agent
    :param t: Description
    :param dt: Description
    """
    curr_pos = np.asarray(traj.lerp(t)[:-1], dtype=float)
    future_pos = np.asarray(traj.lerp(t+dt)[:-1], dtype=float)

    dxy = future_pos - curr_pos
    if np.linalg.norm(dxy) > 1e-6:
        curr_yaw = np.arctan2(dxy[1], dxy[0]) 
        curr_yaw = wrap_to_pi(curr_yaw)
    else:
        curr_yaw = 0.0
    return np.hstack([curr_pos, curr_yaw, t])

def construct_per_robot_waypoints(args):
    """
    Docstring for construct_per_robot_waypoints
    
    :param args: Description
    """
    pkl_path = args.pkl_path
    output_path = args.output_path
    dt = args.dt

    trajs = load_st_trajectory(pkl_path)
    num_robots = len(trajs)

    
    duration =  np.max(np.array([traj.xT for traj in trajs])) + 1.0 # duration depends on the longest trajectory + 1sec buffer

    output_path = Path(output_path)
    output_path.mkdir(parents=True, exist_ok=True)

    ts = np.arange(0, duration, dt)

    # Create numpy array as per-robot waypoint
    for i in range (num_robots):
        robot_ns = f"robot{i}"
        waypoints = np.stack([current_xy_yaw(trajs[i], t, dt) for t in ts])

        out_file = output_path / f"{robot_ns}.npy"
        np.save(out_file, waypoints)
    
    # Saving metadata
    d_json = {
        "source_pkl": str(pkl_path),
        "frame": "world",
        "num_trajectory": len(trajs),
        "num_waypoints": len(ts),
        "dt": dt,
        "duration": duration,
    }

    meta_file = output_path / "metadata.json"
    with meta_file.open("w") as f:
        json.dump(d_json, f, indent=4)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pkl_path", type=str, help="input path for list of STTrajectory")
    parser.add_argument("--output_path", type=str, help="output path for per-robot trajectory data")
    parser.add_argument("--dt", type=float, default=0.05, help="time interval for each st waypoint")

    args = parser.parse_args()
    construct_per_robot_waypoints(args)
    print(f"Saved waypoints at {args.output_path}.")


if __name__ == "__main__":
    main()