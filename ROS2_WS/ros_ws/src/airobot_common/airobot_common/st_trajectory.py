"""
    Spatio-temporal trajectory representation from https://github.com/reso1/stgcs
"""
from __future__ import annotations
from typing import List
import numpy as np
from copy import deepcopy


class STTrajectory:

    def __init__(self, vertex_path:List[str], points:List[np.ndarray], dim:int):
        assert len(vertex_path) == len(points), f"|vertex_path| != |points|"
        self.points = points
        self.vertex_path = vertex_path
        self.dim = dim + 1 # include time dimension
    
    def copy(self) -> STTrajectory:
        return STTrajectory(deepcopy(self.vertex_path), deepcopy(self.points), self.dim - 1)
    
    def _space_dim(self) -> int:
        return self.dim - 1

    @property
    def size(self) -> int:
        return len(self.points)
    
    @property
    def x0(self) -> np.ndarray:
        return self.xA(0)

    @property
    def xT(self) -> np.ndarray:
        return self.xB(self.size - 1)
    
    @property
    def duration(self) -> float:
        return self.xT[-1] - self.x0[-1]

    @staticmethod
    def to_dict(self) -> dict:
        return {
        "vertex_path": self.vertex_path,
        "space_dim": self.dim - 1,
        "points": [p.tolist() for p in self.points]
        }

    @staticmethod
    def from_dict(d:dict):
        points = [np.asarray(p, dtype=float) for p in d["points"]]
        return STTrajectory(d["vertex_path"], points, int(d["space_dim"]))

    def lerp(self, t:float) -> np.ndarray:
        if t <= self.x0[-1]:
            return self.points[0][:self.dim]

        if t >= self.xT[-1]:
            return self.points[-1][-self.dim:]

        idx = self.find_segment_index(t)
        xa, xb = self.points[idx][:self.dim], self.points[idx][-self.dim:]
        k = (t - xa[-1]) / (xb[-1] - xa[-1])
        return xa * (1 - k) + xb * k

    def find_segment_index(self, t:float) -> int:
        if t <= self.x0[-1]:
            return 0

        if t >= self.xT[-1]:
            return self.size - 1

        for idx in range(self.size):
            xa, xb = self.points[idx][:self.dim], self.points[idx][-self.dim:]
            if round(t - xa[-1], 6) >= 0 and round(t - xb[-1], 6) <= 0:
                break
        
        assert round(t - xa[-1], 6) >= 0 and round(t - xb[-1], 6) <= 0, f"t: {t} not in [{xa[-1]}, {xb[-1]}]"
        return idx
    
    def get_chunk(self, t0:float=None, tf:float=None) -> STTrajectory:
        t0 = self.x0[-1] if t0 is None else t0
        tf = self.xT[-1] if tf is None else tf
        
        # find segment indices for t0 and tf
        idx_0 = self.find_segment_index(t0)
        idx_f = self.find_segment_index(tf)
        if idx_0 == idx_f:
            vertex_path = [self.vertex_path[idx_0]]
            points = [np.hstack([self.lerp(t0), self.lerp(tf)])]
        elif idx_0 == idx_f - 1:
            vertex_path = [self.vertex_path[idx_0], self.vertex_path[idx_f]]
            points = [np.hstack([self.lerp(t0), self.xB(idx_0)]),
                        np.hstack([self.xA(idx_f), self.lerp(tf)])]
        else:
            vertex_path = [self.vertex_path[idx_0]]
            points = [np.hstack([self.lerp(t0), self.xB(idx_0)])]
            for idx in range(idx_0 + 1, idx_f):
                vertex_path.append(self.vertex_path[idx])
                points.append(self.points[idx])
            vertex_path.append(self.vertex_path[idx_f])
            points.append(np.hstack([self.xA(idx_f), self.lerp(tf)]))

        return STTrajectory(vertex_path, points, self.dim - 1)

    @staticmethod
    def from_chunks(chunks:List[STTrajectory]) -> STTrajectory:
        vertex_path = []
        points = []
        for chunk in chunks:
            assert np.allclose(points[-1], chunk.points[0]) if points else True, "Chunks are not contiguous."
            vertex_path.extend(chunk.vertex_path)
            points.extend(chunk.points)
            
        return STTrajectory(vertex_path, points, chunks[0].dim - 1)

    def xA(self, idx:int) -> np.ndarray:
        return self.points[idx][:self.dim]
    
    def xB(self, idx:int) -> np.ndarray:
        return self.points[idx][self.dim:]

    def transform_space_time(
        self,
        space_scale: float = 1.0,
        time_scale: float = 1.0,
        x_offset: float = -5.0,
        y_offset: float = -5.0,
    ) -> "STTrajectory":
        """
        Scale space and time independently and return a new trajectory.
        - s_space: float or (space_dim,) per-axis scale
        - s_time: scalar time scale
        """
        out = self.copy()
        space_dim = out._space_dim()

        s_space_vec = np.full(space_dim, float(space_scale))
        time_scale = float(time_scale)

        # Anchor
        origin_full = np.zeros(space_dim, dtype=float)
        o_space = origin_full[:space_dim]
        o_time = float(origin_full[-1])
        offset = np.array([x_offset, y_offset], dtype=float)

        def _scale_state(x: np.ndarray) -> np.ndarray:
            x = np.asarray(x, dtype=float).copy()
            x[:space_dim] = o_space + (x[:space_dim] - o_space) * s_space_vec + offset
            x[-1] = o_time + (x[-1] - o_time) * time_scale
            return x

        out.points = [np.hstack([_scale_state(p[:out.dim]), _scale_state(p[out.dim:])]) for p in out.points]
        return out