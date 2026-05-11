import math
from pathlib import Path
from typing import Dict

import numpy as np
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator


DEFAULT_SOURCE_CSV = (
    "/home/w1nchie/ros2_ws/src/seabed_slam/data/missouri_i64/"
    "site-23_MissouriRiver_I-64_2020-08_CUBE-uncert.csv"
)
DEFAULT_META_PATH = (
    "/home/w1nchie/ros2_ws/src/seabed_slam/data/missouri_i64/"
    "missouri_i64_featured_crop.txt"
)


def parse_meta(path: Path) -> Dict[str, str]:
    raw = path.read_text()
    raw = raw.replace("\\n", "\n")
    values = {}
    for line in raw.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    return values


class BathymetryReference:
    def __init__(
        self,
        source_csv: str = DEFAULT_SOURCE_CSV,
        meta_path: str = DEFAULT_META_PATH,
        mesh_scale: float = 0.5,
        coverage_cell_m: float = 0.5,
        inner_margin_m: float = 4.0,
    ) -> None:
        self.source_csv = Path(source_csv)
        self.meta_path = Path(meta_path)
        self.mesh_scale = mesh_scale
        self.coverage_cell_m = coverage_cell_m
        self.inner_margin_m = inner_margin_m

        meta = parse_meta(self.meta_path)
        self.center_x = float(meta["center_x"])
        self.center_y = float(meta["center_y"])
        self.window_x = float(meta["window_x_m"])
        self.window_y = float(meta["window_y_m"])

        data = np.genfromtxt(self.source_csv, delimiter=",", names=True, dtype=float, encoding=None)
        x = data["X"]
        y = data["Y"]
        z = data["Z"]
        mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        mask &= np.abs(x - self.center_x) <= self.window_x / 2.0
        mask &= np.abs(y - self.center_y) <= self.window_y / 2.0

        x = x[mask]
        y = y[mask]
        z = z[mask]

        self.max_source_z = float(np.max(z))
        self.x = (x - self.center_x) * self.mesh_scale
        self.y = (y - self.center_y) * self.mesh_scale
        self.z = (z - self.max_source_z - 0.5) * self.mesh_scale
        self.xy = np.column_stack([self.x, self.y])

        self.linear = LinearNDInterpolator(self.xy, self.z)
        self.nearest = NearestNDInterpolator(self.xy, self.z)

        self.x_min = float(np.min(self.x))
        self.x_max = float(np.max(self.x))
        self.y_min = float(np.min(self.y))
        self.y_max = float(np.max(self.y))
        self.inner_x_min = self.x_min + self.inner_margin_m
        self.inner_x_max = self.x_max - self.inner_margin_m
        self.inner_y_min = self.y_min + self.inner_margin_m
        self.inner_y_max = self.y_max - self.inner_margin_m

        self.gt_cell_count = len({
            (
                math.floor(float(px) / self.coverage_cell_m),
                math.floor(float(py) / self.coverage_cell_m),
            )
            for px, py in self.xy
        })

    def height(self, xy: np.ndarray) -> np.ndarray:
        values = self.linear(xy)
        missing = ~np.isfinite(values)
        if np.any(missing):
            values[missing] = self.nearest(xy[missing])
        return values

    def inside_inner(self, xy: np.ndarray) -> np.ndarray:
        return (
            (xy[:, 0] >= self.inner_x_min)
            & (xy[:, 0] <= self.inner_x_max)
            & (xy[:, 1] >= self.inner_y_min)
            & (xy[:, 1] <= self.inner_y_max)
        )


class RecordedMapReference:
    def __init__(
        self,
        reference_csv: str,
        coverage_cell_m: float = 0.5,
        inner_margin_m: float = 2.0,
    ) -> None:
        data = np.genfromtxt(reference_csv, delimiter=",", names=True, dtype=float, encoding=None)
        self.x = data["x"]
        self.y = data["y"]
        self.z = data["z"]
        mask = np.isfinite(self.x) & np.isfinite(self.y) & np.isfinite(self.z)
        self.x = self.x[mask]
        self.y = self.y[mask]
        self.z = self.z[mask]
        self.xy = np.column_stack([self.x, self.y])

        self.linear = LinearNDInterpolator(self.xy, self.z)
        self.nearest = NearestNDInterpolator(self.xy, self.z)

        self.x_min = float(np.min(self.x))
        self.x_max = float(np.max(self.x))
        self.y_min = float(np.min(self.y))
        self.y_max = float(np.max(self.y))
        x_margin = min(inner_margin_m, max(0.0, (self.x_max - self.x_min) * 0.20))
        y_margin = min(inner_margin_m, max(0.0, (self.y_max - self.y_min) * 0.20))
        self.inner_x_min = self.x_min + x_margin
        self.inner_x_max = self.x_max - x_margin
        self.inner_y_min = self.y_min + y_margin
        self.inner_y_max = self.y_max - y_margin

        self.gt_cell_count = len({
            (
                math.floor(float(px) / coverage_cell_m),
                math.floor(float(py) / coverage_cell_m),
            )
            for px, py in self.xy
        })

    def height(self, xy: np.ndarray) -> np.ndarray:
        values = self.linear(xy)
        missing = ~np.isfinite(values)
        if np.any(missing):
            values[missing] = self.nearest(xy[missing])
        return values

    def inside_inner(self, xy: np.ndarray) -> np.ndarray:
        return (
            (xy[:, 0] >= self.inner_x_min)
            & (xy[:, 0] <= self.inner_x_max)
            & (xy[:, 1] >= self.inner_y_min)
            & (xy[:, 1] <= self.inner_y_max)
        )
