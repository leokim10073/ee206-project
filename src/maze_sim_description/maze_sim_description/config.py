"""Load and validate sim.yaml. Every key is required; a bad value raises ConfigError
whose message starts with the dotted key, e.g. "camera.fps: expected a positive integer"."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class CameraConfig:
    position: tuple
    look_at: tuple
    up: tuple
    resolution: tuple
    fps: int
    hfov_deg: float


@dataclass(frozen=True)
class RobotConfig:
    ur_type: str
    initial_joints: tuple


@dataclass(frozen=True)
class GripperConfig:
    closed_position: float
    force_limit: float


@dataclass(frozen=True)
class Pose:
    xyz: tuple
    rpy: tuple


@dataclass(frozen=True)
class GoalHole:
    center: tuple
    radius: float


@dataclass(frozen=True)
class BoardConfig:
    mesh_dir: str
    units: str
    grasp_offset: Pose
    mass: float
    surface_friction: tuple
    goal_hole: GoalHole


@dataclass(frozen=True)
class BallConfig:
    radius: float
    mass: float
    start: tuple
    friction: tuple


@dataclass(frozen=True)
class PhysicsConfig:
    timestep: float
    real_time_factor: float
    noslip_iterations: int


@dataclass(frozen=True)
class SimConfig:
    camera: CameraConfig
    robot: RobotConfig
    gripper: GripperConfig
    board: BoardConfig
    ball: BallConfig
    physics: PhysicsConfig


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _vec(n):
    def check(v):
        if not (isinstance(v, list) and len(v) == n and all(_is_num(x) for x in v)):
            raise ValueError(f"expected a list of {n} numbers")
        return tuple(float(x) for x in v)
    return check


def _num(lo=None, hi=None, lo_open=True):
    def check(v):
        if not _is_num(v):
            raise ValueError("expected a number")
        if lo is not None and (v <= lo if lo_open else v < lo):
            raise ValueError(f"must be {'>' if lo_open else '>='} {lo}")
        if hi is not None and v > hi:
            raise ValueError(f"must be <= {hi}")
        return float(v)
    return check


def _pos_int(v):
    if not (isinstance(v, int) and not isinstance(v, bool) and v > 0):
        raise ValueError("expected a positive integer")
    return v


def _nonneg_int(v):
    if not (isinstance(v, int) and not isinstance(v, bool) and v >= 0):
        raise ValueError("expected an integer >= 0")
    return v


def _resolution(v):
    if not (isinstance(v, list) and len(v) == 2 and all(isinstance(x, int) and x > 0 for x in v)):
        raise ValueError("expected [width, height] in pixels")
    return tuple(v)


def _friction(v):
    t = _vec(3)(v)
    if any(x < 0 for x in t):
        raise ValueError("friction values must be >= 0")
    return t


def _choice(*options):
    def check(v):
        if v not in options:
            raise ValueError(f"expected one of {', '.join(options)}")
        return v
    return check


def _str(v):
    if not isinstance(v, str) or not v:
        raise ValueError("expected a non-empty string")
    return v


def _section(d, key, schema):
    """Validate dict d against {field: checker or (sub_schema, ctor)}; return kwargs."""
    if not isinstance(d, dict):
        raise ConfigError(f"{key}: expected a mapping")
    for k in d:
        if k not in schema:
            raise ConfigError(f"{key}.{k}: unknown key")
    out = {}
    for field, check in schema.items():
        path = f"{key}.{field}"
        if field not in d:
            raise ConfigError(f"{path}: missing")
        if isinstance(check, tuple):
            sub_schema, ctor = check
            out[field] = ctor(**_section(d[field], path, sub_schema))
        else:
            try:
                out[field] = check(d[field])
            except ValueError as e:
                raise ConfigError(f"{path}: {e}") from None
    return out


SCHEMA = {
    "camera": ({
        "position": _vec(3),
        "look_at": _vec(3),
        "up": _vec(3),
        "resolution": _resolution,
        "fps": _pos_int,
        "hfov_deg": _num(0.0, 179.0),
    }, CameraConfig),
    "robot": ({
        "ur_type": _choice("ur7e"),
        "initial_joints": _vec(6),
    }, RobotConfig),
    "gripper": ({
        "closed_position": _num(0.0, 1.0, lo_open=False),
        "force_limit": _num(0.0),
    }, GripperConfig),
    "board": ({
        "mesh_dir": _str,
        "units": _choice("mm", "m"),
        "grasp_offset": ({"xyz": _vec(3), "rpy": _vec(3)}, Pose),
        "mass": _num(0.0),
        "surface_friction": _friction,
        "goal_hole": ({"center": _vec(2), "radius": _num(0.0)}, GoalHole),
    }, BoardConfig),
    "ball": ({
        "radius": _num(0.0),
        "mass": _num(0.0),
        "start": _vec(2),
        "friction": _friction,
    }, BallConfig),
    "physics": ({
        "timestep": _num(0.0),
        "real_time_factor": _num(0.0),
        "noslip_iterations": _nonneg_int,
    }, PhysicsConfig),
}


def _check_camera_geometry(cam):
    view = np.subtract(cam.look_at, cam.position)
    if np.linalg.norm(view) < 1e-9:
        raise ConfigError("camera.look_at: must differ from camera.position")
    up = np.asarray(cam.up)
    if np.linalg.norm(up) < 1e-9:
        raise ConfigError("camera.up: must be a non-zero vector")
    if np.linalg.norm(np.cross(view / np.linalg.norm(view), up / np.linalg.norm(up))) < 1e-6:
        raise ConfigError("camera.up: must not be parallel to the viewing direction")


def load_sim_config(path):
    path = Path(path)
    try:
        raw = yaml.safe_load(path.read_text())
    except yaml.YAMLError as e:
        raise ConfigError(f"sim.yaml: not valid YAML ({e})") from None
    if not isinstance(raw, dict):
        raise ConfigError("sim.yaml: expected a mapping at the top level")
    for k in raw:
        if k not in SCHEMA:
            raise ConfigError(f"{k}: unknown key")
    sections = {}
    for name, (schema, ctor) in SCHEMA.items():
        if name not in raw:
            raise ConfigError(f"{name}: missing")
        sections[name] = ctor(**_section(raw[name], name, schema))
    cfg = SimConfig(**sections)
    _check_camera_geometry(cfg.camera)
    return cfg
