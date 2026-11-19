from pathlib import Path
from typing import Any

import yaml

from humkit.controllers.composition import CompositeController
from humkit.controllers.controller_abc import ControllerABC
from humkit.controllers.gravity_comp import GravityCompController
from humkit.controllers.pd_control import PDController
from humkit.robot import Robot


def load_config(path: str | Path) -> dict[str, Any]:
    with open(path) as f:
        return yaml.safe_load(f)


def build_controller(cfg: dict[str, Any], robot: Robot) -> ControllerABC:
    kind = cfg["type"]
    if kind == "pd":
        return PDController(kp=cfg["kp"], kd=cfg["kd"], q_ref=robot.q[7:])
    if kind == "gravity_comp":
        return GravityCompController()
    if kind == "composite":
        return CompositeController([build_controller(c, robot) for c in cfg["children"]])
    raise ValueError(f"Unknown controller type: {kind!r}")
