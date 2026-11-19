import sys
from pathlib import Path

from humkit.config import build_controller, load_config
from humkit.robot import Robot
from humkit.sim import run_viewer

CONFIG_DIR = Path(__file__).resolve().parents[2] / "configs"
DEFAULT_CONFIG = "config"


def resolve_config(arg: str | None) -> Path:
    """Accept a file path, a bare config name, or nothing (use the default)."""
    if arg is None:
        arg = DEFAULT_CONFIG
    path = Path(arg)
    if path.is_file():
        return path
    named = CONFIG_DIR / f"{arg.removesuffix('.yaml')}.yaml"
    if named.is_file():
        return named
    sys.exit(f"Config not found: {arg} (looked in {CONFIG_DIR})")


def main() -> None:
    cfg = load_config(resolve_config(sys.argv[1] if len(sys.argv) > 1 else None))
    robot = Robot.from_menagerie(
        cfg["robot"]["name"],
        hanging=cfg["robot"]["hanging"],
        torque_control=cfg["robot"]["torque_control"],
    )
    controller = build_controller(cfg["controller"], robot)
    run_viewer(robot, controller, viewer=cfg["sim"]["viewer"], steps=cfg["sim"]["steps"])


if __name__ == "__main__":
    main()
