import logging
from pathlib import Path

import mujoco as mj
import numpy as np

from humkit.error import InvalidValueError, ModelLoadError, StateDimensionError

logger = logging.getLogger(__name__)

MENAGERIE_DIR = Path(__file__).resolve().parents[3] / "mujoco_menagerie"


class Robot:
    def __init__(self, model: mj.MjModel, name: str | None = None):
        self._model = model
        self._data = mj.MjData(self._model)
        self._name = name if name is not None else self._model.name

    def __repr__(self) -> str:
        return (
            f"Robot(name={self._name!r}, nq={self._model.nq}, "
            f"nv={self._model.nv}, nu={self._model.nu})"
        )

    def step(self, n: int = 1) -> None:
        for _ in range(n):
            mj.mj_step(self._model, self._data)

    def set_state(self, q: np.ndarray, dq: np.ndarray) -> None:
        q = np.asarray(q, dtype=float)
        dq = np.asarray(dq, dtype=float)
        if q.shape != (self._model.nq,):
            raise StateDimensionError(f"Expected q of shape ({self._model.nq},), got {q.shape}")
        if dq.shape != (self._model.nv,):
            raise StateDimensionError(f"Expected dq of shape ({self._model.nv},), got {dq.shape}")

        q = q.copy()
        q[3:7] /= np.linalg.norm(q[3:7])

        self._data.qpos[:] = q
        self._data.qvel[:] = dq
        mj.mj_forward(self._model, self._data)

    def _mujoco_handles(self) -> tuple[mj.MjModel, mj.MjData]:
        return self._model, self._data

    def set_ctrl(self, u: np.ndarray) -> None:
        u = np.asarray(u, dtype=float)
        if u.shape != (self._model.nu,):
            raise StateDimensionError(f"Expected u of shape ({self._model.nu},), got {u.shape}")
        self._data.ctrl[:] = u

    @property
    def joint_names(self) -> list[str]:
        return [
            self._model.joint(i).name
            for i in range(self._model.njnt)
            if self._model.joint(i).type[0] != mj.mjtJoint.mjJNT_FREE
        ]

    @property
    def base_pose(self) -> tuple[np.ndarray, np.ndarray]:
        position = self._data.qpos[0:3].copy()  # x, y, z
        orientation = self._data.qpos[3:7].copy()  # quaternion (w, x, y, z)
        return position, orientation

    @property
    def q(self) -> np.ndarray:
        return self._data.qpos.copy()

    @property
    def dq(self) -> np.ndarray:
        return self._data.qvel.copy()

    @property
    def nu(self) -> int:
        return self._model.nu

    @property
    def bias_torques(self) -> np.ndarray:
        return self._data.qfrc_bias[6:].copy()

    @classmethod
    def from_menagerie(
        cls, robot_name: str, hanging: bool = False, torque_control: bool = False
    ) -> "Robot":
        robot_dir = MENAGERIE_DIR / robot_name
        scene_file = robot_dir / "scene.xml"

        if not scene_file.exists():
            raise ModelLoadError(
                f"Scene file not found: {scene_file}. "
                "Clone mujoco_menagerie next to the humkit repo."
            )
        try:
            spec = mj.MjSpec.from_file(str(scene_file))
            if hanging:
                weld = spec.add_equality()
                weld.type = mj.mjtEq.mjEQ_WELD
                weld.objtype = mj.mjtObj.mjOBJ_BODY
                weld.name1 = "pelvis"

            if torque_control:
                for act in spec.actuators:
                    joint = spec.joint(act.target)
                    lo, hi = joint.actfrcrange
                    if hi <= lo:
                        raise InvalidValueError(f"No torque limit for joint {act.target!r}")
                    act.gaintype = mj.mjtGain.mjGAIN_FIXED
                    act.gainprm[0] = 1.0
                    act.biastype = mj.mjtBias.mjBIAS_NONE
                    act.biasprm[:3] = 0.0
                    act.ctrlrange = [lo, hi]
            model = spec.compile()
        except ValueError as err:
            raise ModelLoadError(f"Could not load {robot_name}: {err}") from err

        logger.info(
            "Loaded %s (hanging=%s, torque_control=%s)", robot_name, hanging, torque_control
        )
        return cls(model, name=robot_name)
