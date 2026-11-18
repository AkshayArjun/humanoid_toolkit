from pathlib import Path

import mujoco as mj
import numpy as np

MENAGERIE_DIR = Path(__file__).resolve().parents[3] / "mujoco_menagerie"


class Robot:
    def __init__(self, model_path: str, name: str | None = None):
        self._model = mj.MjModel.from_xml_path(model_path)
        self._data = mj.MjData(self._model)
        self._name = name if name is not None else Path(model_path).stem

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
            raise ValueError(f"Expected q of shape ({self._model.nq},), got {q.shape}")
        if dq.shape != (self._model.nv,):
            raise ValueError(f"Expected dq of shape ({self._model.nv},), got {dq.shape}")

        q = q.copy()
        q[3:7] /= np.linalg.norm(q[3:7])

        self._data.qpos[:] = q
        self._data.qvel[:] = dq
        mj.mj_forward(self._model, self._data)

    def _mujoco_handles(self) -> tuple[mj.MjModel, mj.MjData]:
        return self._model, self._data

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

    @classmethod
    def from_menagerie(cls, robot_name: str):
        robot_dir = MENAGERIE_DIR / robot_name
        scene_file = robot_dir / "scene.xml"

        if not scene_file.exists():
            raise FileNotFoundError(
                f"Scene file not found: {scene_file}. "
                "Clone mujoco_menagerie next to the humkit repo."
            )

        return cls(str(scene_file), name=robot_name)
