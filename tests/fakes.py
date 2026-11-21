import numpy as np


class FakeRobot:
    def __init__(
        self,
        nu: int = 3,
        q: np.ndarray | None = None,
        dq: np.ndarray | None = None,
    ) -> None:
        self.nu = nu
        self.q = np.zeros(7 + nu) if q is None else np.asarray(q, dtype=float)
        self.dq = np.zeros(6 + nu) if dq is None else np.asarray(dq, dtype=float)
        self.bias_torques = np.zeros(nu)
