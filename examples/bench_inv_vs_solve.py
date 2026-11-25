"""Day 10: np.linalg.inv vs np.linalg.solve, and where an IK iteration spends its time.

Run: uv run python examples/bench_inv_vs_solve.py
"""

import os

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")  # one core, so the O(n³) scaling is visible

import timeit  # noqa: E402

import numpy as np  # noqa: E402

from humkit.kinematics.ik import dls_step, world_error  # noqa: E402
from humkit.robot import Robot  # noqa: E402

HAND = "left_wrist_yaw_link"


def median_us(fn, number: int) -> float:
    runs = timeit.repeat(fn, number=number, repeat=7)
    return float(np.median(runs)) / number * 1e6


def spd_matrix(n: int, rng: np.random.Generator) -> np.ndarray:
    """A well-conditioned symmetric positive-definite matrix, like J Jᵀ + λ²I."""
    m = rng.normal(size=(n, n))
    return m @ m.T + n * np.eye(n)


def speed() -> None:
    print("1. Solving A x = b once (median time)")
    print(f"{'n':>6} {'inv(A) @ b':>14} {'solve(A, b)':>14} {'inv / solve':>12}")
    rng = np.random.default_rng(0)
    for n in [6, 29, 100, 300, 1000]:
        a, b = spd_matrix(n, rng), rng.normal(size=n)
        number = max(1, 20000 // n**2)
        t_inv = median_us(lambda a=a, b=b: np.linalg.inv(a) @ b, number)
        t_solve = median_us(lambda a=a, b=b: np.linalg.solve(a, b), number)
        print(f"{n:>6} {t_inv:>12.1f}µs {t_solve:>12.1f}µs {t_inv / t_solve:>11.2f}x")


def accuracy() -> None:
    print("\n2. Accuracy on an ill-conditioned matrix (Hilbert, n = 10)")
    n = 10
    a = 1.0 / (np.arange(n)[:, None] + np.arange(n)[None, :] + 1.0)
    x_true = np.ones(n)
    b = a @ x_true
    print(f"   condition number {np.linalg.cond(a):.1e}")
    for name, x in [("inv", np.linalg.inv(a) @ b), ("solve", np.linalg.solve(a, b))]:
        residual = np.linalg.norm(a @ x - b)
        error = np.linalg.norm(x - x_true)
        print(f"   {name:>5}: residual |Ax - b| = {residual:.1e}, error |x - x_true| = {error:.1e}")


def ik_breakdown() -> None:
    print("\n3. One IK iteration on the G1 (m = 6 task rows, n = 29 joints)")
    robot = Robot.from_menagerie("unitree_g1")
    q = robot.q
    target = robot.fk(q)[HAND]
    jac = robot.jac(q, HAND)[:, 6:]
    e = world_error(robot.fk(q)[HAND], target)
    jjt = jac @ jac.T + 1e-4 * np.eye(6)
    x = np.linalg.solve(jjt, e)

    steps = [
        ("robot.fk(q)  (MuJoCo + 31 SE3 objects)", lambda: robot.fk(q)),
        ("robot.jac(q, body)  (MuJoCo)", lambda: robot.jac(q, HAND)),
        ("world_error", lambda: world_error(target, target)),
        ("J @ J.T   O(m²n)", lambda: jac @ jac.T),
        ("solve 6x6   O(m³)", lambda: np.linalg.solve(jjt, e)),
        ("inv 6x6 @ e   O(m³)", lambda: np.linalg.inv(jjt) @ e),
        ("J.T @ x   O(mn)", lambda: jac.T @ x),
        ("dls_step total", lambda: dls_step(jac, e, 1e-2)),
    ]
    for name, fn in steps:
        print(f"   {name:<42} {median_us(fn, 2000):>8.2f}µs")


if __name__ == "__main__":
    speed()
    accuracy()
    ik_breakdown()
