import matplotlib.pyplot as plt
import numpy as np


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def small_rotation(axis, angle):
    axis = np.asarray(axis) / np.linalg.norm(axis)
    return np.concatenate([[np.cos(angle / 2)], np.sin(angle / 2) * axis])


def simulate(n_steps, dtype=np.float64, normalise=False):
    q = np.array([1.0, 0.0, 0.0, 0.0], dtype=dtype)
    dq = small_rotation([0.3, 0.5, 0.8], 0.001).astype(dtype)
    errors = []
    for _ in range(n_steps):
        q = quat_mul(q, dq).astype(dtype)
        if normalise:
            q /= np.linalg.norm(q)
        errors.append(abs(np.linalg.norm(q) - 1.0))
    return np.array(errors)


n = 10000
plt.semilogy(simulate(n, np.float64), label="float64")
plt.semilogy(simulate(n, np.float32), label="float32")
plt.semilogy(simulate(n, np.float32, normalise=True), label="float32 + normalise")
plt.xlabel("rotation step")
plt.ylabel("| ‖q‖ − 1 |")
plt.legend()
plt.show()
