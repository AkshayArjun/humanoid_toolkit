"""Shared type aliases."""

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]
"""A NumPy array of float64. Shape is not tracked by the type checker; check it at runtime."""
