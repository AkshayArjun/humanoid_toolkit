# Day 10: Jacobians and IK

<!-- Your 5 bullet notes go here. -->

## Benchmark: `inv` vs `solve` (`examples/bench_inv_vs_solve.py`, 1 thread)

| n | `inv(A) @ b` | `solve(A, b)` | inv / solve |
| ---: | ---: | ---: | ---: |
| 6 | 4.4 µs | 3.5 µs | 1.3x |
| 29 | 13.1 µs | 7.8 µs | 1.7x |
| 100 | 112 µs | 44 µs | 2.6x |
| 300 | 1.8 ms | 0.52 ms | 3.5x |
| 1000 | 56 ms | 15 ms | 3.7x |

- Both are O(n³), but `inv` does about 3x the work: it factorises A (LU, ~2/3 n³ flops) and then
  solves for all n columns of the identity (~4/3 n³ more), only to multiply by one vector.
  `solve` factorises once and does two triangular solves for one right-hand side (O(n²)).
  From n = 300 to 1000 (3.3x), time grows ~30x, close to 3.3³ = 37.
- At n = 6 the gap is small because Python and LAPACK call overhead dominate the flops.
- Accuracy matters more than speed. On a Hilbert matrix (condition number 1.6e13), `inv`
  leaves a residual |Ax - b| of 4.8e-5; `solve` leaves 5.7e-16. Near an arm singularity,
  J Jᵀ is ill-conditioned in the same way.

## Big-O of one IK iteration (m = 6 task rows, n = 29 joints)

| Step | Cost | Measured |
| --- | --- | ---: |
| `robot.fk(q)` | O(bodies), plus 31 Python `SE3` objects | 142 µs |
| `robot.jac(q, body)` | O(depth · n) in MuJoCo's C code | 7 µs |
| `world_error` | O(1): a few 3x3 products and one Log | 34 µs |
| `J @ J.T` | O(m² n) | 1.2 µs |
| `solve` (6x6) | O(m³) | 3.5 µs |
| `J.T @ x` | O(m n) | 0.9 µs |

- Solving the m x m system `J Jᵀ` instead of the n x n system `Jᵀ J` costs O(m³) = 216 instead
  of O(n³) = 24 389 flops. The trick works because the task (6) is smaller than the joints (29).
- The linear algebra is ~6 µs of a ~190 µs iteration. The bottleneck is Python object creation:
  `fk` builds an `SE3` for all 31 bodies when IK needs one. Big-O hides constant factors;
  profiling (Day 15) is how you find them.
