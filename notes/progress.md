# Bootcamp progress and decisions

Handoff from the planning chat. Read with `CLAUDE.md` and `docs/bootcamp-plan.md`.

## Status (25 Nov 2026)

- Days 1–7 done; tagged **v0.1.0**.
- Day 8 (types): `mypy --strict` passes on `src/` (branch `chore/mypy-strict`). `FloatArray` alias in
  `humkit/types.py`; `RobotLike` Protocol in `controllers/controller_abc.py`; mypy runs in pre-commit.
  Still open: the 3-line note on nominal vs structural typing (`notes/day08.md`).
- **Next: Day 9**, the kinematic tree. Read the tree from `MjModel`, write recursive and
  explicit-stack FK, check it against MuJoCo to 1e-9, then add the production path
  (`Robot.forward_kinematics` via a scratch `MjData` + `mj_kinematics`).
- The GitHub repo was renamed from `humkit` to `humanoid_toolkit`; the Python package is still `humkit`.

## Working style

- Hints first, then code if asked. Strict senior-engineer reviews: bugs, then design, then style.
- Explain concepts in plain notation first; I'm an electronics engineer, not a CS major.
- Learn it, then ship it: build each algorithm by hand, validate against a library,
  production code calls the library.
- Hand-hold early, with specific resources and waypoints.

## Design decisions (and why)

| Decision | Reason |
| --- | --- |
| `Robot` is the only gateway to MuJoCo; others use `_mujoco_handles()` | Encapsulation: swap the backend (C++ in Week 3) without callers changing |
| `Robot.__init__` takes a compiled `MjModel`; class methods handle files | Lets tests and `MjSpec` edits build robots without files |
| `torque_control=True` converts the G1's position servos to torque motors via `MjSpec` | Our own PD, gravity comp and the Day 12 QP need direct torque control |
| Torque limits read from the joint `actuatorfrcrange`, else the actuator `forcerange` | Menagerie has stored them in both places across versions |
| `hanging=True` welds the pelvis to the world | Test controllers without balance |
| Controllers return joint torques (length `nu`); they read `q[7:]`, `dq[6:]` | `q` is 7 base + 29 joints, `dq` is 6 base + 29 joints |
| `CompositeController` sums children (composition, not inheritance) | All children return torques |
| `sim.run_viewer(robot, controller, viewer=..., steps=...)` is a plain function | It holds no state, so it doesn't need to be a class |
| YAML configs in `configs/`, built by `config.build_controller` (simple factory) | Switching controllers needs no code change; becomes a registry on Day 11 |
| `HumkitError` base; `ModelLoadError`, `StateDimensionError(ValueError)`, `InvalidValueError(ValueError)`, `ControllerError`, `ConfigError(ValueError)` | `except HumkitError` separates expected problems from bugs; multiple inheritance keeps `except ValueError` working |
| Libraries log via `logging.getLogger(__name__)`; only `run.py` configures logging | The application decides output, not the library |
| `geometry/geom_tools.py` raw-array maths; `SO3`/`SE3` frozen dataclasses on top | Low-level functions are portable to C++; classes guarantee valid state |
| Quaternions (w, x, y, z), as MuJoCo | Read `qpos[3:7]` directly |
| Solà naming: `Exp`/`Log` on vectors, `exp`/`log` on Lie-algebra matrices; twist `[rho, theta]` | Code maps one-to-one to the paper |
| `SE3` has an `SO3` (composition) | Inheritance would let `rotation @ pose` silently drop the translation |
| Arrays inside `SO3`/`SE3` are read-only | `frozen=True` alone doesn't stop `R.quat[0] = 5` |
| `FloatArray = NDArray[np.float64]`; MuJoCo untyped (`ignore_missing_imports`), converted to typed values inside `Robot` | One `Any` boundary, in the one module that touches MuJoCo |
| Controllers take a `RobotLike` `Protocol`, not `Robot` | Structural typing: `FakeRobot` satisfies it with no inheritance |
| `load_config` raises `ConfigError` unless the YAML top level is a mapping | `yaml.safe_load` returns `Any`; mypy surfaced a missing runtime check |

## Gotchas already hit

- **ROS Humble is sourced in the shell**, which breaks pytest through its plugin: run `PYTHONPATH= uv run pytest`.
  ROS Humble is Python 3.10 and the venv is 3.12, which matters for the Day 25 bridge.
- `MUJOCO_GL=egl` in the shell breaks the interactive viewer; unset it.
- `uv init --lib .` at the repo root (a name creates a nested folder).
- Stage everything before committing, or pre-commit's auto-fixes clash with unstaged changes and get rolled back.
- `ctrl = 0` on position servos means "go to 0 rad", not "no torque".
- Gravity compensation alone can't make the G1 stand: contact forces load the joints. Standing needs the Day 12 QP.
- `assert_allclose` has `atol=0` by default; set `atol` when expected values contain zeros.
- Test against independently known values (hand-worked or geometric facts), never the formula under test.
- `quat_log` flips `q` to `w >= 0` (q and −q are the same rotation).

## Notes on concepts covered

Classes and encapsulation, properties, class methods (alternative constructors, `cls`),
ABCs, dynamic dispatch, `super()` and the MRO, composition vs inheritance, exceptions and
logging, floating-point drift and normalisation, test doubles (fakes vs mocks), fixtures,
coverage, quaternions (sandwich product, half angles, Euler–Rodrigues), Solà's Lie theory
(hat/vee, exp/Log, plus/minus, Adj, the SE(3) left Jacobian), Euler-angle extraction.
