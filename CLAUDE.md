# humkit

Whole-body control stack for the Unitree G1 humanoid in MuJoCo, built during a
30-day coding bootcamp. The plan is in @docs/bootcamp-plan.md, the step-by-step
waypoints for each day are in @docs/daily-guides.md, and where I am, what's been
decided and why is in @notes/progress.md. Update progress.md at the end of each day.

## How to work with me (this is a learning project)

- I'm here to learn, not just to get working code. Explain the concept first, then
  let me write the code. Give hints before full solutions unless I ask for the code.
- Review my code like a strict senior engineer: bugs first, then design, then style.
- Prefer independently computed expected values in tests (worked out by hand or from
  known geometry), never the same formula the code uses.
- Tie explanations back to the day's OOP or core CS concept where it fits.

## Commands

- Run tests: `PYTHONPATH= uv run pytest -v` (the empty PYTHONPATH keeps ROS Humble's
  sourced packages out of the venv; otherwise its pytest plugin breaks the run)
- Coverage: `PYTHONPATH= uv run pytest --cov=humkit --cov-report=term-missing`
- Lint and format: `uv run ruff check --fix src/ tests/` and `uv run ruff format src/ tests/`
- Run a config: `uv run python -m humkit.run <config-name-or-path>`
- Commit only through feature branches and PRs; `main` is protected. Conventional Commits.

## Layout

- `src/humkit/robot.py`: `Robot`, the only gateway to MuJoCo. Nothing outside it touches
  `MjData`; internal adapters use `_mujoco_handles()`.
- `src/humkit/controllers/`: `ControllerABC` plus PD, gravity comp and composite controllers.
- `src/humkit/geometry/`: `geom_tools.py` (raw-array maths), `so3.py`, `se3.py`.
- `src/humkit/sim.py` (`run_viewer`), `config.py`, `run.py`; YAML configs in `configs/`.
- `src/humkit/error.py`: `HumkitError` hierarchy.
- `tests/` (with `fakes.py` and a session `g1` fixture in `conftest.py`), `examples/`, `notes/`.
- MuJoCo Menagerie is cloned next to the repo (`../mujoco_menagerie`).

## Conventions and decisions so far

- Quaternions are (w, x, y, z), matching MuJoCo.
- Geometry follows Solà et al., "A micro Lie theory for state estimation in robotics":
  capital `Exp`/`Log` work on vectors, lowercase `exp`/`log` on Lie-algebra matrices;
  twists are `[rho, theta]` (linear first). `SO3` and `SE3` are frozen dataclasses with
  read-only arrays. `SE3` *has an* `SO3` (composition, not inheritance).
- The G1 loads with `torque_control=True` to turn its position servos into torque motors;
  torque limits come from each joint's `actuatorfrcrange`. `hanging=True` welds the pelvis.
- State arrays: `q` is length 36 (7 base + 29 joints), `dq` length 35 (6 + 29);
  controllers use `q[7:]` and `dq[6:]`.
- Libraries log with `logging.getLogger(__name__)`; only `run.py` configures logging.
