# Daily guides

## How to use these guides

Every day has the same four parts, so you always know what to read, what to build next, and where to look when stuck.

- **Read:** exact resources, with the chapter or section. Read these before you touch code.
- **Waypoints:** numbered checkpoints. Each ends with something you can run or see. Don't move on until the current one works.
- **If stuck:** a hint first, then a reference to compare against. Look at the reference only after 30 minutes of trying.
- **Ask Claude:** a ready-made prompt for when you need a nudge, an explanation, or a review.

**Support fades on purpose.** Week 1 has 5–7 small waypoints per day. By Week 4 there are 3–4 bigger ones, because by then you'll be breaking work down yourself, which is the skill you're building.

**Daily tracking:** tick a waypoint's box in this tab when it works. A day is done when all its boxes are ticked and the PR is merged.

## Week 1 guides (Days 1–7)

### Day 1: Git workflow + git internals

**Read:**

- [Pro Git](https://git-scm.com/book/en/v2), chapter 3 (Branching) and sections 10.1–10.3 (Git Internals: objects and references)
- [Learn Git Branching](https://learngitbranching.js.org): the Main levels 1–4 (interactive, about 30 minutes)
- [Conventional Commits](https://www.conventionalcommits.org), the summary section only
- [MuJoCo Python bindings](https://mujoco.readthedocs.io/en/stable/python.html), the "Tutorial notebook" and "Passive viewer" parts

**Waypoints:**

- [x] Create the `humkit` repo on GitHub with a README and an MIT license. Clone it.
- [x] Turn on branch protection for `main`: require a PR before merging.
- [x] Clone [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) next to it and find `unitree_g1/scene.xml`.
- [x] Branch `feat/viewer`. Write a 15-line script: `MjModel.from_xml_path` → `MjData` → `mujoco.viewer.launch_passive` → loop calling `mj_step`.
- [x] Print `model.nq`, `model.nv`, `model.nu` and every joint and actuator name. Write them in `notes/day01.md`. Note what kind of actuators the G1 uses (check `model.actuator_biastype`): position servos or torque motors.
- [x] Commit with `feat: add G1 viewer script`, open a PR, self-review it, merge.
- [x] Run `git cat-file -p HEAD` and follow it to a tree and a blob. Write 3 lines on what you saw.

**If stuck:** the G1 falls straight away in the viewer. That's correct, since nothing is controlling it yet. If the viewer won't open over SSH or Docker, run it on your laptop for now.

**Ask Claude:** "Explain what `nq` vs `nv` means for a floating-base robot, using the G1's numbers: \[paste your printout\]."

### Day 2: Project layout + how Python runs code

**Read:**

- [Python Packaging User Guide: Packaging Python Projects](https://packaging.python.org/en/latest/tutorials/packaging-projects/)
- [uv docs](https://docs.astral.sh/uv/), the "Projects" guide
- [Ruff docs](https://docs.astral.sh/ruff/) and the [pre-commit](https://pre-commit.com) quick start
- [Python tutorial, chapter 6: Modules](https://docs.python.org/3/tutorial/modules.html)

**Waypoints:**

- [x] `uv init --lib .` at the repo root (the dot means "this folder"; a name would create a nested folder). Check you get `src/humkit/__init__.py` and `pyproject.toml`.
- [x] `uv add mujoco numpy`, then `uv add --dev pytest ruff mypy pre-commit`.
- [x] Move yesterday's script into `src/humkit/view.py` with a `main()` function, runnable as `python -m humkit.view`.
- [x] Add `.pre-commit-config.yaml` with ruff (lint and format) and the basic file-hygiene hooks. Run `pre-commit install`.
- [x] Deliberately commit badly formatted code and watch the hook block it.
- [x] Run `python -m dis` on a tiny function and note what bytecode looks like. Explain in 3 lines why `import humkit` works only after `pip install -e .`.

**If stuck:** if `python -m humkit.view` says the module isn't found, you're in the wrong environment. Use `uv run python -m humkit.view`.

**Ask Claude:** "Review my `pyproject.toml` and pre-commit config as a strict senior Python engineer would."

### Day 3: Clean code + OOP I (classes and encapsulation)

**Read:**

- [Python tutorial, chapter 9: Classes](https://docs.python.org/3/tutorial/classes.html)
- *A Philosophy of Software Design*, chapters 4–5 (deep modules, information hiding). No copy? Use the free alternatives: John Ousterhout's "A Philosophy of Software Design" talk (Talks at Google, YouTube, first 30 minutes) and Parnas (1972), "On the Criteria To Be Used in Decomposing Systems into Modules"
- [MuJoCo docs: MjData](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjdata) — skim `qpos`, `qvel`, `ctrl`, `xpos`, `xquat`

**Waypoints:**

- [x] Write the class design first, in `notes/day03.md`: which data `Robot` owns, which methods it exposes, and which invariants it keeps (e.g. quaternions stay normalised, `q` always has length `nq`).
- [x] Create `src/humkit/robot.py` with `Robot.from_menagerie("unitree_g1")`.
- [x] Add properties `joint_names`, `q`, `dq`, `base_pose`, plus `set_state(q, dq)` and `step(n=1)`.
- [x] Add `joint_limits` that returns a dict of name to (low, high), read from `model.jnt_range`.
- [x] Add `__repr__` that prints a useful summary (e.g. "Robot(g1, nq=36, nv=35, nu=29)"; use your real numbers).
- [x] Rewrite `view.py` to use only `Robot`, with no direct `MjData` access.

**If stuck:** keep `MjModel` and `MjData` as private attributes (`self._model`). If a caller needs something, add a method for it. That's encapsulation in practice.

**Ask Claude:** "Here's my `Robot` class. Which responsibilities don't belong in it?"

### Day 4: Configuration + OOP II (inheritance, ABCs, polymorphism)

**Read:**

- [Python `abc` module docs](https://docs.python.org/3/library/abc.html)
- [Refactoring.Guru: Strategy pattern](https://refactoring.guru/design-patterns/strategy), as a preview of Day 11
- Search "Liskov substitution principle" and read one short article with a code example

**Waypoints:**

- [x] Write `Controller(ABC)` with an abstract `compute(robot) -> np.ndarray` returning one command per actuator.
- [x] Add a "hanging" mode so the G1 can't fall: a weld equality from the pelvis to the world in a small wrapper XML that includes the scene.
- [x] `PDController(kp, kd, q_ref)`: hold the G1's default standing pose (the `home` keyframe if the model has one).
- [x] `GravityCompController`: send the gravity torques. Hint: at zero velocity, `data.qfrc_bias` is exactly gravity.
- [x] `CompositeController([...])` that sums children. That's composition, not inheritance.
- [x] Load which controller to use from a small YAML file. Switching controllers should need no code change.
- [x] Turn hanging mode off and see how long the G1 stays standing with PD plus gravity comp. Write the time in your notes.

**If stuck:** if the G1 uses position actuators, `ctrl` is a target angle, not a torque. In that case PD is already done by the actuator, so your controller outputs targets. Check what you found on Day 1.

**Ask Claude:** "Is `CompositeController` a good use of composition? What would break if I had used inheritance instead?"

### Day 5: Testing + test doubles

**Read:**

- [pytest: Get started](https://docs.pytest.org/en/stable/getting-started.html), plus the pages on fixtures and parametrize
- Search "Martin Fowler Mocks Aren't Stubs" and read the first half

**Waypoints:**

- [x] Create `tests/` and add `uv add --dev pytest-cov`.
- [x] Write a `g1` fixture that loads the robot once per test session.
- [x] Test `Robot`: state round-trips through `set_state`, joint limits match the XML, and `step()` advances `data.time`.
- [x] Make a fake robot (a tiny class with `q`, `dq`, `nu`) and unit-test `PDController` with it. That's a test double, and it runs in milliseconds.
- [x] Parametrize one test over 3 different gain sets.
- [x] Run `pytest --cov=humkit` and push coverage to 85% or more.

**If stuck:** for what to test, ask "what would a bug here look like?" and write a test that catches it.

**Ask Claude:** "Here are my tests. Which important behaviours am I not testing?"

### Day 6: Errors and logging + floating point

**Read:**

- [Python tutorial: Floating-Point Arithmetic](https://docs.python.org/3/tutorial/floatingpoint.html)
- [Python logging HOWTO](https://docs.python.org/3/howto/logging.html), the basic tutorial
- Optional: Goldberg, "What Every Computer Scientist Should Know About Floating-Point Arithmetic", sections 1.1–1.4

**Waypoints:**

- [x] Create `errors.py` with `HumkitError` and subclasses such as `ModelLoadError`, `StateDimensionError` and `ControllerError`.
- [x] Validate inputs at the boundary: wrong-length arrays, NaNs and an unknown robot name raise clear errors.
- [x] Replace every `print` with a module-level `logger = logging.getLogger(__name__)`.
- [x] In a notebook, show that `0.1 + 0.2 != 0.3` and that a quaternion drifts off unit length after 10,000 small rotations. Fix it with normalisation.
- [x] Replace every float `==` in your code and tests with `np.isclose` or `pytest.approx`, with a stated tolerance.

**If stuck:** a good error message says what was wrong, what was expected, and how to fix it.

**Ask Claude:** "Is my exception hierarchy too deep or too shallow for this codebase?"

### Day 7: Code review + OOP III (value objects, operator overloading)

**Read:**

- Solà et al., [A micro Lie theory for state estimation in robotics](https://arxiv.org/abs/1812.01537), sections II–III as a refresher
- [Python data model: emulating numeric types](https://docs.python.org/3/reference/datamodel.html#emulating-numeric-types), for `__matmul__`
- [Google engineering practices: what to look for in a code review](https://google.github.io/eng-practices/review/reviewer/looking-for.html)

**Waypoints:**

- [x] Write a frozen dataclass `SO3` that stores a unit quaternion, with `@` for composition, `inverse()`, `exp()` and `log()`.
- [x] Write `SE3` the same way, built on top of `SO3`.
- [x] Write tests: `exp(log(T)) ≈ T` for 1,000 random poses, `T @ T.inverse() ≈ identity`, and composition is associative.
- [x] Check your `SE3` against MuJoCo's body poses (`xpos`, `xquat`) for 3 bodies.
- [x] Review all Week 1 code against the checklist on the main tab. Fix the 3 worst issues.
- [x] Write `CHANGELOG.md` and tag `v0.1.0`.

**If stuck:** near zero rotation, `log` divides by a tiny angle. Use the Taylor expansion when the angle is below about 1e-8. Solà's paper covers this.

**Ask Claude:** "Review my `se3.py` for numerical edge cases."

## Week 2 guides (Days 8–14)

### Day 8: Type hints + type systems

**Read:**

- [mypy cheat sheet](https://mypy.readthedocs.io/en/stable/cheat_sheet_py3.html)
- [typing docs: Protocols](https://typing.readthedocs.io/en/latest/spec/protocol.html), the intro and examples
- NumPy typing: [`numpy.typing`](https://numpy.org/doc/stable/reference/typing.html), the `NDArray` section

**Waypoints:**

- [x] Add `[tool.mypy] strict = true` to `pyproject.toml` and run it. Expect many errors.
- [x] Fix modules one at a time, starting with `se3.py`, then `robot.py`, then the controllers.
- [x] Define a `RobotLike` `Protocol` and type your controllers against it. Your test fake from Day 5 should now satisfy it without inheriting from anything (structural typing).
- [x] Add mypy to pre-commit.
- [ ] Write 3 lines on nominal vs structural typing, using `Controller` (ABC) and `RobotLike` (Protocol) as the examples.

**If stuck:** use `NDArray[np.float64]` for arrays. Don't sprinkle `Any`; if MuJoCo types are missing, wrap them in one place and silence mypy there only.

**Ask Claude:** "Why does mypy reject this line? \[paste error and code\]"

### Day 9: Parsing + trees and graph traversal

**Read:**

- [`xml.etree.ElementTree` docs](https://docs.python.org/3/library/xml.etree.elementtree.html)
- [MuJoCo XML reference: body, joint, geom](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body)
- *Modern Robotics*, chapter 4 (forward kinematics). Free book and videos: [Modern Robotics](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)

**Waypoints:**

- [x] Write a `Link` node class (name, parent, children, joint axis, offset pose as `SE3`).
- [x] Read the body tree from MjModel (body\_parentid, body\_pos, body\_quat, jnt\_axis) into a tree of `Link`s. Print it as an indented tree.
- [x] Write recursive DFS forward kinematics, then rewrite it with an explicit stack. Compare them.
- [x] Compute a topological order once and do FK as one loop over it. That's the fast version.
- [x] Test: every body's pose matches MuJoCo's `xpos`/`xquat` to 1e-9 for 50 random joint configurations.

**Production path:**

- [x] Add `Robot.forward_kinematics(q) -> dict[str, SE3]` that writes `q` into a scratch `MjData` (created once in `__init__`) and calls `mj_kinematics`. The live simulation is never touched.
- [x] Move your tree FK to a reference module (e.g. `humkit/kinematics/reference.py`) and keep the 1e-9 test comparing the two.

**If stuck:** MJCF can use `<default>` classes and `<include>` files, which make raw parsing hard. Start by reading the tree structure from `MjModel` (`body_parentid`, `jnt_axis`, `body_pos`, `body_quat`). Parse the XML yourself only as a stretch.

**Ask Claude:** "Give me a hint, not the answer: why does my FK disagree with MuJoCo on the hands but not the pelvis?"

### Day 10: NumPy + complexity (Jacobians and IK)

**Read:**

- *Modern Robotics*, chapters 5 (velocity kinematics and Jacobians) and 6.2 (numerical IK)
- Sam Buss, "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares Methods" (search the title; it's a free PDF)
- [MuJoCo: `mj_jacBody`](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-jacbody), to check your answer

**Waypoints:**

- [x] Compute the geometric Jacobian of a hand from your tree. Compare it with `mj_jacBody`.
- [ ] Implement damped least-squares IK for one hand: `dq = Jᵀ(JJᵀ + λ²I)⁻¹ e`.
- [ ] Clamp to joint limits after every step.
- [ ] Test convergence on 100 random reachable targets (generate them with FK from random `q`).
- [x] Time `np.linalg.inv` vs `np.linalg.solve` on the same system. Write down the Big-O of each step.

**Production path:**

- [ ] Production Jacobians come from MuJoCo on the scratch data: `mj_fwdPosition`, then `mj_jacBody` (and `mj_jacSubtreeCom` for the CoM). Keep yours as the reference in tests.
- [ ] IK stays yours, since MuJoCo has no IK solver, but runs on MuJoCo's Jacobians. Stretch goal: compare its results against mink on the same targets.

**If stuck:** reference to compare against after you've tried: [mink](https://github.com/kevinzakka/mink), a differential IK library built on MuJoCo.

**Ask Claude:** "Why does my IK oscillate near singularities? Explain the role of λ."

### Day 11: Extensible design + design patterns

**Read:**

- [Refactoring.Guru](https://refactoring.guru/design-patterns): Strategy, Factory Method, Observer and Template Method (about 10 minutes each)
- Look at how [pink](https://github.com/stephane-caron/pink) structures its `Task` classes. Read only the code, not the docs.

**Waypoints:**

- [ ] Define a `Task` ABC with `error(robot)`, `jacobian(robot)` and `weight`.
- [ ] Implement `ComTask`, `FrameTask` (hands and feet) and `PostureTask`.
- [ ] Write `make_task(config: dict) -> Task`, a factory that builds tasks from YAML.
- [ ] Add an Observer: `TelemetryHub.subscribe(callback)`. Controllers publish per-step data, and a CSV logger subscribes.
- [ ] Check the rule: adding a new task type shouldn't change any existing file except the registry.

**If stuck:** for the CoM Jacobian, MuJoCo has `mj_jacSubtreeCom`.

**Ask Claude:** "Is my factory hiding too much? Show me where it would be clearer without it."

### Day 12: Whole-body QP (hardest day)

**Read:**

- [qpsolvers docs](https://qpsolvers.github.io/qpsolvers/), the quick start
- [OSQP docs](https://osqp.org), the problem form and solver settings
- Look at [TSID](https://github.com/stack-of-tasks/tsid) for how a task-space inverse dynamics QP is structured (decision variables: joint accelerations, contact forces, torques)

**Waypoints:**

- [ ] On paper, write the QP: variables `x = [ddq, f_contact, tau]`, the dynamics equality `M ddq + h = Sᵀ tau + J_cᵀ f`, task costs `||J ddq + dJ dq − a_des||²`, and bounds.
- [ ] Build it with CoM and posture tasks only, plus the dynamics constraint. Get the G1 standing in hanging mode first.
- [ ] Add contact constraints for both feet (zero foot acceleration), then linearised friction cones (a 4-sided pyramid per contact point).
- [ ] Add torque limits from `model.actuator_ctrlrange` or the joint limits.
- [ ] Turn off hanging mode and apply a 50 N side push with `data.xfrc_applied` on the pelvis for 0.1 s.

**Production path:**

- [ ] Every physics quantity comes from MuJoCo: M from `mj_fullM`, h from `qfrc_bias`, contact Jacobians from `mj_jac*`. The QP is solved by OSQP through `qpsolvers`. You write the formulation, never the solver.
- [ ] Add a test that the solution satisfies the dynamics equality to 1e-6.

**If stuck:** get M from `mj_fullM` and h from `data.qfrc_bias`. If the solver says infeasible, remove constraints one at a time to find the conflicting one.

**Ask Claude:** "Here's my QP formulation. Check the dimensions and signs before I code it."

### Day 13: LIPM + dynamic programming (ZMP preview control)

**Read:**

- *Introduction to Humanoid Robotics* (Kajita et al.), chapter 4 on the linear inverted pendulum and preview control
- Kajita et al. (2003), "Biped walking pattern generation by using preview control of zero-moment point" (search the title)
- Any short LQR/Riccati derivation that shows it as dynamic programming (search "LQR dynamic programming derivation")

**Waypoints:**

- [ ] Write a classic DP warm-up first: memoised Fibonacci, then a grid shortest-path with DP. 20 minutes.
- [ ] Implement the discrete LIPM (state: CoM position, velocity, acceleration; output: ZMP).
- [ ] Solve the Riccati recursion for the preview gains. Plot the gains: they should decay to zero.
- [ ] Generate a ZMP reference from 6 footsteps.
- [ ] Simulate the LIPM alone and plot CoM vs ZMP. The ZMP should stay inside each footprint.

**Production path:**

- [ ] Compute the steady-state preview gain with `scipy.linalg.solve_discrete_are` and assert it matches your converged Riccati recursion. Production code uses SciPy's.

**If stuck:** run the Riccati recursion backwards until it converges, then use the steady-state gain. That's the same as value iteration in DP.

**Ask Claude:** "Explain how the Riccati recursion is the Bellman equation for a quadratic cost."

### Day 14: Property-based testing + refactoring + CLI

**Read:**

- [Hypothesis quick start](https://hypothesis.readthedocs.io/en/latest/quickstart.html)
- [Typer tutorial](https://typer.tiangolo.com/tutorial/), the first 3 pages
- [Refactoring.Guru: code smells](https://refactoring.guru/refactoring/smells)

**Waypoints:**

- [ ] Property test: for any random `q` within the limits, IK output stays within the limits.
- [ ] Property test: QP output satisfies the dynamics equality and friction cones to 1e-6.
- [ ] Property test: `SE3` `exp`/`log` round-trips for any random twist.
- [ ] Find 3 code smells in your QP code and refactor them, keeping tests green.
- [ ] Add the `humkit run balance.yaml` command. Tag `v0.2.0`.

**If stuck:** when Hypothesis finds a failure it shrinks it to the smallest case. Add that case as a normal regression test.

**Ask Claude:** "What other invariants should hold for a whole-body controller?"

## Week 3 guides (Days 15–21)

### Day 15: Profiling + memory hierarchy

**Read:**

- [py-spy README](https://github.com/benfred/py-spy) and the [pytest-benchmark docs](https://pytest-benchmark.readthedocs.io)
- Ulrich Drepper, "What Every Programmer Should Know About Memory" (search the title), sections 2–3 on caches
- *CS:APP* chapter 6 (memory hierarchy), if you have the book

**Waypoints:**

- [ ] Write benchmarks for one FK call, one IK solve and one full QP control step.
- [ ] Run `py-spy record -o flame.svg -- python -m humkit run balance.yaml` and open the flame graph.
- [ ] Write down the top 3 hotspots and estimate what fraction of each step they take.
- [ ] Fix one by vectorising (removing a Python loop). Re-measure.
- [ ] Demonstrate a cache effect: sum a large 2D array row-wise vs column-wise and explain the gap.

**Ask Claude:** "Here's my flame graph summary. Which hotspot is worth porting to C++ first?"

### Day 16: Modern C++ + OOP in C++

**Read:**

- [learncpp.com](https://www.learncpp.com): chapters on classes, virtual functions and move semantics (skim what you know)
- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/): sections C (classes) and R (resource management)
- [Eigen: getting started](https://eigen.tuxfamily.org/dox/GettingStarted.html)
- [MuJoCo C API: programming guide](https://mujoco.readthedocs.io/en/stable/programming/index.html)

**Waypoints:**

- [ ] Create `cpp/include/humkit/robot.hpp`: a `Robot` class that owns `mjModel*` and `mjData*` through `std::unique_ptr` with custom deleters (that's RAII).
- [ ] Delete the copy constructor, allow moves, and explain why in a comment (rule of 5).
- [ ] Port FK only (Jacobians come from MuJoCo in production) using `Eigen::Matrix`.
- [ ] Write an abstract `Controller` with a virtual `compute()`. Implement `PDController` and draw its vtable in your notes.
- [ ] Compare results with Python on 100 random configurations (dump them to a file from Python, then read it in C++).

**Production path:**

- [ ] The C++ port is for learning C++. The production C++ core calls the MuJoCo C API (`mj_kinematics`, `mj_jac*`) for kinematics, and only ports humkit's own logic (task errors, QP assembly) where Day 15's profile shows it matters.

**If stuck:** if you're unsure whether something should be `const`, make it `const` and let the compiler complain.

**Ask Claude:** "Review this class for ownership and `const`-correctness mistakes."

### Day 17: CMake, GoogleTest and sanitizers + compiling and linking

**Read:**

- [An Introduction to Modern CMake](https://cliutils.gitlab.io/modern-cmake/), the basics and "do's and don'ts"
- [GoogleTest primer](https://google.github.io/googletest/primer.html)
- *CS:APP* chapter 7 (linking): symbols, relocation, static vs shared libraries

**Waypoints:**

- [ ] Write `CMakeLists.txt` with a `humkit_core` library target and use `target_link_libraries` with `PUBLIC`/`PRIVATE` correctly.
- [ ] Add GoogleTest with `FetchContent` and port 5 Python tests to C++.
- [ ] Add a `CMakePresets.json` with `debug`, `release` and `asan` presets (`-fsanitize=address,undefined`).
- [ ] Add a deliberate out-of-bounds bug, watch ASan catch it, then remove it.
- [ ] Run `nm` on your library and find your symbols. Explain name mangling in 2 lines.

**Ask Claude:** "Why is my link failing with 'undefined reference'? \[paste error\]"

### Day 18: pybind11 + memory management

**Read:**

- [pybind11 docs](https://pybind11.readthedocs.io): first steps, classes, and the [Eigen](https://pybind11.readthedocs.io/en/stable/advanced/cast/eigen.html) page
- [scikit-build-core docs](https://scikit-build-core.readthedocs.io), getting started

**Waypoints:**

- [ ] Expose `Robot` and the C++ FK to Python as `humkit._core`.
- [ ] Switch the build backend to `scikit-build-core` so `uv pip install -e .` compiles the C++.
- [ ] Add `backend="cpp"` to the Python API and parametrize the test suite over both backends.
- [ ] Return Jacobians without copying (`Eigen::Ref` or a NumPy view) and prove it with a timing test.
- [ ] Record the speedup in `benchmarks/README.md`.

**Production path:**

- [ ] Bind whole operations (one full control step), not small ones like a single `SE3` multiply: each Python-to-C++ call has a fixed overhead.

**If stuck:** Python crashes with no traceback? Build in debug and run under `gdb python`. Ownership mistakes across the boundary are the usual cause.

**Ask Claude:** "Who owns this array, Python or C++? Walk me through the lifetime."

### Day 19: Real-time threads + operating systems

**Read:**

- [OSTEP](https://pages.cs.wisc.edu/~remzi/OSTEP/): chapters on concurrency intro, the thread API, and locks
- [rigtorp/SPSCQueue](https://github.com/rigtorp/SPSCQueue), as a reference lock-free single-producer, single-consumer queue
- Search "Linux real-time scheduling SCHED\_FIFO" and read one overview

**Waypoints:**

- [ ] Write a C++ control thread that runs at 1 kHz using `std::chrono::steady_clock` and `sleep_until`.
- [ ] Measure the loop timing and plot the jitter histogram. Write down the p50 and p99.
- [ ] Write your own SPSC ring buffer with `std::atomic` (start from the reference design) and use it to send telemetry to a logging thread.
- [ ] Build with `-fsanitize=thread` and fix whatever it finds.
- [ ] Try `SCHED_FIFO` (needs root or the right capability) and compare jitter. Stretch goal: skip it if you're short on time.

**Production path:**

- [ ] Once your ring buffer passes ThreadSanitizer, switch the control loop to a proven queue such as [rigtorp/SPSCQueue](https://github.com/rigtorp/SPSCQueue), run the same stress test, and keep yours only as a learning artefact.

**If stuck:** never take a lock or allocate memory inside the control loop. That's the main cause of jitter.

**Ask Claude:** "Is my ring buffer's memory ordering correct? \[paste code\]"

### Day 20: Walking + finite state machines

**Read:**

- [Game Programming Patterns: State](https://gameprogrammingpatterns.com/state.html)
- Re-read your Day 13 notes on preview control

**Waypoints:**

- [ ] Draw the walking FSM on paper: states, transitions and the guard for each transition (time elapsed, foot contact).
- [ ] Implement it with the State pattern, with each state setting the QP tasks (which foot is in contact, swing-foot target).
- [ ] Generate swing-foot trajectories (a simple polynomial in height).
- [ ] Get stepping in place working first. Then 1 step forward, then 10.
- [ ] Record a video of the walk.

**If stuck:** if the G1 falls at touchdown, slow the walk down (longer step time, shorter steps) and raise the CoM task weight. Tuning is part of the job.

**Ask Claude:** "The G1 falls after step 3. Here's my CoM vs ZMP plot. What's the likely cause?"

### Day 21: Architecture docs + coupling

**Read:**

- [The C4 model](https://c4model.com), the context and container diagrams
- [ADR GitHub organisation](https://adr.github.io), for ADR templates

**Waypoints:**

- [ ] Draw a C4 container diagram and a component diagram of humkit.
- [ ] Write 3 ADRs: why a QP, why pybind11, and why the core has no ROS dependency.
- [ ] Find one place where modules are too tightly coupled and fix it with dependency inversion.
- [ ] Tag `v0.3.0`.

**Ask Claude:** "Read my architecture doc as a new team member. What's unclear?"

## Week 4 guides (Days 22–30)

### Day 22: CI + reproducibility

**Read:** [GitHub Actions docs](https://docs.github.com/en/actions): the quickstart and "Building and testing Python". [MuJoCo Python docs](https://mujoco.readthedocs.io/en/stable/python.html) on headless rendering (`MUJOCO_GL=egl` or `osmesa`).

**Waypoints:**

- [ ] Write `.github/workflows/ci.yml` with jobs for lint, mypy, pytest with coverage, and the C++ build and tests. Use a Python 3.10–3.12 matrix.
- [ ] Add a headless test: balance for 5 s with a fixed seed and assert the pelvis height stays above a threshold.
- [ ] Make the checks required in branch protection. Add the CI and coverage badges to the README.

**Ask Claude:** "My CI is slow. Here's the workflow; where should I add caching?"

### Day 23: Telemetry + networking

**Read:** [Beej's Guide to Network Programming](https://beej.us/guide/bgnet/), the chapters on sockets and UDP. The [msgpack](https://msgpack.org) overview.

**Waypoints:**

- [ ] Write a UDP sender that sends joint state and CoM at 100 Hz, msgpack-encoded with a sequence number.
- [ ] Write a receiver that plots live (matplotlib animation, or pyqtgraph if you want it smooth).
- [ ] Count dropped packets from the sequence numbers. Explain in 3 lines why UDP suits telemetry and TCP suits commands.

**Ask Claude:** "How should I version my telemetry message format so old clients don't break?"

### Day 24: Packaging + Docker

**Read:** [cibuildwheel docs](https://cibuildwheel.pypa.io). [Docker: multi-stage builds](https://docs.docker.com/build/building/multi-stage/).

**Waypoints:**

- [ ] Build manylinux wheels in CI on every tag and publish them to TestPyPI.
- [ ] Write a multi-stage `Dockerfile`: a build stage that compiles the C++ and a slim runtime stage that renders headless.
- [ ] `docker run humkit run walk.yaml --video out.mp4` produces a video.

### Day 25: ROS2 bridge + publish–subscribe

**Read:** [ROS2 tutorials](https://docs.ros.org/en/jazzy/Tutorials.html): writing a publisher and subscriber (Python), plus the page on QoS settings. Use your installed ROS2 version if it differs.

**Waypoints:**

- [ ] Create a separate `humkit_ros` package. The core stays ROS-free; the node only adapts.
- [ ] Publish `/joint_states` and the pelvis TF; run `robot_state_publisher` with the G1 URDF so RViz shows the robot.
- [ ] Subscribe to `/cmd_vel` and turn it into footstep plans. Steer the G1 with `teleop_twist_keyboard`.
- [ ] Use best-effort QoS for state and reliable QoS for commands, and explain why in the README.

### Day 26: Docs site

**Read:** [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) getting started. [mkdocstrings](https://mkdocstrings.github.io). [Diátaxis](https://diataxis.fr) (tutorials, how-to guides, reference, explanation).

**Waypoints:**

- [ ] Set up mkdocs with a tutorial ("make the G1 balance"), 3 how-tos, an auto-generated API reference and the architecture page.
- [ ] Deploy to GitHub Pages from CI.
- [ ] Have someone (or Claude) follow the quickstart from scratch and fix wherever they get stuck.

### Day 27: Config validation + databases

**Read:** [pydantic docs](https://docs.pydantic.dev), models and validation. [Python `sqlite3` docs](https://docs.python.org/3/library/sqlite3.html). [SQLite: CREATE INDEX](https://www.sqlite.org/lang_createindex.html).

**Waypoints:**

- [ ] Replace raw YAML dicts with pydantic models, so a typo in a config gives a clear error.
- [ ] Design a schema with `runs` (config, git commit, date) and `metrics` (run id, name, value). Write it out before coding.
- [ ] Log every run, add an index on metric name, and write `humkit report` to compare runs.
- [ ] Write the one SQL query that returns the best push-recovery gains.

### Day 28: Open source + reading large codebases

**Read:** [git bisect docs](https://git-scm.com/docs/git-bisect). The `CONTRIBUTING.md` of [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) or [mink](https://github.com/kevinzakka/mink).

**Waypoints:**

- [ ] Write `CONTRIBUTING.md` and issue templates for humkit. Open 3 "good first issue" tickets.
- [ ] Pick a library you used this month, trace one function from its public API to where the work happens, and write notes.
- [ ] Leave one useful review comment, issue or small PR on that project.

### Days 29–30: Release and demo

**Read:** [Keep a Changelog](https://keepachangelog.com) and [Semantic Versioning](https://semver.org).

**Waypoints:**

- [ ] Day 29: clear the top 3 backlog items, freeze the API, update the README (GIF, benchmarks, jitter numbers) and publish the v1.0.0 GitHub Release.
- [ ] Day 30: record the 3-minute video using the shot list on the main tab, write the post, and update your portfolio.
- [ ] Day 30: write `notes/retro.md`.

**Ask Claude:** "Here's my README. Rewrite the first screen so a robotics hiring manager gets it in 10 seconds."
