# 30-Day Coding Bootcamp: Build a Humanoid Control Stack in MuJoCo

Nov 15, 2026 · @ak

## The goal

On Day 30 you release **humkit v1.0.0**, an open-source whole-body control stack for the Unitree G1 humanoid (from MuJoCo Menagerie) running in MuJoCo. The demo video shows the G1 balancing, recovering from a push, reaching for targets and walking 10+ steps.

Each day pairs a **software-engineering practice** with a **core CS or OOP concept**, and you apply both in the same repo:

- **Week 1 (Foundations + OOP):** a clean, tested Python package that wraps MuJoCo in well-designed classes and holds the G1 standing with PD and gravity compensation. Tag v0.1.
- **Week 2 (Design + DSA):** your own kinematic tree, forward kinematics, Jacobians, IK, a whole-body QP controller and ZMP preview control. Tag v0.2.
- **Week 3 (Systems):** a C++ core with pybind11, a real-time 1 kHz control thread and a walking state machine. Tag v0.3.
- **Week 4 (Shipping + networking):** CI, wheels, Docker, live telemetry over UDP, a ROS2 bridge and a docs site. Tag v1.0.0 on Day 29.

What you'll be able to show at the end: a GitHub repo with green CI and a docs site, a `pip install`-able package, a Python vs C++ benchmark table, control-loop jitter numbers, and the demo video. It builds directly on your whole-body control and Lie theory background.

## How each day works

Each day takes about 3.5 hours: 1 hour learning, 2 hours building, 15 minutes reviewing. Every day ends with a merged pull request.

1. **Learn (1 h):** spend 30 minutes on the day's practice and 30 minutes on its core CS or OOP concept. Write 5 bullet notes in `notes/dayNN.md` in your own words.
2. **Build (2 h):** do the task on a feature branch. Get it working first, then make it clean.
3. **Review (15 min):** open a PR against `main` and review it yourself with the checklist below before you merge. Ask Claude to review the diff as a strict senior engineer would.

**PR self-review checklist** (use it every day):

- [ ] Tests added or updated, and all passing
- [ ] No function longer than \~40 lines and no deep nesting
- [ ] Names say what things are, with no `tmp`, `data2` or `foo`
- [ ] No dead code, commented-out blocks or stray prints
- [ ] Commit messages follow Conventional Commits (`feat:`, `fix:`, `refactor:`)
- [ ] README or docstrings updated if behaviour changed

**Rules:** use hints before solutions (your usual style). If a task runs over 4 hours, cut scope and write down what you skipped in `BACKLOG.md`. A smaller merged PR beats an unfinished big one.

**Guided mode:** every day has exact resources to read, numbered waypoints to hit, and "if stuck" hints in Daily guides. Follow it closely in Weeks 1–2, then lean on it less as you go.

## Learn it, then ship it

From Week 2 on, each algorithm is built twice: once by hand to learn it, and once on a proven library for the code humkit actually runs. Your own version stays as the test oracle's partner: both must agree.

1. **Build it to learn it:** write the algorithm yourself, small and readable.
2. **Validate it:** check it against the library to a tight tolerance (usually 1e-9).
3. **Ship the library path:** humkit's production code calls the proven implementation. Your version moves to a reference module used only by tests.
4. **Never ship your own version of** a physics engine, QP solver, Riccati solver or lock-free queue. Write your own only where no library fits (task definitions, controller logic, walking FSM).

| Day | Build to learn | Production path | Validate with |
| --- | --- | --- | --- |
| 7 | `SO3`/`SE3` from Solà | Keep yours (small, fully tested); manif's Python bindings are the drop-in alternative | MuJoCo body poses, SciPy `expm` |
| 9 | Kinematic tree and recursive FK | `Robot.forward_kinematics`: scratch `MjData` + `mj_kinematics` | Your FK vs MuJoCo to 1e-9 |
| 10 | Geometric Jacobians, damped least-squares IK | Jacobians from `mj_jacBody` / `mj_jacSubtreeCom`; IK is yours (MuJoCo has none) on MuJoCo Jacobians | Your Jacobian vs `mj_jacBody`; your IK vs mink |
| 12 | Write the QP formulation yourself | M from `mj_fullM`, h from `qfrc_bias`, contact Jacobians from MuJoCo; OSQP via `qpsolvers` | Dynamics residual of the solution ≈ 0 |
| 13 | Riccati recursion by hand | `scipy.linalg.solve_discrete_are` for the steady-state gain | Your gain vs SciPy's |
| 16–18 | Port FK and Jacobians to C++ | C++ core calls the MuJoCo C API + Eigen; only humkit's own logic (tasks, QP assembly) is ported, and only where Day 15 profiling says it matters | C++ vs Python on 100 random states |
| 19 | Your own SPSC ring buffer | A proven queue (e.g. rigtorp/SPSCQueue) | Same stress test under ThreadSanitizer |

**Time budget:** the production steps add 15–30 minutes on the days above. To keep every day at about 3.5 hours, four things were trimmed: Day 9 reads the tree from MjModel instead of parsing the XML (XML parsing is a stretch goal), Day 16 ports FK only, and the mink comparison (Day 10) and `SCHED_FIFO` experiment (Day 19) are stretch goals. Day 12 and Day 18 cost nothing extra: their production paths are how you'd build them anyway.

## Week 1: Foundations + OOP (Days 1–7)

By Day 7 the G1 stands still in MuJoCo under your own controllers, behind clean, tested classes. Difficulty: easy to medium.

| Day | Practice | Core CS / OOP concept | What you build | Done when |
| --- | --- | --- | --- | --- |
| 1 | Git workflow: branches, rebase vs merge, Conventional Commits, PRs | How git works inside: content-addressed storage, SHA hashes, the commit DAG | `humkit` repo with protected `main`; load the G1 from Menagerie and open the MuJoCo viewer | `python -m humkit.view` shows the G1 |
| 2 | Project layout: `src/` layout, `pyproject.toml`, `uv`, ruff, pre-commit | How Python runs code: interpreter, bytecode, the import system, modules vs packages | Scaffold the package and wire up the hooks | `pip install -e .` works; hooks block a bad commit |
| 3 | Clean code: naming, small functions, single responsibility | **OOP I:** classes, encapsulation, class invariants, properties, `__repr__` and `__eq__` | `Robot` class wrapping `MjModel`/`MjData` with joint names, limits and state get/set | No raw `MjData` access outside `Robot` |
| 4 | Configuration over code | **OOP II:** inheritance vs composition, abstract base classes, polymorphism, Liskov substitution | `Controller` ABC with `PDController` and `GravityCompController`; G1 holds a standing pose | Controllers swap through config alone |
| 5 | Testing with pytest, then test-driven development | Testing theory: unit vs integration tests, test doubles (mocks, fakes, stubs) | Tests for `Robot` and the controllers, using a fake model for fast unit tests | Coverage ≥ 85% |
| 6 | Error handling and logging: exception hierarchies, fail fast | **Floating point:** IEEE 754, machine epsilon, catastrophic cancellation, tolerance comparisons | `HumkitError` hierarchy, input validation, logging, and quaternion normalisation guards | No `==` on floats anywhere; no `print` calls |
| 7 | Code review: what reviewers look for | **OOP III:** operator overloading, immutability, value objects | `se3` module: `SO3`/`SE3` value classes with `@` composition and `exp`/`log`; tag **v0.1.0** | `exp(log(T)) ≈ T` for 1,000 random poses |

## Week 2: Design + data structures and algorithms (Days 8–14)

By Day 14 the G1 balances under pushes with a whole-body QP controller, and preview control plans walking CoM trajectories. Difficulty: medium to hard. Day 12 is the hardest day of the bootcamp.

| Day | Practice | Core CS / OOP concept | What you build | Done when |
| --- | --- | --- | --- | --- |
| 8 | Type hints, `mypy --strict`, dataclasses, `Protocol` | Type systems: static vs dynamic typing, nominal vs structural (duck) typing, generics | Fully type the package | `mypy --strict` passes in pre-commit |
| 9 | Parsing external formats safely | **Trees and graph traversal:** DFS, BFS, recursion vs an explicit stack, topological order | Build your own kinematic tree from MjModel; recursive forward kinematics | FK matches MuJoCo to 1e-9 on all G1 bodies |
| 10 | Numerical code with NumPy | **Complexity:** Big-O of matrix ops, vectorisation, conditioning, why O(n³) matters | Geometric Jacobians plus damped least-squares IK for hands and feet | IK converges on 100 random reachable targets |
| 11 | Extensible design | **Design patterns:** Strategy, Factory, Observer, Template Method | `Task` classes (CoM, foot, hand, posture) built by a factory from YAML; Observer hooks for telemetry | Adding a new task touches zero existing files |
| 12 | Isolating hard logic behind an interface | **Optimisation as an algorithm:** QPs, active-set vs ADMM, sparsity | Whole-body QP controller (tasks, friction cones, torque limits) using `qpsolvers` with OSQP | G1 stays upright after a 50 N side push |
| 13 | Separating models from controllers | **Dynamic programming:** Bellman's principle, memoisation, Riccati recursion as DP | Linear inverted pendulum model, ZMP preview control (Kajita) and a footstep-to-ZMP generator | CoM tracks a 6-step ZMP plan in the LIPM simulation |
| 14 | Property-based testing (Hypothesis), refactoring, a CLI with `typer` | Invariants and pre/post-conditions (design by contract) | Property tests (IK respects joint limits, QP output satisfies constraints); `humkit run balance.yaml`; tag **v0.2.0** | Hypothesis runs 500 cases without a failure |

**Tip for Day 12:** if the QP isn't done in 4 hours, merge a CoM-only version and add the friction cones on Day 13. Note it in `BACKLOG.md`.

## Week 3: Performance + systems (Days 15–21)

By Day 21 the controller runs in C++ on a real-time thread and the G1 walks. Difficulty: hard.

| Day | Practice | Core CS / OOP concept | What you build | Done when |
| --- | --- | --- | --- | --- |
| 15 | Profiling: `cProfile`, `py-spy` flame graphs, `pytest-benchmark` | **Computer architecture:** memory hierarchy, cache lines, locality, SIMD (why vectorised NumPy is fast) | Benchmark the control step; profile and fix the top 3 hotspots | `benchmarks/README.md` has a baseline and a flame graph |
| 16 | Modern C++17/20 with Eigen | **OOP in C++:** RAII, rule of 0/3/5, move semantics, virtual functions and vtables, smart pointers, `const`-correctness | Port `Robot` and FK to C++ on the MuJoCo C API | Results match Python on 100 random configurations |
| 17 | CMake targets, GoogleTest, sanitizers (ASan, UBSan), clang-tidy | **Compilers and linkers:** preprocessing, compilation units, static vs shared libraries, symbols, ABI | CMake project with unit tests and a sanitizer preset | Tests pass with no sanitizer reports |
| 18 | Bindings with pybind11 and `scikit-build-core` | **Memory management:** stack vs heap, ownership, reference counting vs RAII, zero-copy Eigen ↔ NumPy | `backend="cpp"` behind the same Python API; run the test suite on both backends | Both green; speedup recorded (aim for 10× or more) |
| 19 | Concurrency done safely; ThreadSanitizer | **Operating systems:** processes vs threads, scheduling, priorities, real-time jitter, mutexes vs lock-free queues, deadlock | 1 kHz control thread plus a logging/viz thread joined by a lock-free single-producer, single-consumer ring buffer | p99 loop jitter under 100 µs; TSan is clean |
| 20 | Behaviour as explicit states | **Finite state machines:** states, transitions, guards; the State pattern | Walking FSM (stand → weight shift → single support → double support) driving preview control and the QP | G1 walks 10 steps in MuJoCo without falling |
| 21 | Architecture docs: C4 diagrams, ADRs | **Layered architecture and coupling:** cohesion, dependency inversion | `docs/architecture.md` plus 3 ADRs (why a QP, why pybind11, why a ROS-free core); tag **v0.3.0** | A newcomer could explain the design from the docs alone |

**Tip for Day 20:** walking is the riskiest milestone. If it falls, ship a stable "step in place" first and keep tuning on Day 21.

## Week 4: Shipping + networking and data (Days 22–28)

By Day 28 anyone can install, run, monitor and extend humkit, and every change is checked automatically. Difficulty: medium, but wide in scope.

| Day | Practice | Core CS / OOP concept | What you build | Done when |
| --- | --- | --- | --- | --- |
| 22 | CI with GitHub Actions: matrices, caching, required checks | Regression testing and determinism: seeds, reproducible simulation | CI runs lint, mypy, pytest, the C++ tests and sanitizers, plus a headless MuJoCo balance test | PRs can't merge unless CI is green |
| 23 | Wire protocols and live tooling | **Networking:** TCP vs UDP, sockets, serialisation (msgpack or protobuf), latency vs throughput | UDP telemetry streamer plus a live joint and CoM plotting client | Live plots at 100 Hz with under 1% packet loss on localhost |
| 24 | Packaging (semver, wheels, `cibuildwheel`) and Docker | Virtualisation vs containers: namespaces, cgroups, image layers | Wheels on TestPyPI; a multi-stage headless Docker image | `pip install` works in a clean venv; `docker run` produces a video |
| 25 | Hexagonal architecture (ports and adapters) | **Distributed systems:** publish–subscribe, DDS, QoS (reliable vs best-effort) | `humkit_ros` ROS2 package: publish `/joint_states` and TF, subscribe to `/cmd_vel` to steer walking | G1 shown in RViz, steered from the keyboard |
| 26 | Documentation: docstrings, mkdocs-material plus mkdocstrings | Writing for readers: tutorials, how-tos, reference and explanation (Divio) | Docs site on GitHub Pages | Quickstart goes from zero to a walking video in under 10 minutes |
| 27 | Config validation (pydantic) and observability | **Databases:** relational schemas, SQL, indexes, transactions (SQLite) | Log every run's config and metrics to SQLite; `humkit report` compares runs | Query "best push-recovery gains" in one SQL line |
| 28 | Open-source etiquette: issues, `CONTRIBUTING.md`, reviewing others' code | Reading large codebases: tracing call graphs, git blame/bisect | Issue templates and 3 "good first issue" tickets; one review or PR to MuJoCo Menagerie or another library you use | One public contribution outside your repo |

## Days 29–30: Release and demo

Day 29 ships v1.0.0. Day 30 makes sure people actually see it.

**Day 29: Release v1.0.0**

- Concept: release engineering, which covers changelogs, API stability promises and deprecation.
- Build: fix the top 3 items in `BACKLOG.md`, freeze the public API, put the benchmark and jitter tables at the top of the README, and publish **v1.0.0** as a GitHub Release.
- Done when: a stranger can `pip install` it, run the quickstart and watch the G1 walk.

**Day 30: Demo and retrospective**

- Concept: technical communication. Lead with the result, then show the evidence.
- Build: record a 3-minute video (stand → push recovery → reaching → walking → RViz → benchmarks). Write a short post and add humkit to your portfolio site.
- Retro: write `notes/retro.md` covering what got easier, what you'll keep using at work, and what's next (e.g. an RL locomotion policy in MuJoCo MJX that uses your QP as a safety filter).

## Core CS map

The bootcamp covers the main core CS subjects. Each one is applied to the humanoid, not studied in isolation.

| Area | Concepts | Days |
| --- | --- | --- |
| Object-oriented programming | Encapsulation, invariants, inheritance vs composition, ABCs, polymorphism, Liskov, operator overloading, value objects, vtables, RAII, rule of 0/3/5 | 3, 4, 7, 16 |
| Design patterns and architecture | Strategy, Factory, Observer, Template Method, State, layered and hexagonal architecture, dependency inversion | 11, 20, 21, 25 |
| Data structures and algorithms | Trees, DFS/BFS, recursion, topological order, Big-O, dynamic programming, ring buffers, FSMs | 9, 10, 13, 19, 20 |
| Numerical computing | IEEE 754, conditioning, QPs and their solvers, sparsity | 6, 10, 12 |
| Computer architecture | Memory hierarchy, caches, locality, SIMD | 15 |
| Compilers and languages | Bytecode, imports, compilation and linking, ABI, type systems | 2, 8, 17 |
| Operating systems | Processes, threads, scheduling, real-time jitter, locks, lock-free queues, deadlock, memory management, containers | 18, 19, 24 |
| Networking and distributed systems | TCP vs UDP, sockets, serialisation, pub–sub, DDS QoS | 23, 25 |
| Databases | Schemas, SQL, indexes, transactions | 27 |
| Software engineering | Git internals, testing theory, TDD, property-based testing, design by contract, CI, code review | 1, 5, 14, 22, 28 |

**Daily DSA habit (optional, 20 min):** one hint-first LeetCode problem in the matching topic, e.g. trees on Day 9, DP on Day 13 and graphs on Day 20.

## Final deliverable checklist

Tick these off on Day 30. If all are ticked, the bootcamp is done.

- [ ] Public GitHub repo with protected `main`, 30+ merged PRs and Conventional Commits
- [ ] CI green: lint, `mypy --strict`, pytest (≥ 85% coverage), C++ tests, sanitizers and a headless balance test
- [ ] Own kinematic tree, FK, Jacobians, IK and `se3` module, all tested against MuJoCo
- [ ] Whole-body QP controller that recovers from a 50 N push
- [ ] ZMP preview control plus a walking FSM: G1 walks 10+ steps
- [ ] C++ core with pybind11, a measured speedup, and p99 jitter under 100 µs at 1 kHz
- [ ] UDP telemetry, a ROS2 bridge with RViz, and a SQLite run log
- [ ] Wheels on TestPyPI, a Docker image, and a docs site with an architecture doc and 3 ADRs
- [ ] v1.0.0 release, a demo video and a write-up

## Resources

- **MuJoCo:** [MuJoCo docs](https://mujoco.readthedocs.io); [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) for the G1 model
- **Humanoid control:** *Introduction to Humanoid Robotics* (Kajita et al.) for LIPM and ZMP preview control; *Modern Robotics* (Lynch and Park) for kinematics and Jacobians
- **OOP and design:** *A Philosophy of Software Design* (Ousterhout); [Refactoring.Guru design patterns](https://refactoring.guru/design-patterns); *Refactoring*, 2nd ed. (Fowler)
- **Core CS:** [OSTEP](https://pages.cs.wisc.edu/~remzi/OSTEP/) for operating systems; *Computer Systems: A Programmer's Perspective* (Bryant and O'Hallaron) for architecture, linking and memory; [Beej's Guide to Network Programming](https://beej.us/guide/bgnet/)
- **C++:** [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/); *Effective Modern C++* (Meyers); [pybind11 docs](https://pybind11.readthedocs.io)
- **Testing:** [pytest docs](https://docs.pytest.org); [Hypothesis docs](https://hypothesis.readthedocs.io)
