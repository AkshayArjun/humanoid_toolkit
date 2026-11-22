# Changelog

All notable changes to this project are documented here.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project uses [Semantic Versioning](https://semver.org/).

## [Unreleased]

## [0.1.0] - 2026-11-22

### Added
- `Robot` class wrapping MuJoCo, with validated `set_state`/`set_ctrl`, hanging mode and torque control.
- Controllers: `ControllerABC`, `PDController`, `GravityCompController`, `CompositeController`.
- YAML config runner (`python -m humkit.run <config>`).
- Error hierarchy (`HumkitError` and subclasses) and module-level logging.
- `humkit.geometry`: `SO3` and `SE3` following Solà et al., with Exp/Log, exp/log, plus/minus and Adj.
- Test suite with fakes and mocks, plus checks against MuJoCo body poses.
