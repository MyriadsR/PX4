# Repository Guidelines

## Project Structure & Module Organization
- Core source: `PX4-Autopilot/src` (modules, drivers, libraries). Boards in `PX4-Autopilot/boards`.
- Scripts and utilities: `PX4-Autopilot/Tools`.
- Firmware assets/ROMFS: `PX4-Autopilot/ROMFS`.
- Tests: unit/integration in `PX4-Autopilot/test` and `PX4-Autopilot/integrationtests`.
- Documentation: `PX4-Autopilot/docs`. Build outputs live under `PX4-Autopilot/build/<config>`.

## Build, Test, and Development Commands
- Build SITL (default): `make -C PX4-Autopilot px4_sitl_default`.
- Run SITL + Gazebo Classic: `make -C PX4-Autopilot px4_sitl_default sitl_gazebo-classic`.
- Hardware target example: `make -C PX4-Autopilot px4_fmu-v5_default`.
- Quick CI-like sweep: `make -C PX4-Autopilot quick_check` (builds, tests, style check).
- Unit/integration tests: `make -C PX4-Autopilot tests` (filter with `TESTFILTER=<pattern>`).
- Static analysis: `make -C PX4-Autopilot clang-tidy` or `scan-build`.

## Coding Style & Naming Conventions
- Languages: C/C++ (headers typically `.hpp`).
- Formatting: run `make -C PX4-Autopilot format` (astyle) and `make ... check_format` before committing.
- Prefer descriptive module prefixes in files and symbols (e.g., `commander_*, mavlink_*`).
- Follow existing patterns in `src/modules/*` and `src/drivers/*` for class and file naming.

## Testing Guidelines
- Run all tests locally: `make -C PX4-Autopilot tests`.
- Coverage: `make -C PX4-Autopilot tests_coverage` (outputs `coverage/lcov.info`).
- ROS/Gazebo integration tests: `make -C PX4-Autopilot rostest` then use provided test runners in `test/`.
- Place new tests alongside related modules (mirror `src/*` structure). Name clearly (e.g., `test_<module>_*.cpp`).

## Commit & Pull Request Guidelines
- Commit messages: use imperative mood and scope prefix where helpful.
  - Example: `commander: fix failsafe state transition on link loss`.
- PRs should include: concise description, linked issues, testing evidence (logs/flight data), and screenshots if UI/tools are affected.
- Keep changes focused; update docs in `docs/` and parameters/metadata when interfaces change.

## Security & Configuration Tips
- Clone with submodules: `git submodule update --init --recursive`.
- Ninja is supported and preferred when available; Python is required for many tools (set `PYTHON_EXECUTABLE` if needed).

// Agent-specific Guidelines

