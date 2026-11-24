# AGENTS.md

Notes for agents that work on chrono_drive. `README.md` holds the user-facing build steps. This file holds the context, the decisions, and the traps.

## Project

chrono_drive is a physics-first driving simulator built on Project Chrono. The goal is a base for a game: a drivable vehicle on deformable terrain. The current state is an HMMWV on a heightmap SCM terrain patch, with keyboard driving and a chase camera.

## Layout

- `flake.nix`: the dev shells. `default` holds the toolchain, VSG, vsgXchange, and vsgImGui. `cuda` adds CUDA 12.9 for GPU modules.
- `chrono/`: a submodule on branch `chrono_drive` of `skylarmb/chrono` (a fork). Our Chrono patches live on that branch.
- `build-chrono/`: the Chrono build that chrono_drive links against (Vehicle + VSG, no demos). This directory is gitignored.
- `build/`: the chrono_drive build. This directory is gitignored.
- `build-crm/`: an experimental CUDA build of Chrono (FSI/SPH, CRM demos). It is not linked by chrono_drive.
- `src/main.cpp`: the application. `data/maps/`: grayscale heightmaps.

## Build and run

```sh
nix develop -c ninja -C build-chrono   # only after changes in chrono/
nix develop -c ninja -C build
nix develop -c ./build/chrono_drive
```

`README.md` has the first-time `cmake` configure commands.

## Workflow rules

- Commit locally after each working step. Push only at milestones that are verified (QA'd). The user confirms a milestone.
- Patch Chrono only when you cannot extend it from `src/` (for example, with a subclass). Commit patches in `chrono/` on branch `chrono_drive`. Never commit to the `main` branch of the fork.
- After a patch, commit the new submodule pointer in this repo.
- Commit trailer: `✨ Created by Claude Code`, then the `Co-Authored-By` line.
- Ask the user before design decisions. Fix bugs that are clear without asking, then report them.

## Traps (all confirmed)

- **Flakes see only tracked files.** Run `git add` on a new or changed file before `nix develop`, or Nix does not see it.
- **The vehicle does not step a system it does not own.** `main.cpp` creates `ChSystemSMC` and gives it to `HMMWV_Full`. Then `ChVehicle::Advance()` does not call `DoStepDynamics()` (see `ChVehicle.cpp`, `m_ownsSystem`). The loop must call `sys.DoStepDynamics()` itself.
- **Do not use `ChRealtimeStepTimer::Spin()`.** It pads each step to the step size and drops the extra time from render frames. The loop then runs slower than real time on any hardware. `main.cpp` paces on accumulated sim time against the wall clock.
- **`CHRONO_VEHICLE_DATA_DIR` from the Chrono target is wrong for us.** It points to `build-chrono/bin/data/vehicle/`, which does not exist. Use `CHRONO_DATA_DIR "vehicle/"`.
- **SCM timers are not per-step seconds.** `SCMTerrain::GetTimer*()` returns milliseconds, and the reset happens in `SCMLoader::ComputeInternalForces()`. Do not sum the values per step. Use `perf` or wall-clock timers.
- **CUDA binaries load a stub `libcuda`.** The `cuda` shell puts `/run/opengl-driver/lib` on `LD_LIBRARY_PATH`. Without it, CUDA code fails with `cudaErrorStubLibrary`.
- **nvcc 12.9 does not accept GCC 16.** The `cuda` shell uses `cudaPackages_12.backendStdenv` (GCC 14).
- **VSG uses X11 (xcb).** On Wayland, it runs through XWayland. It renders on the NVIDIA GPU (confirmed with `nvidia-smi pmon`).

## Performance facts (2026-11-23, RTX 4090 laptop, 22 cores)

- **Rendering limits frame time.** Physics does not. Physics runs at 4x to 9x real time up to a 0.025 m grid.
- **SCM physics cost does not change with map size.** Active domains limit ray casts to the area around the tires.
- **More threads do not help.** One thread is as fast as eight, and sixteen is slower.
- **The cause is in `ChVisualSystemVSG::Update()`** (`chrono_vsg/ChVisualSystemVSG.cpp`, near line 1150). Each frame, it converts and marks dirty the full vertex, normal, and color arrays of every mutable mesh. The arrays use the mesh-soup layout (three vertices per triangle). `GetFaceVertices()` also builds a new full array each call.
- **SCM already reports changed vertices.** It calls `SetModifiedVertices()` on its shape (`SCMTerrain.cpp`, near line 2075). The old Irrlicht layer used that list. The VSG layer ignores it.
- **A coarse grid changes the physics.** At 0.4 m, the HMMWV reaches 1.4 m/s at full throttle. At 0.1 m, it reaches 8.5 m/s. Do not use a coarse grid as a performance fix.
- **CRM (SPH sand) is GPU-bound.** It is about 10x slower than real time on a 20 m x 3 m strip. It is not a candidate for real-time play.

## Benchmark mode (temporary)

`main.cpp` reads environment overrides: `BENCH=<sim seconds>` (full throttle, prints a timing line, then exits), `GRID`, `MAP`, `THR`, `RENDER=0`, `PACE=1`. Example:

```sh
nix develop -c sh -c 'BENCH=10 GRID=0.05 ./build/chrono_drive'
```

Remove this code after the SCM render work is complete.
