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
- `data/vehicle/hmmwv/`: our HMMWV spec (JSON), loaded with Chrono's generic `WheeledVehicle`. `SetVehicleDataPath` points here. The `.obj` and `.mtl` files are symlinks into `chrono/data`. The chassis mesh path is relative to the Chrono data path, so it needs no link.
- `src/main.cpp`: the application. `data/maps/`: grayscale heightmaps. `tools/gen_heightmap.py` writes `test_track.bmp` (a flat strip along X with hills on both sides).

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
- **The vehicle does not step a system it does not own.** `main.cpp` creates `ChSystemSMC` and gives it to the vehicle. Then `ChVehicle::Advance()` does not call `DoStepDynamics()` (see `ChVehicle.cpp`, `m_ownsSystem`). The loop must call `sys.DoStepDynamics()` itself.
- **A system we own gets the wrong solver.** `ChVehicle` sets Barzilai-Borwein with 150 iterations only on a system it creates. Our system needs the same lines (`main.cpp`). With the default PSOR, the steering and suspension joints stretch: the front wheels wobble, fold in reverse, and the truck drifts about 4 m over 50 m with zero steering.
- **The default SMC contact material is soft.** `ChContactMaterialSMC` defaults to E = 2e5 Pa. Tires sink 17 cm into a bump made with it. Rigid objects use E = 2e7 Pa, friction 0.9, restitution 0.01 (as in Chrono's vehicle demos). Then the tires sink 4 cm.
- **Do not use `ChRealtimeStepTimer::Spin()`.** It pads each step to the step size and drops the extra time from render frames. The loop then runs slower than real time on any hardware. `main.cpp` paces on accumulated sim time against the wall clock.
- **`CHRONO_VEHICLE_DATA_DIR` from the Chrono target is wrong for us.** It points to `build-chrono/bin/data/vehicle/`, which does not exist. Use `CHRONO_DATA_DIR "vehicle/"`.
- **SCM timers are not per-step seconds.** `SCMTerrain::GetTimer*()` returns milliseconds, and the reset happens in `SCMLoader::ComputeInternalForces()`. Do not sum the values per step. Use `perf` or wall-clock timers.
- **CUDA binaries load a stub `libcuda`.** The `cuda` shell puts `/run/opengl-driver/lib` on `LD_LIBRARY_PATH`. Without it, CUDA code fails with `cudaErrorStubLibrary`.
- **nvcc 12.9 does not accept GCC 16.** The `cuda` shell uses `cudaPackages_12.backendStdenv` (GCC 14).
- **VSG uses X11 (xcb).** On Wayland, it runs through XWayland. It renders on the NVIDIA GPU (confirmed with `nvidia-smi pmon`).

## Suspension tuning

The JSON spec started as a copy of Chrono's `HMMWV_Vehicle_4WD.json` and its parts. The suspension values are our own tune. Edit the JSON, then press R in the app to restart with the new values. A rebuild is not necessary.

- **Chrono's HMMWV suspension sits on its droop stops.** The stock front spring is 6 mm from its droop stop at rest. The stock rear spring is 6.5 mm into its droop stop. In a bump test, the wheels spent about 1.7 s of 2.7 s on the stops. Pitch over the 15 cm bump was +/-6.7 deg.
- **The first-pass tune (2026-11-23):**
  - Linear springs. `Free Length` is the static length, so `Preload` (the static force) holds the ride height, and `Spring Coefficient` changes the stiffness only.
  - Targets: 1.3 Hz front and 1.5 Hz rear, with a motion ratio of about 0.49 and about 560 kg sprung mass for each corner.
  - Droop stops at 5 to 7 cm of spring extension. Bump stops at about 10 cm of compression.
  - Dampers (`Damping Curve Data`, velocity in m/s and force in N, compression is negative): about 0.25 of critical in compression and 0.5 in rebound. Above 0.25 m/s, the slope falls to 40%.
- **Result of the first-pass tune:** no time on the stops. The bump pitch is -5.3/+3.7 deg. The ride frequency is about 1.4 Hz front and 1.7 Hz rear (zero-crossing estimate, under throttle).
- **The softer springs make the truck accelerate more slowly on SCM** (9.5 m against 11.7 m in 3 s from rest). The dampers do not cause this. The cause is not confirmed. A possible cause is toe and camber change in droop, which makes the rigid tires scrub the soil.

## Performance facts (2026-11-23, RTX 4090 laptop, 22 cores)

- **Rendering limits frame time.** Physics does not. Physics runs at 4x to 9x real time up to a 0.025 m grid.
- **SCM physics cost does not change with map size.** Active domains limit ray casts to the area around the tires.
- **More threads do not help.** One thread is as fast as eight, and sixteen is slower.
- **The cause was full re-upload of mutable meshes.** Unpatched `ChVisualSystemVSG::Update()` rewrote and marked dirty the full vertex, normal, and color arrays of every mutable mesh each frame. VSG 1.1.16 has no partial buffer upload.
- **Fix (fork commit `831aef9`): tiled mutable meshes.** `ChVisualSystemVSG` splits each mutable triangle mesh into tiles of 16384 triangles. `ChVisualShapeTriangleMesh` accumulates modified vertices (`AddModifiedVertices`). SCM adds to that list. The VSG layer rewrites only the tiles that hold modified vertices, then clears the list. A shape without a modified list rewrites all tiles.
- **A coarse grid changes the physics.** At 0.4 m, the HMMWV reaches 1.4 m/s at full throttle. At 0.1 m, it reaches 8.5 m/s. Do not use a coarse grid as a performance fix.
- **CRM (SPH sand) is GPU-bound.** It is about 10x slower than real time on a 20 m x 3 m strip. It is not a candidate for real-time play.

## Benchmark mode (temporary)

`main.cpp` reads environment overrides: `BENCH=<sim seconds>` (full throttle, prints a timing line, then exits), `GRID`, `MAP`, `THR`, `RENDER=0`, `PACE=1`. Example:

```sh
nix develop -c sh -c 'BENCH=10 GRID=0.05 ./build/chrono_drive'
```

Results with BENCH=8 (RTF, unpatched -> tiled): 60/0.1: 4.20 -> 6.84. 60/0.05: 0.90 -> 3.53. 120/0.1: 1.63 -> 5.73. 240/0.1: 0.44 -> 1.77. 60/0.025: 0.11 -> 0.64. The end speeds are identical, so the physics did not change.

Other overrides: `WIRE=0` renders solid terrain. `SHOT=<file.png>` writes the last frame (it needs `vis->Run()` and a short sleep first, because `Render()` skips frames above its fps cap).

For an unpatched comparison, use `git -C chrono worktree add --detach ../chrono-base <commit>`, build it into `build-chrono-base`, then configure the app with `-DChrono_DIR=$PWD/build-chrono-base/cmake` into `build-base`. These directories are gitignored.

Ride logging: `LOG=<file.csv>` writes t, x, chassis z, pitch, roll, and the FL, FR, RL, RR spring lengths every 5 steps. `THROTTLE` sets the benchmark throttle (default 1). `SPAWNX` sets the spawn x. Examples:

```sh
BENCH=3 THROTTLE=0 LOG=settle.csv ./build/chrono_drive    # static ride height
BENCH=5 SPAWNX=0 LOG=bump.csv ./build/chrono_drive        # full throttle over the first bump (x = 10)
```

Remove this code after the SCM render work and the suspension tune are complete.
