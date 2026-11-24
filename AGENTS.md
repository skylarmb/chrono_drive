# AGENTS.md

Notes for agents that work on chrono_drive. `README.md` holds the user-facing build steps. This file holds the context, the decisions, and the traps.

## Project

chrono_drive is a physics-first driving simulator built on Project Chrono. The goal is a base for a game: a drivable vehicle on deformable terrain. The current state is a simple test vehicle (box frame, double wishbones) on a heightmap SCM terrain patch with test bumps. It has keyboard driving, a side camera, and a restart key.

## Layout

- `flake.nix`: the dev shells. `default` holds the toolchain, VSG, vsgXchange, and vsgImGui. `cuda` adds CUDA 12.9 for GPU modules.
- `chrono/`: a submodule on branch `chrono_drive` of `skylarmb/chrono` (a fork). Our Chrono patches live on that branch.
- `build-chrono/`: the Chrono build that chrono_drive links against (Vehicle + VSG, no demos). This directory is gitignored.
- `build/`: the chrono_drive build. This directory is gitignored.
- `build-crm/`: an experimental CUDA build of Chrono (FSI/SPH, CRM demos). It is not linked by chrono_drive.
- `data/vehicle/proto/`: the test vehicle (JSON written by `tools/gen_vehicle.py`), loaded with Chrono's generic `WheeledVehicle`. `SetVehicleDataPath` points to `data/vehicle/`.
- `src/sweep.cpp`: `suspension_sweep`, a kinematic check of the proto suspension.
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
- **A system we own gets the wrong solver.** `ChVehicle` sets a solver only on a system it creates. With the default PSOR, the steering and suspension joints stretch: the front wheels wobble, fold in reverse, and the truck drifts about 4 m over 50 m with zero steering. `main.cpp` uses `SPARSE_LU`, a direct solver. Its joints are exact, and it is faster than Barzilai-Borwein with 150 iterations (physics RTF 6.4 against about 4).
- **`ChSimpleDriveline` created energy (fixed on the fork).** It divided the wheel torque by the bevel gear ratio but multiplied the driveshaft speed by it. The wheels then got 25 times the engine power at ratio 0.2. The fork commit on `chrono_drive` divides the speed too.
- **A vehicle that creates its own system has no collision system.** Rigid tires then fall through the test rig posts. `sweep.cpp` sets Bullet.
- **The default SMC contact material is soft.** `ChContactMaterialSMC` defaults to E = 2e5 Pa. Tires sink 17 cm into a bump made with it. Rigid objects use E = 2e7 Pa, friction 0.9, restitution 0.01 (as in Chrono's vehicle demos). Then the tires sink 4 cm.
- **Do not use `ChRealtimeStepTimer::Spin()`.** It pads each step to the step size and drops the extra time from render frames. The loop then runs slower than real time on any hardware. `main.cpp` paces on accumulated sim time against the wall clock.
- **`CHRONO_VEHICLE_DATA_DIR` from the Chrono target is wrong for us.** It points to `build-chrono/bin/data/vehicle/`, which does not exist. Use `CHRONO_DATA_DIR "vehicle/"`.
- **SCM timers are not per-step seconds.** `SCMTerrain::GetTimer*()` returns milliseconds, and the reset happens in `SCMLoader::ComputeInternalForces()`. Do not sum the values per step. Use `perf` or wall-clock timers.
- **CUDA binaries load a stub `libcuda`.** The `cuda` shell puts `/run/opengl-driver/lib` on `LD_LIBRARY_PATH`. Without it, CUDA code fails with `cudaErrorStubLibrary`.
- **nvcc 12.9 does not accept GCC 16.** The `cuda` shell uses `cudaPackages_12.backendStdenv` (GCC 14).
- **VSG uses X11 (xcb).** On Wayland, it runs through XWayland. It renders on the NVIDIA GPU (confirmed with `nvidia-smi pmon`).

## Test vehicle (proto)

`tools/gen_vehicle.py` writes the "proto" vehicle to `data/vehicle/proto/`. Proto has a box frame with ballast mass, a double-wishbone front, a solid rear axle (`REAR_SUSPENSION = "solid"`, Chrono `SolidThreeLinkAxle` with vertical coilovers) or a wishbone rear, rack-and-pinion steering, and limited-slip 4WD. The parameters (mass, geometry, ride frequency, damping ratios, travel) are at the top of the script. To change the vehicle, edit the script, run it, then press R in the app. Do not edit the generated JSON by hand. `engine.json` and `transmission.json` are copies of Chrono's HMMWV powertrain, and the script does not write them.

`./build/suspension_sweep` runs a Chrono test rig. The rig holds the chassis and moves the wheels through their full travel, first together (heave), then in opposite directions (articulation). Then it turns the steering to full lock. The output is the toe and camber against wheel travel. On a solid axle, the toe change in articulation is roll steer. Run the sweep after each geometry change.

- **Results (2026-11-24):**
  - Toe changes by less than 0.7 deg over 30 cm of travel.
  - Steer at full lock is 30 deg.
  - Over the 15 cm bump at about 6 m/s, pitch is -3.7/+6.8 deg, and the springs reach both stops.
  - The ride is soft. Tune it by feel.
  - Solid rear (2026-11-24): heave gives no toe or camber change. Articulation reaches +/-20 cm of wheel travel and 12 deg of axle roll, with 0.2 deg of roll steer. The springs are inboard of the wheels, so the wheels articulate past the 12 cm stop setting.
  - The SimpleDriveline torque goes to the wheel shafts, so the solid axle does not show torque lift (driveshaft torque reacting on the axle). Unconfirmed: check this before tuning for it.
- **Spring curve convention:** `Free Length` is the ride length, and `Preload` holds the static load. The force is `Preload - curve(length - Free Length)`. The generator puts stiff progressive stops at the ends of the curve, because the JSON `Minimum/Maximum Length` stops are soft (1 kN at 10 mm).
- **The HMMWV model (removed) was a trap for suspension work.** Its stock suspension rests on the droop stops. Its steering linkage goes over center at about 0.34 m front spring length: under throttle, a front wheel flipped to about 45 deg and locked there. Do not judge a suspension without a sweep of the full travel.

## Performance facts (2026-11-23, RTX 4090 laptop, 22 cores)

- **Rendering limits frame time.** Physics does not. Physics runs at 4x to 9x real time up to a 0.025 m grid.
- **SCM physics cost does not change with map size.** Active domains limit ray casts to the area around the tires.
- **More threads do not help.** One thread is as fast as eight, and sixteen is slower.
- **The cause was full re-upload of mutable meshes.** Unpatched `ChVisualSystemVSG::Update()` rewrote and marked dirty the full vertex, normal, and color arrays of every mutable mesh each frame. VSG 1.1.16 has no partial buffer upload.
- **Fix (fork commit `831aef9`): tiled mutable meshes.** `ChVisualSystemVSG` splits each mutable triangle mesh into tiles of 16384 triangles. `ChVisualShapeTriangleMesh` accumulates modified vertices (`AddModifiedVertices`). SCM adds to that list. The VSG layer rewrites only the tiles that hold modified vertices, then clears the list. A shape without a modified list rewrites all tiles.
- **A coarse grid changes the physics.** At 0.4 m, the HMMWV reached 1.4 m/s at full throttle. At 0.1 m, it reached 8.5 m/s. Do not use a coarse grid as a performance fix.
- **CRM (SPH sand) is GPU-bound.** It is about 10x slower than real time on a 20 m x 3 m strip. It is not a candidate for real-time play.

## Benchmark mode (temporary)

`main.cpp` reads environment overrides: `BENCH=<sim seconds>` (full throttle, prints a timing line, then exits), `GRID`, `MAP`, `THR`, `RENDER=0`, `PACE=1`. Example:

```sh
nix develop -c sh -c 'BENCH=10 GRID=0.05 ./build/chrono_drive'
```

Results with BENCH=8 (RTF, unpatched -> tiled): 60/0.1: 4.20 -> 6.84. 60/0.05: 0.90 -> 3.53. 120/0.1: 1.63 -> 5.73. 240/0.1: 0.44 -> 1.77. 60/0.025: 0.11 -> 0.64. The end speeds are identical, so the physics did not change.

Other overrides: `WIRE=0` renders solid terrain. `SHOT=<file.png>` writes the last frame (it needs `vis->Run()` and a short sleep first, because `Render()` skips frames above its fps cap).

For an unpatched comparison, use `git -C chrono worktree add --detach ../chrono-base <commit>`, build it into `build-chrono-base`, then configure the app with `-DChrono_DIR=$PWD/build-chrono-base/cmake` into `build-base`. These directories are gitignored.

Ride logging: `LOG=<file.csv>` writes t, x, chassis z, pitch, roll, the FL, FR, RL, RR spring lengths, and the FL, FR steer angles [rad] every 5 steps. `THROTTLE` sets the benchmark throttle (default 1). `SPAWNX` sets the spawn x. Examples:

```sh
BENCH=3 THROTTLE=0 LOG=settle.csv ./build/chrono_drive    # static ride height
BENCH=5 SPAWNX=0 LOG=bump.csv ./build/chrono_drive        # full throttle over the first bump (x = 10)
```

Vehicle preview: `PREVIEW=uaz452` or `PREVIEW=man10t` drives one of Chrono's vehicles instead of proto. The files in `data/vehicle/preview/` replace the parts that do not work on SCM:
- **UAZ-452:** a 4WD copy and a rigid tire (Chrono ships only Pacejka and TMeasy tires for it).
- **MAN 10t:** a simple engine and a transmission with a 6:1 final drive. Its engine map gives about 150 N m at 0 rpm, and the simple transmission has no torque converter. Its `SimpleDrivelineXWD` has no final drive. Without these files, the truck does not move on SCM.

Remove this code after the SCM render work and the suspension tune are complete.
