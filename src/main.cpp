// chrono_drive: drive an HMMWV over deformable (SCM) terrain.
//
// Frame convention (Chrono::Vehicle): Z up, X forward, Y left.
//
// Controls (handled by Chrono's vehicle VSG viewer):
//   W / S        throttle / brake (held, game-style)
//   A / D        steer left / right
//   Z            toggle forward / reverse;  X  neutral
//   1..5         camera mode: chase, follow, track, inside, free
//   Arrow keys   orbit (left/right) and zoom (up/down) the camera
//   PgUp / PgDn  raise / lower the camera

#include <chrono>
#include <thread>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// -----------------------------------------------------------------------------
// Map settings
// -----------------------------------------------------------------------------

// Grayscale heightmap: black = min_height, white = max_height.
const std::string heightmap = CHRONO_DRIVE_DATA_DIR "maps/test_track.bmp";  // tools/gen_heightmap.py
// BENCH (temporary): env overrides for benchmarking.
static double envd(const char* k, double d) { const char* v = getenv(k); return v ? atof(v) : d; }
const double map_size = envd("MAP", 60.0);    // square map edge length [m]
const double min_height = 0.0;   // [m]
const double max_height = 2.0;   // [m]
const double grid_delta = envd("GRID", 0.1);   // SCM grid spacing [m]; smaller = finer ruts, slower

// Physics step [s]. SMC contact with SCM is stable at 2 ms.
const double step_size = 2e-3;

int main() {
    // Point Chrono at its data (vehicle meshes, textures) in the submodule.
    SetChronoDataPath(CHRONO_DATA_DIR);
    SetVehicleDataPath(CHRONO_DATA_DIR "vehicle/");

    // Own the system so the terrain exists before the vehicle is placed on it.
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    // Match the solver ChVehicle sets on a system it creates itself (ChVehicle.cpp). The generic
    // default (PSOR) is too weak for the suspension and steering joints, which then stretch under load.
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetNumThreads((int)envd("THR", 8), (int)envd("THR", 8), 1);

    // -------------------------------------------------------------------------
    // Deformable terrain
    // -------------------------------------------------------------------------
    SCMTerrain terrain(&sys);
    // Soil parameters from Chrono's demo_VEH_SCMTerrain_WheeledVehicle (firm loose soil).
    terrain.SetSoilParameters(2e6,   // Bekker Kphi
                              0,     // Bekker Kc
                              1.1,   // Bekker n exponent
                              0,     // Mohr cohesive limit [Pa]
                              30,    // Mohr friction limit [deg]
                              0.01,  // Janosi shear coefficient [m]
                              2e8,   // elastic stiffness [Pa/m]
                              3e4);  // damping [Pa s/m]
    terrain.Initialize(heightmap, map_size, map_size, min_height, max_height, grid_delta);
    terrain.SetPlotType(SCMTerrain::PLOT_SINKAGE, 0, 0.1);  // false-color ruts by depth
    terrain.GetMesh()->SetWireframe(envd("WIRE", 1) != 0);  // BENCH (temporary): WIRE=0 renders solid

    // -------------------------------------------------------------------------
    // Suspension test obstacles on the flat strip (+X from the spawn point)
    // -------------------------------------------------------------------------
    // Each bump is a fixed cylinder across the track, buried so that `height` shows above the ground.
    auto bump_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    auto add_bump = [&](double x, double y, double width, double radius, double height) {
        auto bump = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, radius, width, 1000, true, true, bump_mat);
        bump->SetFixed(true);
        bump->SetPos(ChVector3d(x, y, terrain.GetHeight(ChVector3d(x, y, 0)) + height - radius));
        bump->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.6f, 0.1f));
        sys.Add(bump);
    };
    // Single full-width bump: both wheels of an axle hit it together (heave and pitch).
    add_bump(10, 0, 3.0, 0.3, 0.12);
    // Staggered bumps one wheelbase apart (HMMWV: 3.3 m): the front right and rear left wheels
    // are on bumps at the same time, which twists the chassis (cross-axle articulation).
    add_bump(20.0, 0.9, 1.0, 0.3, 0.12);   // left wheels
    add_bump(23.3, -0.9, 1.0, 0.3, 0.12);  // right wheels

    // -------------------------------------------------------------------------
    // Vehicle: spawn at the map center, just above the ground
    // -------------------------------------------------------------------------
    // BENCH (temporary): in benchmark mode, spawn 5 m from the -X edge so the run stays on the map.
    const double spawn_x = envd("BENCH", 0) > 0 ? -map_size / 2 + 5 : 0;
    const double spawn_z = terrain.GetHeight(ChVector3d(spawn_x, 0, 0)) + 0.6;

    HMMWV_Full hmmwv(&sys);
    hmmwv.SetContactMethod(ChContactMethod::SMC);
    hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(spawn_x, 0, spawn_z), QUNIT));
    hmmwv.SetEngineType(EngineModelType::SIMPLE);
    hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    hmmwv.SetTireType(TireModelType::RIGID);
    hmmwv.SetTireStepSize(step_size);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(VisualizationType::MESH);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetWheelVisualizationType(VisualizationType::MESH);
    hmmwv.SetTireVisualizationType(VisualizationType::MESH);

    // Only ray-cast SCM nodes near the tires (big speedup on large maps).
    for (auto& axle : hmmwv.GetVehicle().GetAxles())
        for (auto& wheel : axle->GetWheels())
            terrain.AddActiveDomain(wheel->GetSpindle(), VNULL, ChVector3d(1.0, 0.6, 1.0));

    // -------------------------------------------------------------------------
    // Driver + visualization
    // -------------------------------------------------------------------------
    ChInteractiveDriver driver(hmmwv.GetVehicle());
    driver.SetKeyboardMode(ChInteractiveDriver::KeyboardMode::HELD);
    driver.Initialize();

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
    vis->SetWindowTitle("chrono_drive");
    vis->SetWindowSize(1600, 900);
    vis->SetChaseCamera(ChVector3d(0, 0, 1.75), 8.0, 1.0);
    vis->AttachVehicle(&hmmwv.GetVehicle());
    vis->AttachDriver(&driver);
    vis->AttachTerrain(&terrain);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->EnableSkyTexture(SkyMode::DOME);
    vis->Initialize();

    // -------------------------------------------------------------------------
    // Loop: step physics, render at ~60 fps, cap to real time
    // -------------------------------------------------------------------------
    const int steps_per_frame = static_cast<int>(std::round((1.0 / 60) / step_size));
    using clock = std::chrono::steady_clock;
    auto wall_origin = clock::now();  // wall-clock time that corresponds to sim time 0
    int step = 0;

    // BENCH (temporary): BENCH=<sim seconds> forces full throttle and prints a timing breakdown.
    const double bench_t = envd("BENCH", 0);
    const bool bench_render = envd("RENDER", 1) != 0;
    ChTimer t_wall, t_render, t_step;
    double coll = 0, solve = 0;
    t_wall.start();

    while (vis->Run()) {
        double time = sys.GetChTime();

        if (bench_t > 0 && time >= bench_t) {
            t_wall.stop();
            double w = t_wall.GetTimeSeconds();
            printf("MAP=%g GRID=%g THR=%g RENDER=%d | v=%.2f RTF=%.2f | wall=%.2f render=%.2f step=%.2f "
                   "(coll=%.2f solve=%.2f) other=%.2f\n",
                   map_size, grid_delta, envd("THR", 8), bench_render, hmmwv.GetVehicle().GetSpeed(), time / w, w,
                   t_render.GetTimeSeconds(), t_step.GetTimeSeconds(), coll, solve,
                   w - t_render.GetTimeSeconds() - t_step.GetTimeSeconds());
            // SHOT=<file.png>: capture one final frame for visual checks.
            if (const char* shot = getenv("SHOT")) {
                vis->WriteImageToFile(shot);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Render() skips frames above its fps cap
                vis->Run();
                vis->BeginScene();
                vis->Render();
                vis->EndScene();
            }
            break;
        }

        if (step % steps_per_frame == 0) {
            if (bench_t == 0 || bench_render) {
                t_render.start();
                vis->BeginScene();
                vis->Render();
                vis->EndScene();
                t_render.stop();
            }

            // Pace to real time: sleep while the simulation is ahead of the wall clock.
            // If we fall far behind (e.g. a hitch), re-anchor instead of fast-forwarding to catch up.
            auto sim_now = wall_origin + std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(time));
            if (clock::now() - sim_now > std::chrono::milliseconds(100))
                wall_origin += clock::now() - sim_now;
            else if (bench_t == 0 || envd("PACE", 0))
                std::this_thread::sleep_until(sim_now);
        }

        DriverInputs inputs = driver.GetInputs();
        if (bench_t > 0) {
            inputs.m_throttle = 1;
            inputs.m_braking = 0;
        }
        driver.Synchronize(time);
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, inputs, terrain);
        vis->Synchronize(time, inputs);

        driver.Advance(step_size);
        terrain.Advance(step_size);
        hmmwv.Advance(step_size);
        vis->Advance(step_size);
        // We own the system (not the vehicle), so the vehicle's Advance() does not step physics.
        t_step.start();
        sys.DoStepDynamics(step_size);
        t_step.stop();
        coll += sys.GetTimerCollision();
        solve += sys.GetTimerLSsolve() + sys.GetTimerLSsetup();

        step++;
    }

    return 0;
}
