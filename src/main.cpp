// chrono_drive: drive a test vehicle over deformable (SCM) terrain.
//
// Frame convention (Chrono::Vehicle): Z up, X forward, Y left.
//
// Controls (handled by Chrono's vehicle VSG viewer):
//   W / S        throttle / brake (held, game-style)
//   A / D        steer left / right
//   Z            toggle forward / reverse;  X  neutral
//   1..5         camera mode: chase, follow, track, inside, free
//   6            side view of the test track (the start view)
//   R            restart: fresh map and vehicle, same model
//   Arrow keys   orbit (left/right) and zoom (up/down) the camera
//   PgUp / PgDn  raise / lower the camera

#include <chrono>
#include <thread>
#include <fstream>

#include <unistd.h>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
#include "chrono_vsg/ChEventHandlerVSG.h"

#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishbone.h"

using namespace chrono;
using namespace chrono::vehicle;

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

// Fixed camera beside the bumps. The chase camera stays here and turns to follow the vehicle.
const ChVector3d side_camera(17, -14, 2.5);

// Keys that the vehicle viewer does not handle.
class AppKeys : public vsg3d::ChEventHandlerVSG {
  public:
    AppKeys(ChWheeledVehicleVisualSystemVSG& vis, bool& restart) : vis(vis), restart(restart) {}
    void process(vsg::KeyPressEvent& ev) override {
        if (ev.keyBase == vsg::KEY_6)
            vis.GetChaseCamera().SetCameraPos(side_camera);
        if (ev.keyBase == vsg::KEY_r)
            restart = true;
    }

  private:
    ChWheeledVehicleVisualSystemVSG& vis;
    bool& restart;
};

// Build and run one simulation. Returns true if the user asked for a restart.
static bool run() {
    // Point Chrono at its data (vehicle meshes, textures) in the submodule.
    SetChronoDataPath(CHRONO_DATA_DIR);
    // Vehicle specs (JSON) are ours to tune: data/vehicle/. Shared meshes there are symlinks into Chrono.
    SetVehicleDataPath(CHRONO_DRIVE_DATA_DIR "vehicle/");

    // Own the system so the terrain exists before the vehicle is placed on it.
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    // Direct sparse solver: the suspension and steering joints hold exactly. The generic default
    // (PSOR) lets them stretch under load. SMC contacts are forces, so a direct solver works here.
    sys.SetSolverType(ChSolver::Type::SPARSE_LU);
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
    // Rigid ground material from Chrono's vehicle demos. The SMC default (E = 2e5 Pa) lets the tires sink in.
    auto bump_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    bump_mat->SetFriction(0.9f);
    bump_mat->SetRestitution(0.01f);
    bump_mat->SetYoungModulus(2e7f);
    bump_mat->SetPoissonRatio(0.3f);
    auto add_bump = [&](double x, double y, double width, double radius, double height) {
        auto bump = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, radius, width, 1000, true, true, bump_mat);
        bump->SetFixed(true);
        bump->SetPos(ChVector3d(x, y, terrain.GetHeight(ChVector3d(x, y, 0)) + height - radius));
        bump->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.6f, 0.1f));
        sys.Add(bump);
    };
    // Single full-width bump: both wheels of an axle hit it together (heave and pitch).
    add_bump(10, 0, 3.75, 0.375, 0.15);
    // Staggered bumps one wheelbase apart (3.0 m, see tools/gen_vehicle.py): the front right and rear left wheels
    // are on bumps at the same time, which twists the chassis (cross-axle articulation).
    add_bump(20.0, 0.9, 1.25, 0.375, 0.15);   // left wheels
    add_bump(23.0, -0.9, 1.25, 0.375, 0.15);  // right wheels

    // -------------------------------------------------------------------------
    // Vehicle: spawn at the map center, just above the ground
    // -------------------------------------------------------------------------
    // BENCH (temporary): in benchmark mode, spawn 5 m from the -X edge so the run stays on the map.
    const double spawn_x = envd("SPAWNX", envd("BENCH", 0) > 0 ? -map_size / 2 + 5 : 0);
    // Vehicle specs: JSON paths and the spawn height of the chassis frame above the ground.
    struct Spec {
        std::string vehicle, engine, transmission, tire;
        double spawn_height;
    };
    // Test vehicle (tools/gen_vehicle.py writes data/vehicle/proto/). Press R to restart with new values.
    // The chassis frame origin is at wheel center height: tire radius (0.45 m) plus a small drop.
    Spec spec{"proto/vehicle.json", "proto/engine.json", "proto/transmission.json", "proto/tire.json", 0.5};

    // PREVIEW (temporary): PREVIEW=uaz452 or PREVIEW=man10t drives one of Chrono's vehicles instead.
    const std::string preview = getenv("PREVIEW") ? getenv("PREVIEW") : "";
    const std::string preview_dir = CHRONO_DRIVE_DATA_DIR "vehicle/preview/";
    if (!preview.empty())
        SetVehicleDataPath(CHRONO_DATA_DIR "vehicle/");  // their parts and meshes live in Chrono's data
    if (preview == "uaz452")  // 4WD copy of the UAZ-452 van, with a rigid tire of the UAZ size
        spec = {preview_dir + "UAZBUS_4WD.json", "uaz/powertrain/UAZBUS_EngineSimpleMap.json",
                "uaz/powertrain/UAZBUS_AutomaticTransmissionSimpleMap.json", preview_dir + "uaz_rigid_tire.json", 0.5};
    if (preview == "man10t")  // MAN 10t 8x8 with its own rigid mesh tire (0.63 m radius). Its engine map stalls at 0 rpm without a torque converter, and its XWD driveline has no final drive.
        spec = {"MAN/vehicle/MAN_10t_Vehicle_8WD.json", preview_dir + "man_engine_simple.json",
                preview_dir + "man_transmission.json", "MAN/tire/MAN_RigidMeshTire.json", 0.8};
    // Absolute paths are ours; others are relative to the vehicle data path.
    auto path = [](const std::string& f) { return f[0] == '/' ? f : GetVehicleDataFile(f); };
    const auto vis_type = preview.empty() ? VisualizationType::PRIMITIVES : VisualizationType::MESH;
    const double spawn_z = terrain.GetHeight(ChVector3d(spawn_x, 0, 0)) + spec.spawn_height;

    WheeledVehicle vehicle(&sys, path(spec.vehicle));
    vehicle.Initialize(ChCoordsys<>(ChVector3d(spawn_x, 0, spawn_z), QUNIT));
    vehicle.SetChassisVisualizationType(vis_type);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(vis_type);

    auto engine = ReadEngineJSON(path(spec.engine));
    auto transmission = ReadTransmissionJSON(path(spec.transmission));
    vehicle.InitializePowertrain(chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission));

    for (auto& axle : vehicle.GetAxles())
        for (auto& wheel : axle->GetWheels())
            vehicle.InitializeTire(ReadTireJSON(path(spec.tire)), wheel, vis_type);

    // Only ray-cast SCM nodes near the tires (big speedup on large maps). The box covers the whole tire.
    for (auto& axle : vehicle.GetAxles())
        for (auto& wheel : axle->GetWheels()) {
            double r = wheel->GetTire()->GetRadius();
            terrain.AddActiveDomain(wheel->GetSpindle(), VNULL, ChVector3d(2.2 * r, 0.6, 2.2 * r));
        }

    // -------------------------------------------------------------------------
    // Driver + visualization
    // -------------------------------------------------------------------------
    ChInteractiveDriver driver(vehicle);
    driver.SetKeyboardMode(ChInteractiveDriver::KeyboardMode::HELD);
    driver.Initialize();

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
    vis->SetWindowTitle("chrono_drive");
    vis->SetWindowSize(1600, 900);
    vis->SetChaseCamera(ChVector3d(0, 0, 1.75), 8.0, 1.0);
    vis->AttachVehicle(&vehicle);
    vis->AttachDriver(&driver);
    vis->AttachTerrain(&terrain);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->EnableSkyTexture(SkyMode::DOME);
    bool restart = false;
    vis->AddEventHandler(chrono_types::make_shared<AppKeys>(*vis, restart));
    vis->Initialize();
    vis->GetChaseCamera().SetCameraPos(side_camera);

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

    while (vis->Run() && !restart) {
        double time = sys.GetChTime();

        if (bench_t > 0 && time >= bench_t) {
            t_wall.stop();
            double w = t_wall.GetTimeSeconds();
            printf("MAP=%g GRID=%g THR=%g RENDER=%d | v=%.2f RTF=%.2f | wall=%.2f render=%.2f step=%.2f "
                   "(coll=%.2f solve=%.2f) other=%.2f\n",
                   map_size, grid_delta, envd("THR", 8), bench_render, vehicle.GetSpeed(), time / w, w,
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
            inputs.m_throttle = envd("THROTTLE", 1);
            inputs.m_braking = 0;
        }
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, inputs, terrain);
        vis->Synchronize(time, inputs);

        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
        vis->Advance(step_size);
        // We own the system (not the vehicle), so the vehicle's Advance() does not step physics.
        t_step.start();
        sys.DoStepDynamics(step_size);
        t_step.stop();
        coll += sys.GetTimerCollision();
        solve += sys.GetTimerLSsolve() + sys.GetTimerLSsetup();

        // LOG (temporary): ride data for suspension tuning, every 5 steps.
        static std::ofstream log_file(getenv("LOG") ? getenv("LOG") : "");
        if (log_file.is_open() && preview.empty() && step % 5 == 0) {  // proto only (double wishbones)
            auto& ch = vehicle.GetChassis()->GetBody()->GetFrameRefToAbs();
            auto susp = [&](int a) { return std::static_pointer_cast<ChDoubleWishbone>(vehicle.GetSuspension(a)); };
            log_file << time << "," << vehicle.GetPos().x() << "," << ch.GetPos().z() << "," << ch.GetRot().GetCardanAnglesXYZ().y()
                     << "," << ch.GetRot().GetCardanAnglesXYZ().x() << "," << susp(0)->GetSpringLength(LEFT) << ","
                     << susp(0)->GetSpringLength(RIGHT) << "," << susp(1)->GetSpringLength(LEFT) << ","
                     << susp(1)->GetSpringLength(RIGHT);
            // Front wheel steer angles relative to the chassis (spindle axis in the chassis frame)
            for (auto side : {LEFT, RIGHT}) {
                auto ax = ch.GetRot().RotateBack(vehicle.GetSpindleRot(0, side).GetAxisY());
                log_file << "," << std::atan2(ax.x(), ax.y());
            }
            log_file << "\n";
        }
        step++;
    }

    return restart;
}

int main(int argc, char** argv) {
    // Restart by re-running this executable, so every Chrono object starts fresh.
    // run() returns first, which closes the window.
    if (run())
        execv("/proc/self/exe", argv);
    return 0;
}
