// chrono_drive: drive a test vehicle over deformable (SCM) terrain.
//
// Frame convention (Chrono::Vehicle): Z up, X forward, Y left.
//
// Controls (handled by Chrono's vehicle VSG viewer):
//   W / S        throttle / brake (held, game-style)
//   A / D        steer left / right
//   Z            toggle forward / reverse;  X  neutral
//   1..4         camera mode: chase, follow, track, inside
//   5            3/4 chase view (replaces the free camera)
//   6            side view: a fixed camera beside the vehicle (the start view)
//   R            restart: fresh map and vehicle, same model
//   Esc          quit
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

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Map settings
// -----------------------------------------------------------------------------

// Grayscale heightmap: black = min_height, white = max_height.
// tools/gen_heightmap.py writes it. The map size and height range must match that script.
const std::string heightmap = CHRONO_DRIVE_DATA_DIR "maps/test_track.bmp";
// BENCH (temporary): env overrides for benchmarking.
static double envd(const char* k, double d) { const char* v = getenv(k); return v ? atof(v) : d; }
const double map_x = 140.0, map_y = 40.0;  // map size [m]; the course runs along +X on the strip at y = 0
const double min_height = -0.5;  // [m]
const double max_height = 2.0;   // [m]
const double grid_delta = envd("GRID", 0.1);   // SCM grid spacing [m]; smaller = finer ruts, slower

// Physics step [s]. SMC contact with SCM is stable at 2 ms.
const double step_size = 2e-3;

// Side view: a fixed camera to the right of and slightly ahead of a point. It stays there and turns to
// follow the vehicle.
static ChVector3d SideCamera(const ChVector3d& p) { return p + ChVector3d(8, -12, 2); }

// 3/4 chase view: the chase camera orbited this far around the vehicle [rad].
const double three_quarter_angle = CH_PI / 4;

// Keys that the vehicle viewer does not handle (6, R), or that we change (5).
class AppKeys : public vsg3d::ChEventHandlerVSG {
  public:
    AppKeys(ChWheeledVehicleVisualSystemVSG& vis, bool& restart, bool& three_quarter)
        : vis(vis), restart(restart), three_quarter(three_quarter) {}
    void process(vsg::KeyPressEvent& ev) override {
        if (ev.keyBase == vsg::KEY_6)
            vis.GetChaseCamera().SetCameraPos(SideCamera(vis.GetChaseCamera().GetTargetPos()));
        if (ev.keyBase == vsg::KEY_r)
            restart = true;
        // The vehicle viewer handles 5 after this handler (free camera), so the main loop applies the view.
        if (ev.keyBase == vsg::KEY_5)
            three_quarter = true;
    }

  private:
    ChWheeledVehicleVisualSystemVSG& vis;
    bool& restart;
    bool& three_quarter;
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
    terrain.SetSoilParameters(1.4e6, // Bekker Kphi (2e6 in the Chrono demo; 1.4e6 sinks 25% deeper, more mud-like)
                              0,     // Bekker Kc
                              1.1,   // Bekker n exponent
                              0,     // Mohr cohesive limit [Pa]
                              30,    // Mohr friction limit [deg]
                              0.01,  // Janosi shear coefficient [m]
                              2e8,   // elastic stiffness [Pa/m]
                              3e4);  // damping [Pa s/m]
    terrain.Initialize(heightmap, map_x, map_y, min_height, max_height, grid_delta);
    terrain.SetPlotType(SCMTerrain::PLOT_SINKAGE, 0, 0.1);  // false-color ruts by depth
    terrain.GetMesh()->SetWireframe(envd("WIRE", 1) != 0);  // BENCH (temporary): WIRE=0 renders solid

    // -------------------------------------------------------------------------
    // Test course along +X on the flat strip. Rigid obstacles first, then the terrain sections in the
    // heightmap: cross-axle ditches (x = 33..45) and a side slope (x = 50..64).
    // -------------------------------------------------------------------------
    // Rigid ground material from Chrono's vehicle demos. The SMC default (E = 2e5 Pa) lets the tires sink in.
    auto obstacle_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    obstacle_mat->SetFriction(0.9f);
    obstacle_mat->SetRestitution(0.01f);
    obstacle_mat->SetYoungModulus(2e7f);
    obstacle_mat->SetPoissonRatio(0.3f);
    auto ground = [&](double x, double y) { return terrain.GetHeight(ChVector3d(x, y, 0)); };
    auto add_obstacle = [&](std::shared_ptr<ChBody> body, const ChVector3d& pos) {
        body->SetFixed(true);
        body->SetPos(pos);
        body->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.6f, 0.1f));
        sys.Add(body);
        return body;
    };
    // A bump is a cylinder across the track, buried so that `height` shows above the ground.
    auto add_bump = [&](double x, double y, double width, double radius, double height) {
        add_obstacle(chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, radius, width, 1000, true, true, obstacle_mat),
                     ChVector3d(x, y, ground(x, y) + height - radius));
    };
    const double width = 3.75;  // full-width obstacles [m]

    // 1. Single full-width bump: both wheels of an axle hit it together (heave and pitch).
    add_bump(-50, 0, width, 0.375, 0.15);
    // 2. Staggered bumps one wheelbase apart (3.0 m, see tools/gen_vehicle.py): the front right and rear left
    //    wheels are on bumps at the same time, which twists the chassis (cross-axle articulation).
    add_bump(-40, 0.9, 1.25, 0.375, 0.15);  // left wheels
    add_bump(-37, -0.9, 1.25, 0.375, 0.15);  // right wheels
    // 3. Log: a big hit that reaches the bump stops.
    add_bump(-28, 0, width, 0.18, 0.30);
    // 4. Washboard: small, closely spaced bumps (high-frequency damping, wheel hop).
    for (int i = 0; i < 10; i++)
        add_bump(-18 + 0.6 * i, 0, width, 0.15, 0.05);
    // 5. Whoops: alternating left and right bumps (repeated roll).
    for (int i = 0; i < 6; i++)
        add_bump(-5 + 2.5 * i, i % 2 ? -0.9 : 0.9, 1.25, 0.375, 0.15);
    // 6. Ramp up onto a 0.5 m step, then a drop off its end (landing, bottoming).
    const double ramp_x0 = 12, ramp_len = 3, step_len = 6, step_h = 0.5, thick = 0.3;
    const double ramp_angle = std::atan2(step_h, ramp_len);
    auto ramp = add_obstacle(chrono_types::make_shared<ChBodyEasyBox>(std::hypot(ramp_len, step_h), width, thick, 1000,
                                                                      true, true, obstacle_mat),
                             // Top face center, moved down half the thickness along the face normal.
                             ChVector3d(ramp_x0 + ramp_len / 2, 0, ground(ramp_x0, 0) + step_h / 2) +
                                 (thick / 2) * ChVector3d(std::sin(ramp_angle), 0, -std::cos(ramp_angle)));
    ramp->SetRot(QuatFromAngleY(-ramp_angle));  // the +X end rises
    add_obstacle(chrono_types::make_shared<ChBodyEasyBox>(step_len, width, step_h + thick, 1000, true, true, obstacle_mat),
                 ChVector3d(ramp_x0 + ramp_len + step_len / 2, 0, ground(ramp_x0, 0) + (step_h - thick) / 2));

    // -------------------------------------------------------------------------
    // Vehicle: spawn at the map center, just above the ground
    // -------------------------------------------------------------------------
    // Spawn at the start of the course (SPAWNX overrides, for tests of one section).
    const double spawn_x = envd("SPAWNX", -60);
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
    // Fullscreen when playing (Esc quits). Benchmark and screenshot runs use a small window.
    vis->SetWindowSize(1600, 900);
    vis->EnableFullscreen(envd("BENCH", 0) == 0);
    vis->SetChaseCamera(ChVector3d(0, 0, 1.75), 8.0, 1.0);
    vis->AttachVehicle(&vehicle);
    vis->AttachDriver(&driver);
    vis->AttachTerrain(&terrain);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->EnableSkyTexture(SkyMode::DOME);
    bool restart = false, three_quarter = false;
    vis->AddEventHandler(chrono_types::make_shared<AppKeys>(*vis, restart, three_quarter));
    vis->Initialize();
    vis->GetChaseCamera().SetCameraPos(SideCamera(ChVector3d(spawn_x, 0, spawn_z)));

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
        if (three_quarter) {  // key 5 (see AppKeys)
            vis->SetChaseCameraState(utils::ChChaseCamera::Chase);
            vis->GetChaseCamera().SetCameraAngle(three_quarter_angle);
            three_quarter = false;
        }
        double time = sys.GetChTime();

        if (bench_t > 0 && time >= bench_t) {
            t_wall.stop();
            double w = t_wall.GetTimeSeconds();
            printf("MAP=%g GRID=%g THR=%g RENDER=%d | v=%.2f RTF=%.2f | wall=%.2f render=%.2f step=%.2f "
                   "(coll=%.2f solve=%.2f) other=%.2f\n",
                   map_x, grid_delta, envd("THR", 8), bench_render, vehicle.GetSpeed(), time / w, w,
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
        if (log_file.is_open() && preview.empty() && step % 5 == 0) {  // proto only
            auto& ch = vehicle.GetChassis()->GetBody()->GetFrameRefToAbs();
            // Spring length of any suspension type (the spring is the first reported force element).
            auto spring = [&](int a, VehicleSide side) { return vehicle.GetSuspension(a)->ReportSuspensionForce(side)[0].length; };
            log_file << time << "," << vehicle.GetPos().x() << "," << ch.GetPos().z() << "," << ch.GetRot().GetCardanAnglesXYZ().y()
                     << "," << ch.GetRot().GetCardanAnglesXYZ().x() << "," << spring(0, LEFT) << ","
                     << spring(0, RIGHT) << "," << spring(1, LEFT) << "," << spring(1, RIGHT);
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
