// suspension_sweep: kinematic check of the proto vehicle (data/vehicle/proto/).
//
// A Chrono test rig holds the chassis fixed and moves a platform under each tire.
// The platforms move slowly through bump and droop, then the steering turns to full lock.
// The output is the steer (toe) and camber angle of the left wheels against wheel travel.
// Good geometry has a small toe change over the full travel.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

// Phase lengths [s]: settle, travel sweep (up to bump, down to droop, back to ride), steering sweep.
const double t_settle = 1.0, t_travel = 8.0, t_steer = 2.0;

// Post inputs in [-1, 1], scaled by the rig displacement limit. Triangle wave: 0 -> +1 -> -1 -> 0.
class SweepDriver : public ChSuspensionTestRigDriver {
    void Synchronize(double time) override {
        ChSuspensionTestRigDriver::Synchronize(time);
        double u = std::clamp((time - t_settle) / t_travel, 0.0, 1.0);
        double d = u < 0.25 ? 4 * u : (u < 0.75 ? 2 - 4 * u : 4 * u - 4);
        for (int a = 0; a < m_naxles; a++)
            m_displLeft[a] = m_displRight[a] = d;
        double s = std::clamp((time - t_settle - t_travel) / t_steer, 0.0, 1.0);
        m_steering = s;
    }
};

// Steer angle (positive = wheel points left) and camber (positive = top of the wheel leans out) of a wheel.
static void Angles(const ChQuaternion<>& rot, VehicleSide side, double& steer, double& camber) {
    ChVector3d a = rot.GetAxisY();  // spin axis, points to the left
    steer = std::atan2(-a.x(), a.y()) * CH_RAD_TO_DEG;
    camber = std::atan2(side == LEFT ? -a.z() : a.z(), a.y()) * CH_RAD_TO_DEG;
}

int main() {
    SetChronoDataPath(CHRONO_DATA_DIR);
    SetVehicleDataPath(CHRONO_DRIVE_DATA_DIR "vehicle/");

    auto vehicle = chrono_types::make_shared<WheeledVehicle>(GetVehicleDataFile("proto/vehicle.json"),
                                                             ChContactMethod::SMC);
    vehicle->GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);  // rigid tires touch the posts
    const double displ_limit = 0.2;  // larger than the bump and droop travel, so the stops engage
    auto rig = chrono_types::make_shared<ChSuspensionTestRigPlatform>(vehicle, std::vector<int>{0, 1}, displ_limit);
    rig->IncludeSteeringMechanism(0);
    rig->SetInitialRideHeight(0.45);  // chassis frame height above the posts: tire radius (frame origin is at wheel center)
    for (auto& axle : rig->GetVehicle().GetAxles())
        for (auto& wheel : axle->GetWheels())
            rig->GetVehicle().InitializeTire(ReadTireJSON(GetVehicleDataFile("proto/tire.json")), wheel,
                                             VisualizationType::NONE);
    rig->SetDriver(chrono_types::make_shared<SweepDriver>());
    rig->Initialize();
    rig->GetVehicle().GetSystem()->SetSolverType(ChSolver::Type::SPARSE_LU);

    // Travel sweep: one row per cm of travel (first pass through each value only).
    std::map<int, std::pair<double, double>> rows[2];  // per axle: travel [cm] -> (steer, camber)
    const double step = 1e-3;
    double steer_left = 0, steer_right = 0;
    while (rig->GetVehicle().GetSystem()->GetChTime() < t_settle + t_travel + t_steer) {
        double t = rig->GetVehicle().GetSystem()->GetChTime();
        rig->Advance(step);
        if (t < t_settle)
            continue;
        if (t < t_settle + t_travel) {
            for (int a = 0; a < 2; a++) {
                double steer, camber;
                Angles(rig->GetSpindleRot(a, LEFT), LEFT, steer, camber);
                int cm = (int)std::lround(rig->GetWheelTravel(a, LEFT) * 100);
                rows[a].try_emplace(cm, steer, camber);
            }
        } else {
            double c;
            Angles(rig->GetSpindleRot(0, LEFT), LEFT, steer_left, c);
            Angles(rig->GetSpindleRot(0, RIGHT), RIGHT, steer_right, c);
        }
    }

    for (int a = 0; a < 2; a++) {
        printf("%s left wheel (travel: + is bump)\n  travel[cm]  steer[deg]  camber[deg]\n", a == 0 ? "FRONT" : "REAR");
        double smin = 1e9, smax = -1e9;
        for (auto& [cm, r] : rows[a]) {
            printf("  %9d  %10.3f  %11.3f\n", cm, r.first, r.second);
            smin = std::min(smin, r.first);
            smax = std::max(smax, r.first);
        }
        printf("  toe change over travel: %.3f deg\n\n", smax - smin);
    }
    printf("full left lock: left wheel %.1f deg, right wheel %.1f deg\n", steer_left, steer_right);
    return 0;
}
