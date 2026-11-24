// suspension_sweep: kinematic check of the proto vehicle (data/vehicle/proto/).
//
// A Chrono test rig holds the chassis fixed and moves a platform under each tire.
// The platforms move slowly through bump and droop with both wheels together (heave), then with the
// wheels in opposite directions (articulation), then the steering turns to full lock.
// The output is the steer (toe) and camber angle of the left wheels against wheel travel.
// Good geometry has a small toe change over the full travel. On a solid axle, toe change in
// articulation is roll steer.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "compliant_tire.h"

using namespace chrono;
using namespace chrono::vehicle;

// Phase lengths [s]: settle, travel sweep (up to bump, down to droop, back to ride), steering sweep.
// The travel sweep runs twice: heave, then articulation.
const double t_settle = 1.0, t_travel = 8.0, t_steer = 2.0;
const double t_heave_end = t_settle + t_travel, t_artic_end = t_heave_end + t_travel;

// Post inputs in [-1, 1], scaled by the rig displacement limit. Triangle wave: 0 -> +1 -> -1 -> 0.
class SweepDriver : public ChSuspensionTestRigDriver {
    void Synchronize(double time) override {
        ChSuspensionTestRigDriver::Synchronize(time);
        bool artic = time >= t_heave_end;
        double u = std::fmod(std::max(time - t_settle, 0.0), t_travel) / t_travel;
        double d = time >= t_artic_end ? 0 : (u < 0.25 ? 4 * u : (u < 0.75 ? 2 - 4 * u : 4 * u - 4));
        for (int a = 0; a < m_naxles; a++) {
            m_displLeft[a] = d;
            m_displRight[a] = artic ? -d : d;
        }
        m_steering = std::clamp((time - t_artic_end) / t_steer, 0.0, 1.0);
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
            rig->GetVehicle().InitializeTire(LoadTire(GetVehicleDataFile("proto/tire.json")), wheel,
                                             VisualizationType::NONE);
    rig->SetDriver(chrono_types::make_shared<SweepDriver>());
    rig->Initialize();
    rig->GetVehicle().GetSystem()->SetSolverType(ChSolver::Type::SPARSE_LU);

    // Travel sweeps: one row per cm of left wheel travel (first pass through each value only).
    // Per axle: travel [cm] -> steer and camber in heave, then in articulation.
    struct Row {
        double steer[2] = {NAN, NAN}, camber[2] = {NAN, NAN};
    };
    std::map<int, Row> rows[2];
    const double step = 1e-3;
    double steer_left = 0, steer_right = 0;
    while (rig->GetVehicle().GetSystem()->GetChTime() < t_artic_end + t_steer) {
        double t = rig->GetVehicle().GetSystem()->GetChTime();
        rig->Advance(step);
        if (t < t_settle)
            continue;
        if (t < t_artic_end) {
            int mode = t < t_heave_end ? 0 : 1;
            for (int a = 0; a < 2; a++) {
                Row& r = rows[a][(int)std::lround(rig->GetWheelTravel(a, LEFT) * 100)];
                if (std::isnan(r.steer[mode]))
                    Angles(rig->GetSpindleRot(a, LEFT), LEFT, r.steer[mode], r.camber[mode]);
            }
        } else {
            double c;
            Angles(rig->GetSpindleRot(0, LEFT), LEFT, steer_left, c);
            Angles(rig->GetSpindleRot(0, RIGHT), RIGHT, steer_right, c);
        }
    }

    for (int a = 0; a < 2; a++) {
        printf("%s left wheel (travel: + is bump) [deg]\n  travel[cm]  heave: steer  camber   articulation: steer  camber\n",
               a == 0 ? "FRONT" : "REAR");
        double range[2][2] = {{1e9, -1e9}, {1e9, -1e9}};  // per mode: min, max steer
        for (auto& [cm, r] : rows[a]) {
            printf("  %9d  %12.3f %7.3f  %19.3f %7.3f\n", cm, r.steer[0], r.camber[0], r.steer[1], r.camber[1]);
            for (int m = 0; m < 2; m++)
                if (!std::isnan(r.steer[m])) {
                    range[m][0] = std::min(range[m][0], r.steer[m]);
                    range[m][1] = std::max(range[m][1], r.steer[m]);
                }
        }
        printf("  toe change: heave %.3f deg, articulation %.3f deg\n\n", range[0][1] - range[0][0],
               range[1][1] - range[1][0]);
    }
    printf("full left lock: left wheel %.1f deg, right wheel %.1f deg\n", steer_left, steer_right);
    return 0;
}
