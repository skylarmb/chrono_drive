// CompliantTire: a rigid tire on a sidewall spring. See compliant_tire.h.

#include "compliant_tire.h"

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/collision/ChCollisionShapeCylinder.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/input_output/ChUtilsJSON.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

using namespace chrono;
using namespace chrono::vehicle;

std::shared_ptr<ChTire> LoadTire(const std::string& filename) {
    rapidjson::Document d;
    ReadFileJSON(filename, d);
    if (std::string(d["Template"].GetString()) != "CompliantTire")
        return ReadTireJSON(filename);

    CompliantTire::Params p;
    p.radius = d["Radius"].GetDouble();
    p.width = d["Width"].GetDouble();
    p.mass = d["Mass"].GetDouble();
    p.stiffness = d["Sidewall Stiffness"].GetDouble();
    p.damping = d["Sidewall Damping"].GetDouble();
    p.max_deflection = d["Maximum Deflection"].GetDouble();
    p.stop_stiffness = d["Stop Stiffness"].GetDouble();
    return chrono_types::make_shared<CompliantTire>(d["Name"].GetString(), p);
}

CompliantTire::CompliantTire(const std::string& name, const Params& params) : ChTire(name), m_params(params) {}

ChVector3d CompliantTire::GetTireInertia() const {
    const double m = m_params.mass, r = m_params.radius, w = m_params.width;
    // Most of the mass is near the tread: a thick ring.
    return ChVector3d(m * (3 * r * r + w * w) / 12, 0.7 * m * r * r, m * (3 * r * r + w * w) / 12);
}

ChVector3d CompliantTire::HubCenter() const {
    return m_wheel->GetSpindle()->TransformPointLocalToParent(ChVector3d(0, GetOffset(), 0));
}

void CompliantTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);
    auto spindle = wheel->GetSpindle();
    auto sys = spindle->GetSystem();

    // Rubber on soil or rigid ground: the same values as the rigid tire.
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.9f);
    mat->SetRestitution(0.1f);
    mat->SetYoungModulus(2e7f);
    mat->SetPoissonRatio(0.3f);

    m_body = chrono_types::make_shared<ChBody>();
    m_body->SetName(m_name + "_body");
    m_body->SetMass(m_params.mass);
    m_body->SetInertiaXX(GetTireInertia());
    m_body->SetPos(HubCenter());
    m_body->SetRot(spindle->GetRot());
    m_body->SetPosDt(spindle->GetPosDt());
    // The spin axis is the spindle Y axis; a collision cylinder is along Z.
    m_body->AddCollisionShape(
        chrono_types::make_shared<ChCollisionShapeCylinder>(mat, m_params.radius, m_params.width),
        ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));
    m_body->EnableCollision(true);
    m_body->GetCollisionModel()->SetFamily(VehicleCollisionFamily::TIRE_FAMILY);
    sys->AddBody(m_body);

    // The Oldham joint frees translation in the joint XY plane: put its Z axis on the spin axis.
    m_joint = chrono_types::make_shared<ChLinkLockOldham>();
    m_joint->SetName(m_name + "_sidewall");
    m_joint->Initialize(m_body, spindle, ChFrame<>(HubCenter(), spindle->GetRot() * QuatFromAngleX(CH_PI_2)));
    sys->AddLink(m_joint);

    m_acc_tire = m_body->AddAccumulator();
    m_acc_spindle = spindle->AddAccumulator();
}

void CompliantTire::Synchronize(double time, const ChTerrain& terrain) {
    auto spindle = m_wheel->GetSpindle();
    const ChVector3d hub = HubCenter();

    // Deflection and its rate in the wheel plane (the joint keeps the axial part at zero).
    ChVector3d d = m_body->GetPos() - hub;
    ChVector3d v = m_body->GetPosDt() - spindle->PointSpeedLocalToParent(ChVector3d(0, GetOffset(), 0));
    ChVector3d force = -m_params.stiffness * d - m_params.damping * v;
    double dl = d.Length();
    if (dl > m_params.max_deflection)  // the rim
        force -= m_params.stop_stiffness * (dl - m_params.max_deflection) * (d / dl);

    // Equal and opposite: on the tire center, and on the hub center.
    m_body->EmptyAccumulator(m_acc_tire);
    spindle->EmptyAccumulator(m_acc_spindle);
    m_body->AccumulateForce(m_acc_tire, force, m_body->GetPos(), false);
    spindle->AccumulateForce(m_acc_spindle, -force, hub, false);
}

double CompliantTire::GetDeflection() const {
    return (m_body->GetPos() - HubCenter()).Length();
}

TerrainForce CompliantTire::GetTireForce() const {
    // Contacts on the tire body reach the hub through the joint and the spring: report no extra force.
    TerrainForce f;
    f.point = m_wheel->GetPos();
    return f;
}

TerrainForce CompliantTire::ReportTireForce(ChTerrain* terrain) const {
    TerrainForce f;
    f.point = m_body->GetPos();
    if (auto scm = dynamic_cast<SCMTerrain*>(terrain))
        scm->GetContactForceBody(m_body, f.force, f.moment);
    return f;
}

TerrainForce CompliantTire::ReportTireForceLocal(ChTerrain* terrain, ChCoordsys<>& tire_frame) const {
    throw std::runtime_error("CompliantTire::ReportTireForceLocal is not implemented");
}

void CompliantTire::InitializeInertiaProperties() {
    m_mass = GetTireMass();
    m_inertia.setZero();
    m_inertia.diagonal() = GetTireInertia().eigen();
    m_com = ChFrame<>();
}

void CompliantTire::UpdateInertiaProperties() {
    m_xform = ChFrame<>(m_body->GetPos(), m_body->GetRot());
}

void CompliantTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;
    // Draw the tire on the hub, not on the tire body: it stays centered on the rim. The drawn tire overlaps
    // the ground by the deflection (AGENTS.md: a deformed mesh would show the flat spot instead).
    m_vis_shape = chrono_types::make_shared<ChVisualShapeCylinder>(m_params.radius, m_params.width);
    m_vis_shape->SetColor(ChColor(0.15f, 0.15f, 0.15f));
    m_wheel->GetSpindle()->AddVisualShape(m_vis_shape, ChFrame<>(ChVector3d(0, GetOffset(), 0), QuatFromAngleX(CH_PI_2)));
}

void CompliantTire::RemoveVisualizationAssets() {
    // Remove only this shape: the wheel adds its own shapes to the same spindle body.
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_vis_shape);
}
