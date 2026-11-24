// CompliantTire: a rigid tire on a sidewall spring, for SCM soil.
//
// The tire is a rigid cylinder on its own body. An Oldham joint lets it slide in the wheel plane relative
// to the hub, but it turns with the hub. A spring-damper (the sidewall and the inflation pressure) pulls it
// back to the hub center, and a stiff stop limits the deflection (the rim). The tire carries the contact
// (rigid bodies and SCM), so it reports zero force to the vehicle: forces reach the hub through the joint
// and the spring.
//
// This gives ride compliance and tire squish under load. The contact patch does not flatten: the tire
// stays round, and its footprint in the soil comes from sinkage only.

#pragma once

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

/// Load a tire from JSON: a CompliantTire if the file's "Template" is "CompliantTire", else any Chrono tire.
std::shared_ptr<chrono::vehicle::ChTire> LoadTire(const std::string& filename);

class CompliantTire : public chrono::vehicle::ChTire {
  public:
    struct Params {
        double radius = 0.45;        // [m]
        double width = 0.30;         // [m]
        double mass = 40.0;          // [kg]
        double stiffness = 2.5e5;    // sidewall stiffness in the wheel plane [N/m]
        double damping = 1000;       // sidewall damping [N s/m]
        double max_deflection = 0.10;  // deflection where the stop (rim) starts [m]
        double stop_stiffness = 5e6;   // stiffness of the stop [N/m]
    };

    CompliantTire(const std::string& name, const Params& params);

    virtual std::string GetTemplateName() const override { return "CompliantTire"; }
    virtual double GetRadius() const override { return m_params.radius; }
    virtual double GetWidth() const override { return m_params.width; }
    virtual double GetTireMass() const override { return m_params.mass; }
    virtual chrono::ChVector3d GetTireInertia() const override;
    /// Tire center offset from the hub center, in the wheel plane [m].
    virtual double GetDeflection() const override;

    virtual chrono::vehicle::TerrainForce ReportTireForce(chrono::vehicle::ChTerrain* terrain) const override;
    virtual chrono::vehicle::TerrainForce ReportTireForceLocal(chrono::vehicle::ChTerrain* terrain,
                                                                 chrono::ChCoordsys<>& tire_frame) const override;

    virtual void Initialize(std::shared_ptr<chrono::vehicle::ChWheel> wheel) override;
    /// Apply the sidewall spring-damper force for this step.
    virtual void Synchronize(double time, const chrono::vehicle::ChTerrain& terrain) override;
    virtual void AddVisualizationAssets(chrono::VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override;

  private:
    virtual double GetAddedMass() const override { return 0; }  // the tire has its own body
    virtual chrono::ChVector3d GetAddedInertia() const override { return chrono::VNULL; }
    virtual chrono::vehicle::TerrainForce GetTireForce() const override;
    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Hub center (tire center at zero deflection) in the absolute frame.
    chrono::ChVector3d HubCenter() const;

    Params m_params;
    std::shared_ptr<chrono::ChBody> m_body;               // the tire
    std::shared_ptr<chrono::ChLinkLockOldham> m_joint;    // in-plane slide, no relative rotation
    unsigned int m_acc_tire = 0, m_acc_spindle = 0;       // force accumulators for the sidewall force
};
