#!/usr/bin/env python3
"""Generate the "proto" test vehicle: a box frame, a double-wishbone front, and a solid or wishbone rear.

Edit the parameters below, run this script, then press R in chrono_drive.
The script writes Chrono::Vehicle JSON files to data/vehicle/proto/.

Frames (Chrono::Vehicle): X forward, Y left, Z up. Each suspension frame has its
origin at the wheel center at ride height. The script gives left-side points.
Chrono mirrors them to the right side.

Geometry rules (double wishbone):
- Both control arms are horizontal at ride height. The upper arm is shorter,
  so the wheel gains negative camber in bump.
- The tie rod forms a third arm. Its inner point lies on the line through the
  inner arm pivots, and its outer point lies on the kingpin line (both in the
  front view). This keeps the toe change small over the wheel travel.
- The spring and the damper (coilover) mount on the lower arm.
"""
import json
import math
import os

OUT = os.path.join(os.path.dirname(__file__), "..", "data", "vehicle", "proto")
G = 9.81

# -----------------------------------------------------------------------------
# Parameters
# -----------------------------------------------------------------------------

# Body
SPRUNG_MASS = 2000.0        # frame + ballast [kg]
CG_HEIGHT = 0.90            # above ground [m]
CG_X = 0.0                  # ahead of the wheelbase center [m]
FRAME_SIZE = (4.2, 1.4, 0.3)  # visual box: length, width, height [m]
INERTIA_BOX = (4.0, 1.8, 1.2)  # box that sets the moments of inertia [m]

# Layout
WHEELBASE = 3.0             # [m]
TRACK = 1.8                 # wheel center to wheel center [m]
TIRE_RADIUS = 0.45          # [m]
TIRE_WIDTH = 0.30           # [m]
TIRE_MASS = 40.0            # [kg]
WHEEL_MASS = 20.0           # rim [kg]
# Tire type: "compliant" (rigid cylinder on a sidewall spring, src/compliant_tire.h) or "rigid"
TIRE_TYPE = "compliant"
TIRE_STIFFNESS = 2.5e5      # sidewall stiffness [N/m]; lower = softer tire (lower pressure)
TIRE_DAMPING = 1000.0       # sidewall damping [N s/m]
TIRE_MAX_DEFLECTION = 0.10  # deflection where the rim stop starts [m]

# Corner geometry (relative to the wheel center at ride height) [m]
LOWER_BALL_JOINT = (0.08, -0.15)  # (inboard offset, height)
UPPER_BALL_JOINT = (0.14, 0.20)   # (inboard offset, height)
LOWER_ARM_LENGTH = 0.45           # front view, ball joint to inner pivot
UPPER_ARM_LENGTH = 0.30
STEERING_ARM = 0.15               # tie rod outer point behind the kingpin
TIEROD_HEIGHT = 0.0
SPRING_ON_ARM = 0.65              # coilover mount, fraction of lower arm from inner pivot
SPRING_TOP = (0.05, 0.35)         # coilover top: (inboard of the bottom mount, height)

# Rear suspension type: "solid" (three-link coilover axle) or "wishbone" (same as the front)
REAR_SUSPENSION = "solid"

# Solid axle geometry (relative to the axle center at ride height; links point forward) [m]
AXLE_TUBE_MASS = 120.0            # tube, differential, and brakes [kg]
TRIANGLE_APEX_HEIGHT = 0.20       # ball joint on top of the differential; sets the roll center
TRIANGLE_CHASSIS = (0.60, 0.40)   # chassis pivots: (ahead of the axle, half spacing)
LOWER_LINK_AXLE = (0.55, -0.15)   # lower link axle mounts: (half spacing, height)
LOWER_LINK_LENGTH = 0.70          # lower link chassis mounts are this far ahead, at the same height
COIL_AXLE = (0.62, -0.05)         # coilover axle mount: (half spacing, height)
COIL_TOP_HEIGHT = 0.45            # coilover chassis mount height (vertical coilover)
TRIANGLE_MASS, LOWER_LINK_MASS = 10.0, 6.0

# Ride (per axle: front, rear)
RIDE_FREQ = (1.3, 1.5)            # [Hz]
DAMPING_COMPRESSION = (0.25, 0.25)  # fraction of critical
DAMPING_REBOUND = (0.5, 0.5)        # fraction of critical
DAMPER_KNEE = 0.4                 # wheel speed where damping falls to 40% [m/s]
DROOP_TRAVEL = (0.12, 0.12)       # wheel travel down from ride height to the droop stop [m]
BUMP_TRAVEL = (0.15, 0.15)        # wheel travel up from ride height to the bump stop [m]

# Powertrain: the HMMWV simple engine and its first two gears, in low range (crawl speeds).
# Top speed = governor speed * top gear / low range * conical gear ratio * tire radius (about 3 m/s).
ENGINE_TORQUE = 330.0             # [N m]
ENGINE_POWER = 110e3              # [W]
GOVERNOR_RPM = 3000.0             # no torque above this engine speed
GEAR_RATIOS = (0.1708, 0.2791)    # transmission output / input speed (HMMWV gears 1 and 2)
REVERSE_RATIO = -0.151
LOW_RANGE = 2.6                   # transfer case reduction
SHIFT_RPM = (1000, 2200)          # downshift, upshift engine speeds

# Steering and driveline
MAX_STEER_DEG = 30.0
TORQUE_BIAS = 2.5                 # limited-slip axle differentials (Torsen-like)
FRONT_TORQUE_FRACTION = 0.5
CONICAL_GEAR_RATIO = 0.2          # as in the HMMWV driveline
BRAKE_TORQUE = 4000.0             # per wheel [N m]

# Unsprung part masses [kg]
SPINDLE_MASS, UPRIGHT_MASS, UPPER_ARM_MASS, LOWER_ARM_MASS = 10.0, 15.0, 5.0, 8.0

# -----------------------------------------------------------------------------
# Derived geometry
# -----------------------------------------------------------------------------

Y_WHEEL = TRACK / 2
y_lbj, z_lbj = Y_WHEEL - LOWER_BALL_JOINT[0], LOWER_BALL_JOINT[1]
y_ubj, z_ubj = Y_WHEEL - UPPER_BALL_JOINT[0], UPPER_BALL_JOINT[1]
y_lin, y_uin = y_lbj - LOWER_ARM_LENGTH, y_ubj - UPPER_ARM_LENGTH


def lerp_y(y0, z0, y1, z1, z):
    """Y on the line (y0, z0)-(y1, z1) at height z."""
    return y0 + (y1 - y0) * (z - z0) / (z1 - z0)


tierod_out = [-STEERING_ARM, lerp_y(y_lbj, z_lbj, y_ubj, z_ubj, TIEROD_HEIGHT), TIEROD_HEIGHT]
tierod_in = [-STEERING_ARM, lerp_y(y_lin, z_lbj, y_uin, z_ubj, TIEROD_HEIGHT), TIEROD_HEIGHT]

y_spring = y_lin + SPRING_ON_ARM * LOWER_ARM_LENGTH
spring_bot = [0.0, y_spring, z_lbj]
spring_top = [0.0, y_spring - SPRING_TOP[0], SPRING_TOP[1]]
spring_len = math.dist(spring_bot, spring_top)
# Motion ratio: spring travel per unit of wheel travel (lever ratio times the spring angle cosine).
motion_ratio = SPRING_ON_ARM * (SPRING_TOP[1] - z_lbj) / spring_len

unsprung = SPINDLE_MASS + UPRIGHT_MASS + TIRE_MASS + WHEEL_MASS + (UPPER_ARM_MASS + LOWER_ARM_MASS) / 2
front_share = 0.5 + CG_X / WHEELBASE
corner_mass = (SPRUNG_MASS * front_share / 2, SPRUNG_MASS * (1 - front_share) / 2)


def rod_inertia(m, length):
    """Moments of inertia of a thin rod along Y (small, nonzero on the axis)."""
    i = m * length**2 / 12
    return [i, 1e-3 + m * 1e-3, i]


def ride_elements(axle, motion_ratio):
    """Spring and damper for axle 0 (front) or 1 (rear), from the ride targets.

    motion_ratio is the spring travel per unit of wheel travel.
    Returns (preload, spring curve, damper curve).
    """
    m = corner_mass[axle]
    k_wheel = (2 * math.pi * RIDE_FREQ[axle]) ** 2 * m
    k_spring = k_wheel / motion_ratio**2
    preload = m * G / motion_ratio
    c_crit = 2 * math.sqrt(k_wheel * m) / motion_ratio**2  # critical damping at the damper
    v_knee = DAMPER_KNEE * motion_ratio                     # damper speed at the knee

    # Spring curve: force = preload - curve(length - ride length). Stops are stiff and progressive.
    droop = DROOP_TRAVEL[axle] * motion_ratio
    bump = BUMP_TRAVEL[axle] * motion_ratio
    k_stop = 30 * k_spring
    spring_curve = [[-bump - 0.04, -k_spring * (bump + 0.04) - 3 * k_stop * 0.04],
                    [-bump - 0.02, -k_spring * (bump + 0.02) - k_stop * 0.02],
                    [-bump, -k_spring * bump],
                    [0, 0],
                    [droop, k_spring * droop],
                    [droop + 0.02, k_spring * droop + k_stop * 0.02],
                    [droop + 0.04, k_spring * droop + 3 * k_stop * 0.04]]

    # Damper curve: velocity [m/s] (extension positive) -> force [N]. Slope falls to 40% past the knee.
    def damper(zeta, v):
        c = zeta * c_crit
        return c * v if abs(v) <= v_knee else math.copysign(c * v_knee + 0.4 * c * (abs(v) - v_knee), v)

    vels = [-2.0, -v_knee, 0.0, v_knee, 2.0]
    damper_curve = [[v, round(damper(DAMPING_REBOUND[axle] if v > 0 else DAMPING_COMPRESSION[axle], v), 1)]
                    for v in vels]
    return preload, spring_curve, damper_curve


def wishbone(axle):
    """Double wishbone suspension JSON for axle 0 (front) or 1 (rear)."""
    preload, spring_curve, damper_curve = ride_elements(axle, motion_ratio)
    upright_com = [0.0, (y_lbj + y_ubj) / 2, (z_lbj + z_ubj) / 2]
    return {
        "Name": "proto " + ("front" if axle == 0 else "rear") + " double wishbone",
        "Type": "Suspension",
        "Template": "DoubleWishbone",
        "Camber Angle (deg)": 0,
        "Toe Angle (deg)": 0,
        "Spindle": {"Mass": SPINDLE_MASS, "COM": [0, Y_WHEEL, 0], "Inertia": [0.04, 0.07, 0.04],
                    "Radius": 0.1, "Width": 0.06},
        "Upright": {"Mass": UPRIGHT_MASS, "COM": upright_com, "Moments of Inertia": [0.2, 0.2, 0.05],
                    "Products of Inertia": [0, 0, 0], "Radius": 0.04},
        "Upper Control Arm": {"Mass": UPPER_ARM_MASS, "COM": [0, (y_uin + y_ubj) / 2, z_ubj],
                              "Moments of Inertia": rod_inertia(UPPER_ARM_MASS, UPPER_ARM_LENGTH),
                              "Products of Inertia": [0, 0, 0], "Radius": 0.02,
                              "Location Chassis Front": [0.15, y_uin, z_ubj],
                              "Location Chassis Back": [-0.15, y_uin, z_ubj],
                              "Location Upright": [0, y_ubj, z_ubj]},
        "Lower Control Arm": {"Mass": LOWER_ARM_MASS, "COM": [0, (y_lin + y_lbj) / 2, z_lbj],
                              "Moments of Inertia": rod_inertia(LOWER_ARM_MASS, LOWER_ARM_LENGTH),
                              "Products of Inertia": [0, 0, 0], "Radius": 0.03,
                              "Location Chassis Front": [0.2, y_lin, z_lbj],
                              "Location Chassis Back": [-0.2, y_lin, z_lbj],
                              "Location Upright": [0, y_lbj, z_lbj]},
        "Tierod": {"Location Chassis": tierod_in, "Location Upright": tierod_out},
        "Spring": {"Location Chassis": spring_top, "Location Arm": spring_bot,
                   "Free Length": spring_len, "Preload": preload, "Spring Curve Data": spring_curve},
        "Shock": {"Location Chassis": spring_top, "Location Arm": spring_bot,
                  "Damping Curve Data": damper_curve},
        "Axle": {"Inertia": 0.4},
    }


def solid_axle(axle):
    """Three-link solid axle JSON (Chrono SolidThreeLinkAxle) with vertical coilovers.

    A triangular link (A-arm) from the chassis to a ball joint on top of the axle center
    locates the axle sideways and fore-aft. Two lower links take the drive and brake torque.
    The axle can heave and roll (articulate). All links are horizontal at ride height.
    """
    coil_bot = [0.0, COIL_AXLE[0], COIL_AXLE[1]]
    coil_top = [0.0, COIL_AXLE[0], COIL_TOP_HEIGHT]
    # A vertical coilover on the axle moves one-to-one with its wheel in heave.
    preload, spring_curve, damper_curve = ride_elements(axle, 1.0)
    tube_len = 2 * Y_WHEEL
    return {
        "Name": "proto " + ("front" if axle == 0 else "rear") + " solid axle",
        "Type": "Suspension",
        "Template": "SolidThreeLinkAxle",
        "Camber Angle (deg)": 0,
        "Toe Angle (deg)": 0,
        "Spindle": {"Mass": SPINDLE_MASS, "COM": [0, Y_WHEEL, 0], "Inertia": [0.04, 0.07, 0.04],
                    "Radius": 0.1, "Width": 0.06},
        "Axle Tube": {"Mass": AXLE_TUBE_MASS, "COM": [0, 0, 0],
                      "Inertia": [AXLE_TUBE_MASS * tube_len**2 / 12, 0.5, AXLE_TUBE_MASS * tube_len**2 / 12],
                      "Radius": 0.05},
        "Triangular Link": {"Mass": TRIANGLE_MASS, "Inertia": [0.3, 0.3, 0.5],
                            "Location Chassis": [TRIANGLE_CHASSIS[0], TRIANGLE_CHASSIS[1], TRIANGLE_APEX_HEIGHT],
                            "Location Axle": [0, 0, TRIANGLE_APEX_HEIGHT]},
        "Longitudinal Link": {"Mass": LOWER_LINK_MASS, "Inertia": rod_inertia(LOWER_LINK_MASS, LOWER_LINK_LENGTH),
                              "Location Chassis": [LOWER_LINK_LENGTH, LOWER_LINK_AXLE[0], LOWER_LINK_AXLE[1]],
                              "Location Axle": [0, LOWER_LINK_AXLE[0], LOWER_LINK_AXLE[1]]},
        "Spring": {"Location Chassis": coil_top, "Location Axle": coil_bot,
                   "Free Length": COIL_TOP_HEIGHT - COIL_AXLE[1], "Preload": preload,
                   "Spring Curve Data": spring_curve},
        "Shock": {"Location Chassis": coil_top, "Location Axle": coil_bot, "Damping Curve Data": damper_curve},
        "Axle": {"Inertia": 0.4},
    }


def chassis():
    lx, ly, lz = INERTIA_BOX
    m = SPRUNG_MASS
    cg_z = CG_HEIGHT - TIRE_RADIUS  # chassis frame origin is at wheel center height
    return {
        "Name": "proto frame",
        "Type": "Chassis",
        "Template": "RigidChassis",
        "Components": [{
            "Centroidal Frame": {"Location": [CG_X, 0, cg_z], "Orientation": [1, 0, 0, 0]},
            "Mass": m,
            "Moments of Inertia": [m / 12 * (ly**2 + lz**2), m / 12 * (lx**2 + lz**2), m / 12 * (lx**2 + ly**2)],
            "Products of Inertia": [0, 0, 0],
            "Void": False,
        }],
        "Driver Position": {"Location": [0, 0.5, 1.2], "Orientation": [1, 0, 0, 0]},
        "Visualization": {"Primitives": [
            {"Type": "BOX", "Location": [0, 0, SPRING_TOP[1] + FRAME_SIZE[2] / 2],
             "Orientation": [1, 0, 0, 0], "Dimensions": list(FRAME_SIZE)},
            # Marks the front of the vehicle.
            {"Type": "BOX", "Location": [FRAME_SIZE[0] / 2 - 0.2, 0, SPRING_TOP[1] + FRAME_SIZE[2] + 0.1],
             "Orientation": [1, 0, 0, 0], "Dimensions": [0.4, FRAME_SIZE[1], 0.2]},
        ]},
    }


def vehicle():
    x_front, x_rear = WHEELBASE / 2, -WHEELBASE / 2

    def axle(x, susp, brake, steer):
        a = {"Suspension Input File": "proto/" + susp, "Suspension Location": [x, 0, 0],
             "Left Wheel Input File": "proto/wheel.json", "Right Wheel Input File": "proto/wheel.json",
             "Left Brake Input File": "proto/brake.json", "Right Brake Input File": "proto/brake.json"}
        if steer:
            a["Steering Index"] = 0
        return a

    return {
        "Name": "proto",
        "Type": "Vehicle",
        "Template": "WheeledVehicle",
        "Chassis": {"Input File": "proto/chassis.json"},
        "Axles": [axle(x_front, "suspension_front.json", "brake.json", True),
                  axle(x_rear, "suspension_rear.json", "brake.json", False)],
        "Steering Subsystems": [{"Input File": "proto/steering.json",
                                 "Location": [x_front + tierod_in[0], 0, tierod_in[2]],
                                 "Orientation": [1, 0, 0, 0]}],
        "Wheelbase": WHEELBASE,
        "Minimum Turning Radius": WHEELBASE / math.sin(math.radians(MAX_STEER_DEG)),
        "Maximum Steering Angle (deg)": MAX_STEER_DEG,
        "Driveline": {"Input File": "proto/driveline.json", "Suspension Indexes": [0, 1]},
    }


def steering():
    # The rack moves the tie rod inner points. Rack travel at full lock: steering arm * sin(max steer).
    rack_travel = STEERING_ARM * math.sin(math.radians(MAX_STEER_DEG))
    pinion_angle = math.radians(30)
    return {
        "Name": "proto rack and pinion",
        "Type": "Steering",
        "Template": "RackPinion",
        "Steering Link": {"Mass": 5.0, "COM": 0, "Inertia": [0.3, 0.01, 0.3], "Radius": 0.03,
                          "Length": 2 * tierod_in[1]},
        "Pinion": {"Radius": rack_travel / pinion_angle, "Maximum Angle (deg)": math.degrees(pinion_angle)},
    }


def tire():
    m, r, w = TIRE_MASS, TIRE_RADIUS, TIRE_WIDTH
    i_spin = m * r**2 * 0.7  # most of the tire mass is near the tread
    i_other = m * (3 * r**2 + w**2) / 12
    if TIRE_TYPE == "compliant":
        return {"Name": "proto compliant tire", "Type": "Tire", "Template": "CompliantTire",
                "Radius": r, "Width": w, "Mass": m, "Sidewall Stiffness": TIRE_STIFFNESS,
                "Sidewall Damping": TIRE_DAMPING, "Maximum Deflection": TIRE_MAX_DEFLECTION,
                "Stop Stiffness": 20 * TIRE_STIFFNESS}
    return {
        "Name": "proto rigid tire",
        "Type": "Tire",
        "Template": "RigidTire",
        "Radius": r, "Width": w, "Mass": m, "Inertia": [i_other, i_spin, i_other],
        "Contact Material": {"Coefficient of Friction": 0.9, "Coefficient of Restitution": 0.1,
                             "Properties": {"Young Modulus": 2e7, "Poisson Ratio": 0.3}},
    }


def main():
    os.makedirs(OUT, exist_ok=True)
    files = {
        "vehicle.json": vehicle(),
        "chassis.json": chassis(),
        "suspension_front.json": wishbone(0),
        "suspension_rear.json": solid_axle(1) if REAR_SUSPENSION == "solid" else wishbone(1),
        "steering.json": steering(),
        "tire.json": tire(),
        "wheel.json": {"Name": "proto wheel", "Type": "Wheel", "Template": "Wheel", "Mass": WHEEL_MASS,
                       "Inertia": [0.3, 0.5, 0.3],
                       "Visualization": {"Radius": TIRE_RADIUS * 0.6, "Width": TIRE_WIDTH * 1.05}},  # rim faces show beside the tire
        "engine.json": {"Name": "proto engine", "Type": "Engine", "Template": "EngineSimple",
                        "Maximum Engine Torque": ENGINE_TORQUE, "Maximum Engine Power": ENGINE_POWER,
                        "Maximum Engine Speed": GOVERNOR_RPM * math.pi / 30},  # Chrono reads rad/s
        "transmission.json": {"Name": "proto transmission (low range)", "Type": "Transmission",
                              "Template": "AutomaticTransmissionSimpleMap",
                              "Gear Box": {"Reverse Gear Ratio": REVERSE_RATIO / LOW_RANGE,
                                           "Forward Gear Ratios": [g / LOW_RANGE for g in GEAR_RATIOS],
                                           "Shift Points Map RPM": [list(SHIFT_RPM)] * len(GEAR_RATIOS)}},
        "brake.json": {"Name": "proto brake", "Type": "Brake", "Template": "BrakeSimple",
                       "Maximum Torque": BRAKE_TORQUE},
        "driveline.json": {"Name": "proto limited-slip 4WD", "Type": "Driveline", "Template": "SimpleDriveline",
                           "Front Torque Fraction": FRONT_TORQUE_FRACTION,
                           "Front Differential Max Bias": TORQUE_BIAS, "Rear Differential Max Bias": TORQUE_BIAS,
                           "Front Conical Gear Ratio": CONICAL_GEAR_RATIO,
                           "Rear Conical Gear Ratio": CONICAL_GEAR_RATIO},
    }
    for name, data in files.items():
        with open(os.path.join(OUT, name), "w") as f:
            json.dump(data, f, indent=2)
            f.write("\n")

    # Summary of the derived values.
    print(f"motion ratio {motion_ratio:.3f}, unsprung mass per corner {unsprung:.0f} kg")
    for axle, name in enumerate(["front", "rear"]):
        s = files["suspension_" + name + ".json"]["Spring"]
        k = s["Spring Curve Data"][4][1] / s["Spring Curve Data"][4][0]
        print(f"{name}: corner mass {corner_mass[axle]:.0f} kg, spring {k / 1e3:.1f} kN/m, "
              f"preload {s['Preload']:.0f} N, ride length {s['Free Length']:.3f} m")
    print(f"tie rod inner {tierod_in}, outer {tierod_out}")
    w_max = GOVERNOR_RPM * math.pi / 30 * GEAR_RATIOS[-1] / LOW_RANGE * CONICAL_GEAR_RATIO
    print(f"top speed {w_max * TIRE_RADIUS:.1f} m/s ({w_max * TIRE_RADIUS * 3.6:.0f} km/h)")


if __name__ == "__main__":
    main()
