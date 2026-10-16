/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*  Paraglider/paramotor dynamics loosely based on:
    N. Umenberger and A. Goktogan,
    "Guidance, Navigation and Control of a Small-Scale Paramotor",
    Proc. Australasian Conference on Robotics and Automation (ACRA), 2012.
    https://www.araa.asn.au/acra/acra2012/papers/pap151.pdf

    Brake force and moment conventions follow equations 20-22. Aerodynamic
    coefficients are dimensionless; brake derivatives are per radian.
    An optional reduced-order pitch joint separates canopy and payload.
    Roll and yaw remain shared; this is not a full multibody canopy model.
*/

#pragma once

#include "SIM_config.h"
#include "SIM_Aircraft.h"

#include <AP_JSON/AP_JSON.h>


namespace SITL {

class Paraglider : public Aircraft {
public:
    explicit Paraglider(const char *frame_str);

    static Aircraft *create(const char *frame_str) { return NEW_NOTHROW Paraglider(frame_str); }

    void update(const struct sitl_input &input) override;

private:
    friend class ParagliderTest;
    void load_coeffs(const char *model_json);

    struct AeroCoeffs {
        // Fuselage drag coefficient
        float CD0_F = 0.15f;
        float CDa_F = 1.0f;

        // Parafoil baseline CL/CD
        float CL0_P = 0.4f;
        float CLa_P = 2.0f;
        float CD0_P = 0.15f;
        float CDa_P = 1.0f;

        // Moments (Eq. 18-style)
        float Clp     = -0.1f;
        float Clphi   = -0.05f;
        float Cmq     = -2.0f;
        float Cm0     = 0.018f;
        float Cmalpha = -0.2f;
        // Provisional effective damping derivative against r*b/(2*V).
        float Cnr     = -0.05f;

        // Dimensionless lift/drag derivatives per radian of brake deflection.
        // Forces use the maximum left/right deflection, following Eq. 20-21.
        float CL_da = 0.0021f;
        float CD_da = 0.0001f;

        // Dimensionless roll/yaw derivatives per radian (Eq. 22).
        float Cl_da = 0.0001f;
        float Cn_da = 0.004f;
    };

    struct StallModel {
        // Where linear aerodynamics begin to break down
        float alpha_stall_rad = radians(15.0f);

        // Fully-developed stall / flat-plate-ish blending target
        float alpha_full_rad  = radians(75.0f);

        // Drag at 90deg AoA for the blended model
        float CD_90 = 1.6f;
    };

    struct Model {
        float mass_kg = 1.55f;

        // Inertia tensor in body frame (kg m^2), with Ixz coupling
        float Ixx = 0.336f;
        float Iyy = 0.292f;
        float Izz = 0.109f;
        float Ixz = -0.059f;

        // Areas
        float A_fuse_m2 = 0.5f;
        float A_para_m2 = 1.16f;

        // Geometry
        float b_span_m  = 2.15f;
        float c_chord_m = 0.54f;
        float d_brake_m = 0.40f;

        // Actuator limits
        float thrust_max_N = 10.0f;

        // Signed body-X reaction torque / thrust (Nm/N = m).
        // Negative corresponds to propeller rotation about positive body X.
        float prop_torque_per_thrust_m = -0.02f;
        float brake_max_rad = radians(45.0f);

        // Canopy pitch relative to body (rad).
        // Positive = canopy pitched "nose-up" relative to body.
        float canopy_pitch_rad = radians(20.0f);

        // Offsets in body frame from system CG B to points (meters)
        Vector3f S_FB_B{0.037f, 0.0f, 0.149f};     // fuselage mass centre F wrt B
        Vector3f S_PB_B{-0.266f, 0.0f, -1.066f};   // parafoil mass centre P wrt B

        // Motor offset in fuselage frame (assume F aligned with B)
        Vector3f S_MF_F{0.0f, 0.0f, -0.012f};

        // Baseline structural/harness roll damping (Nm per rad/s)
        // Adds torque: Mx += -roll_damp_Nm_per_rps * p
        float roll_damp_Nm_per_rps = 0.6f;

        // Optional pitch articulation. Zero retains the original rigid model.
        // Provisional small-scale geometry, not identified from flight data.
        float pitch_joint_enabled = 0;
        float pitch_joint_locked = 0; // matched-geometry rigid control case
        float canopy_mass_kg = 0.19f;
        float payload_pitch_inertia = 0.03f; // about payload CG, kg m^2
        float canopy_pitch_inertia = 0.025f; // about canopy CG, kg m^2
        float payload_hinge_z_m = 0.35f; // hinge -> payload CG, down positive
        float canopy_hinge_x_m = -0.30f;
        float canopy_hinge_z_m = -0.85f;
        float thrust_payload_z_m = -0.10f; // payload CG -> thrust line
        float pitch_joint_damping = 0.015f; // Nm/(rad/s), equal/opposite
        float pitch_joint_stiffness = 0; // Nm/rad; free pin by default

        AeroCoeffs aero{};
        StallModel stall{};
    } model;

    struct ForceBreakdown {
        Vector3f F_fuse_bf{};
        Vector3f F_para_bf{};
        Vector3f F_brake_bf{};
        Vector3f F_thrust_bf{};

        float V_pf = 0.0f;
        float alpha_pf_rad = 0.0f;      // raw AoA (positive = lifting convention)
        float beta_pf_rad = 0.0f;       // sideslip
        float alpha_eff_rad = 0.0f;     // stall-limited alpha used for coeffs/moments
    };

    // Cached aero state (telemetry/debug)
    float aoa_rad = 0.0f;
    float beta_rad = 0.0f;
    float tas_mps = 0.0f;

    // Launcher configuration/state
    bool have_launcher = false;
    bool launch_guide_attitude = false; // synthetic throw guide, released after launch_time
    float launch_accel = 0.0f;     // m/s^2
    float launch_time = 0.0f;      // seconds
    uint64_t launch_start_ms = 0;
    bool launch_started = false;
    bool launch_released = false;

    // Canopy physical pitch relative to payload, excluding rigging incidence.
    float joint_pitch_rad = 0;
    float joint_pitch_rate = 0;
    float joint_pitch_accel = 0;
    Vector3f payload_accel_offset{};
    uint64_t joint_log_us = 0;

    bool pitch_joint_enabled() const { return model.pitch_joint_enabled > 0.5f; }
    void pitch_geometry(Vector3f &payload_arm, Vector3f &canopy_arm) const;
    void pitch_accelerations(const ForceBreakdown &F, float &payload_accel, float &canopy_accel) const;
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel);

    ForceBreakdown compute_forces_bf(float brake_left_rad,
                                     float brake_right_rad,
                                     float throttle_norm);

    Vector3f compute_torque_bf(float brake_left_rad,
                               float brake_right_rad,
                               const ForceBreakdown &F);

    Vector3f inertia_mul(const Vector3f &w) const;
    Vector3f inertia_inv_mul(const Vector3f &t) const;

    void eval_parafoil_coeffs(float alpha_rad,
                              float &CL_out,
                              float &CD_out,
                              float &alpha_eff_out) const;

};

} // namespace SITL
