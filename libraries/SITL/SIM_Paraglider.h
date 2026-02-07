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
    void load_coeffs(const char *model_json);
    struct AeroCoeffs {
        // Fuselage drag coefficient: C_D^F = CD0_F + CDa_F * alpha_F^2
        float CD0_F = 0.15f;
        float CDa_F = 1.0f;

        // Parafoil coefficients (Eq. 15): CL^P = CL0_P + CLa_P * alpha_P
        //                           CD^P = CD0_P + CDa_P * alpha_P^2
        float CL0_P = 0.4f;
        float CLa_P = 2.0f;
        float CD0_P = 0.15f;
        float CDa_P = 1.0f;

        // Pure aero moments (Eq. 18)
        float Clp     = -0.1f;
        float Clphi   = -0.05f;
        float Cmq     = -2.0f;
        float Cm0     = 0.018f;
        float Cmalpha = -0.2f;

        // Eq. 18 includes a yaw-rate damping term Cnr; default 0 unless tuned
        float Cnr = 0.0f;

        // Brake force/moment coefficients (Eq. 21, 22)
        float CL_da = 0.0021f;
        float CD_da = 0.0001f;
        float Cl_da = 0.0001f;
        float Cn_da = 0.004f;
    };

    struct Model {
        float mass_kg = 1.55f;

        // Table 1 inertia tensor in body frame (kg m^2).
        float Ixx = 0.336f;
        float Iyy = 0.292f;
        float Izz = 0.109f;
        float Ixz = -0.059f;

        // Environment (sea-level fallback)
        float rho = 1.225f;

        // Areas
        float A_fuse_m2 = 0.5f;
        float A_para_m2 = 1.16f;

        // Geometry
        float b_span_m = 2.15f;
        float c_chord_m = 0.54f;
        float d_brake_m = 0.40f;

        // Actuator Limits
        float thrust_max_N = 10.0f;
        float brake_max_rad = radians(45);

        // Canopy relative pitch (chi) (deg->rad)
        float canopy_pitch_rad = radians(20.0f);

        // Offsets in body frame from system CG B to points (meters)
        Vector3f S_FB_B{0.037f, 0.0f, 0.149f};     // fuselage mass centre F wrt B
        Vector3f S_PB_B{-0.266f, 0.0f, -1.066f};   // parafoil mass centre P wrt B

        // Motor offset in fuselage frame (assume F aligned with B for SITL)
        Vector3f S_MF_F{0.0f, 0.0f, -0.012f};

        AeroCoeffs aero{};
    } model;

    struct ForceBreakdown {
        Vector3f F_fuse_bf{};
        Vector3f F_para_bf{};
        Vector3f F_brake_bf{};
        Vector3f F_thrust_bf{};
        float VP = 0.0f;         // parafoil relative airspeed magnitude (m/s)
        float alpha_P = 0.0f;    // parafoil AoA (rad)
    };

    // Cached aero state (for telemetry/debug)
    float aoa_rad = 0.0f;
    float beta_rad = 0.0f;
    float tas_mps = 0.0f;

    // Launcher configuration and state
    bool have_launcher = false;
    float launch_accel = 0.0f;      // m/s^2
    float launch_time = 0.0f;       // seconds
    uint64_t launch_start_ms = 0;

    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel);

    ForceBreakdown compute_forces_bf(float brake_left_rad,
                                     float brake_right_rad,
                                     float throttle_norm);

    Vector3f compute_torque_bf(float brake_left_rad,
                               float brake_right_rad,
                               const ForceBreakdown &F);

    // Inertia multiplication methods handle non-diagonal inertia tensor (Ixz coupling).
    // This is necessary for paragliders due to their asymmetric mass distribution.
    // The inertia tensor has the form:
    //   I = [ Ixx   0  Ixz
    //         0   Iyy   0
    //         Ixz   0  Izz ]
    // Standard aircraft often use diagonal-only approximation, but paragliders with
    // offset fuselage/canopy centers require the coupling term for accurate dynamics.
    Vector3f inertia_mul(const Vector3f &w) const;
    Vector3f inertia_inv_mul(const Vector3f &t) const;
};

} // namespace SITL
