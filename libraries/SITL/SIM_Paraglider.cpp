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

/* 
    Paraglider/paramotor dynamics based on:
    N. Umenberger and A. Goktogan,
    "Guidance, Navigation and Control of a Small-Scale Paramotor",
    Proc. Australasian Conference on Robotics and Automation (ACRA), 2012.
    https://www.araa.asn.au/acra/acra2012/papers/pap151.pdf

*/

#include "SIM_config.h"

#include "SIM_Paraglider.h"

#include <AP_Math/AP_Math.h>
#include <AP_Filesystem/AP_Filesystem_config.h>
#include <AP_Filesystem/AP_Filesystem.h>

using namespace SITL;

Paraglider::Paraglider(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = model.mass_kg;

    // Try to load JSON coefficient file if specified
    const char *colon = strchr(frame_str, ':');
    size_t slen = strlen(frame_str);
    if (colon != nullptr && slen > 5 && strcmp(&frame_str[slen-5], ".json") == 0) {
        load_coeffs(colon+1);
    }

    // Configure ground behavior and frame parameters
    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;
    lock_step_scheduled = true;
    frame_height = 0.1f;

    // Launch method support
    if (strstr(frame_str, "-throw")) {
        // Hand/hill throw launch - brief high acceleration to build initial airspeed
        have_launcher = true;
        launch_accel = 20;      // m/s^2
        launch_time = 0.5;      // 0.5 seconds
    }
    if (strstr(frame_str, "-tow")) {
        // Winch/vehicle tow launch - gentler acceleration over longer period
        have_launcher = true;
        launch_accel = 5;       // m/s^2
        launch_time = 30;       // 30 seconds to reach altitude
    }
}

Paraglider::ForceBreakdown Paraglider::compute_forces_bf(float brake_left_rad,
                                                         float brake_right_rad,
                                                         float throttle_norm)
{
    ForceBreakdown out{};

    // Use inherited air_density with fallback to model default
    const float rho = air_density;

    // Body-frame air-relative velocity at system CG (B)
    const Vector3f vB_bf = velocity_air_bf;
    const Vector3f omega_bf = gyro; // p,q,r (rad/s)

    // Velocity at fuselage mass centre (Eq. 9 form)
    const Vector3f vF_bf = vB_bf + (omega_bf % model.S_FB_B);

    // Fuselage AoA alpha_F = atan(w_F/u_F) (Eq. 11)
    const float uF = vF_bf.x;
    const float wF = vF_bf.z;
    const float alpha_F = atan2f(wF, MAX(0.01f, uF));
    const float CD_F = model.aero.CD0_F + model.aero.CDa_F * sq(alpha_F);

    // Fuselage drag force (Eq. 8): -0.5 rho A_F ||vF|| CD_F * vF
    const float VF = vF_bf.length();
    if (VF > 0.1f) {
        out.F_fuse_bf = vF_bf * (-0.5f * rho * model.A_fuse_m2 * VF * CD_F);
    }

    // Body->Parafoil transform T_BP (Eq. 14): pitch about Y by chi
    Matrix3f T_BP;
    T_BP.from_euler(0.0f, model.canopy_pitch_rad, 0.0f);

    // Velocity at parafoil mass centre, expressed in body, then parafoil (Eq. 13 form)
    const Vector3f vP_bf = vB_bf + (omega_bf % model.S_PB_B);
    const Vector3f vP_pf = T_BP * vP_bf;

    const float uP = vP_pf.x;
    const float vP = vP_pf.y;
    const float wP = vP_pf.z;
    const float VP = vP_pf.length();

    // Parafoil AoA (Eq. 15): alpha_P = atan(wP/uP)
    const float alpha_P = atan2f(wP, MAX(0.01f, uP));

    out.VP = VP;
    out.alpha_P = alpha_P;

    aoa_rad = alpha_P;
    tas_mps = VP;

    // CL/CD (Eq. 15)
    const float CL_P = model.aero.CL0_P + model.aero.CLa_P * alpha_P;
    const float CD_P = model.aero.CD0_P + model.aero.CDa_P * sq(alpha_P);

    // Parafoil aero force (Eq. 12) in parafoil axes
    Vector3f F_para_pf{};
    if (VP > 0.1f) {
        const Vector3f lift_vec_pf{wP, 0.0f, -uP};
        const Vector3f drag_vec_pf{uP, vP, wP};
        F_para_pf = (lift_vec_pf * CL_P - drag_vec_pf * CD_P) * (0.5f * rho * model.A_para_m2 * VP);
    }

    // Brakes: define symmetric/asymmetric deflections
    const float delta_s = 0.5f * (brake_left_rad + brake_right_rad);
    const float delta_a = 0.5f * (brake_left_rad - brake_right_rad);

    // Brake force (Eq. 20-21) in parafoil axes
    Vector3f F_brake_pf{};
    if (VP > 0.1f) {
        const float s = (delta_a >= 0.0f) ? 1.0f : -1.0f;

        const float CLda = model.aero.CL_da;
        const float CDda = model.aero.CD_da;

        const float ax = (CLda * wP - CDda * uP);
        const float ay = (-CDda * vP);
        const float az = (-CLda * uP - CDda * wP);

        const float Fx = ax * (s * delta_a + delta_s);
        const float Fy = ay * (s * delta_a + delta_s);
        const float Fz = az * (s * delta_a + delta_s);

        F_brake_pf = Vector3f{Fx, Fy, Fz} * (0.5f * rho * model.A_para_m2 * VP);
    }

    // Transform parafoil + brake forces back to body (Eq. 16)
    const Matrix3f T_PB = T_BP.transposed();
    out.F_para_bf = T_PB * F_para_pf;
    out.F_brake_bf = T_PB * F_brake_pf;

    // Thrust in +X body axis
    const float thrust_N = MAX(0.0f, MIN(1.0f, throttle_norm)) * model.thrust_max_N;
    out.F_thrust_bf = Vector3f{thrust_N, 0.0f, 0.0f};

    // Note: weight/gravity is NOT included here.
    // update_dynamics() adds gravity in earth frame.

    return out;
}

Vector3f Paraglider::compute_torque_bf(float brake_left_rad,
                                       float brake_right_rad,
                                       const ForceBreakdown &F)
{
    const float rho = air_density;

    const float VP = F.VP;
    const float alpha_P = F.alpha_P;

    const float p = gyro.x;
    const float q = gyro.y;
    const float r = gyro.z;

    float phi = 0.0f;
    dcm.to_euler(&phi, nullptr, nullptr);

    // Pure aero moments (Eq. 18), expressed in body frame
    Vector3f M_aero_bf{};
    if (VP > 0.1f) {
        const float qbar = 0.5f * rho * model.A_para_m2 * sq(VP);

        const float L_roll =
            model.aero.Clp * (sq(model.b_span_m) * p / (2.0f * VP)) +
            model.aero.Clphi * (model.b_span_m * phi);

        const float M_pitch =
            model.aero.Cmq * (sq(model.c_chord_m) * q / (2.0f * VP)) +
            model.aero.Cm0 * model.c_chord_m +
            model.aero.Cmalpha * (model.c_chord_m * alpha_P);

        const float N_yaw =
            model.aero.Cnr * (sq(model.b_span_m) * r / (2.0f * VP));

        M_aero_bf = Vector3f{L_roll, M_pitch, N_yaw} * qbar;
    }

    // Brake asymmetric moment (Eq. 22)
    const float delta_a = 0.5f * (brake_left_rad - brake_right_rad);
    Vector3f M_brake_bf{};
    if (VP > 0.1f) {
        const float qbar = 0.5f * rho * model.A_para_m2 * sq(VP);
        const float scale = (model.b_span_m / model.d_brake_m) * delta_a;
        M_brake_bf = Vector3f{
            model.aero.Cl_da * scale,
            0.0f,
            model.aero.Cn_da * scale
        } * qbar;
    }

    // Thrust torque about CG (Eq. 19): r_MB x F_thrust
    const Vector3f r_MB_bf = model.S_FB_B + model.S_MF_F;
    const Vector3f M_thrust_bf = r_MB_bf % F.F_thrust_bf;

    // Lever-arm moments from forces applied away from CG (Eq. 17 terms)
    const Vector3f M_fuse_arm_bf = model.S_FB_B % F.F_fuse_bf;
    const Vector3f M_para_arm_bf = model.S_PB_B % (F.F_para_bf + F.F_brake_bf);

    return M_aero_bf + M_brake_bf + M_thrust_bf + M_fuse_arm_bf + M_para_arm_bf;
}


Vector3f Paraglider::inertia_mul(const Vector3f &w) const
{
    // I = [ Ixx  0  Ixz
    //       0   Iyy 0
    //       Ixz 0  Izz ]
    return Vector3f{
        model.Ixx * w.x + model.Ixz * w.z,
        model.Iyy * w.y,
        model.Ixz * w.x + model.Izz * w.z
    };
}

Vector3f Paraglider::inertia_inv_mul(const Vector3f &t) const
{
    const float det = model.Ixx * model.Izz - model.Ixz * model.Ixz;
    if (fabsf(det) < 1.0e-8f) {
        // Fallback to diagonal if ill-conditioned
        return Vector3f{
            t.x / model.Ixx,
            t.y / model.Iyy,
            t.z / model.Izz
        };
    }

    const float inv_det = 1.0f / det;

    return Vector3f{
        ( model.Izz * t.x - model.Ixz * t.z) * inv_det,
        t.y / model.Iyy,
        (-model.Ixz * t.x + model.Ixx * t.z) * inv_det
    };
}

void Paraglider::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel)
{
    // Extract servo inputs using base class filtered methods for servo dynamics
    const float throttle = filtered_servo_range(input, 2);
    const float brake_left_cmd_rad  = filtered_servo_angle(input, 0) * model.brake_max_rad / 2.0f;
    const float brake_right_cmd_rad = filtered_servo_angle(input, 1) * model.brake_max_rad / 2.0f;

    const ForceBreakdown F = compute_forces_bf(brake_left_cmd_rad, brake_right_cmd_rad, throttle);

    Vector3f force_bf = F.F_fuse_bf + F.F_para_bf + F.F_brake_bf + F.F_thrust_bf;
    const Vector3f torque_bf = compute_torque_bf(brake_left_cmd_rad, brake_right_cmd_rad, F);

    // Launcher support - applies extra acceleration during launch phase
    if (have_launcher) {
        bool launch_triggered = input.servos[6] > 1700;
        if (launch_triggered) {
            uint64_t now = AP_HAL::millis64();
            if (launch_start_ms == 0) {
                launch_start_ms = now;
            }
            if (now - launch_start_ms < launch_time*1000) {
                // Apply launch acceleration in forward and slightly upward direction
                force_bf.x += mass * launch_accel;
                force_bf.z -= mass * launch_accel / 3;  // negative z is up
            }
        } else {
            // allow reset of launcher
            launch_start_ms = 0;
        }
    }

    // Non-gravitational acceleration in body frame.
    accel_body = force_bf / mass;

    // Rigid-body rotational dynamics with Ixz coupling
    const Vector3f omega = gyro;
    const Vector3f Iomega = inertia_mul(omega);
    const Vector3f gyro_term = omega % Iomega;
    const Vector3f net_tau = torque_bf - gyro_term;

    rot_accel = inertia_inv_mul(net_tau);

    // Simulate paramotor RPM for telemetry
    motor_mask |= (1U << 2);
    rpm[2] = constrain_float(throttle, 0.0f, 1.0f) * 8000;
}

void Paraglider::load_coeffs(const char *model_json)
{
#if AP_FILESYSTEM_FILE_READING_ENABLED
    char *fname = nullptr;
    struct stat st;
    if (AP::FS().stat(model_json, &st) == 0) {
        fname = strdup(model_json);
    } else {
        IGNORE_RETURN(asprintf(&fname, "@ROMFS/models/%s", model_json));
        if (AP::FS().stat(fname, &st) != 0) {
            AP_HAL::panic("%s failed to load", model_json);
        }
    }
    if (fname == nullptr) {
        AP_HAL::panic("%s failed to load", model_json);
    }

    AP_JSON::value *obj = AP_JSON::load_json(fname);
    if (obj == nullptr) {
        AP_HAL::panic("%s failed to load", fname);
    }

    enum class VarType {
        FLOAT,
    };

    struct json_search {
        const char *label;
        void *ptr;
        VarType t;
    };

    // Aero coefficients
    json_search aero_vars[] = {
#define AERO_FLOAT(s) { #s, &model.aero.s, VarType::FLOAT }
        AERO_FLOAT(CD0_F),
        AERO_FLOAT(CDa_F),
        AERO_FLOAT(CL0_P),
        AERO_FLOAT(CLa_P),
        AERO_FLOAT(CD0_P),
        AERO_FLOAT(CDa_P),
        AERO_FLOAT(Clp),
        AERO_FLOAT(Clphi),
        AERO_FLOAT(Cmq),
        AERO_FLOAT(Cm0),
        AERO_FLOAT(Cmalpha),
        AERO_FLOAT(Cnr),
        AERO_FLOAT(CL_da),
        AERO_FLOAT(CD_da),
        AERO_FLOAT(Cl_da),
        AERO_FLOAT(Cn_da),
    };

    auto aero_obj = obj->get("aero");
    if (!aero_obj.is<AP_JSON::null>()) {
        for (uint8_t i=0; i<ARRAY_SIZE(aero_vars); i++) {
            auto v = aero_obj.get(aero_vars[i].label);
            if (!v.is<AP_JSON::null>()) {
                if (!v.is<double>()) {
                    AP_HAL::panic("Bad json type for aero.%s", aero_vars[i].label);
                }
                *((float *)aero_vars[i].ptr) = v.get<double>();
            }
        }
    }

    // Physical parameters
    json_search phys_vars[] = {
#define PHYS_FLOAT(s) { #s, &model.s, VarType::FLOAT }
        PHYS_FLOAT(mass_kg),
        PHYS_FLOAT(Ixx),
        PHYS_FLOAT(Iyy),
        PHYS_FLOAT(Izz),
        PHYS_FLOAT(Ixz),
        PHYS_FLOAT(rho),
        PHYS_FLOAT(A_fuse_m2),
        PHYS_FLOAT(A_para_m2),
        PHYS_FLOAT(b_span_m),
        PHYS_FLOAT(c_chord_m),
        PHYS_FLOAT(d_brake_m),
        PHYS_FLOAT(thrust_max_N),
    };

    for (uint8_t i=0; i<ARRAY_SIZE(phys_vars); i++) {
        auto v = obj->get(phys_vars[i].label);
        if (!v.is<AP_JSON::null>()) {
            if (!v.is<double>()) {
                AP_HAL::panic("Bad json type for %s", phys_vars[i].label);
            }
            *((float *)phys_vars[i].ptr) = v.get<double>();
        }
    }

    // Update mass now that we've potentially loaded a new value
    mass = model.mass_kg;

    delete obj;

    ::printf("Loaded paraglider aero coefficients from %s\n", fname);
    free(fname);
#endif
}

/*
  update the paraglider simulation by one time step
 */
void Paraglider::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    // Update wind effects
    update_wind(input);

    // Calculate forces and moments
    calculate_forces(input, rot_accel);

    // Update attitude and position using rigid-body dynamics
    update_dynamics(rot_accel);

    // Update external payload effects if any
    update_external_payload(input);

    // Update lat/lon/altitude from position
    update_position();

    // Advance time step
    time_advance();

    // Update magnetic field in body frame
    update_mag_field_bf();
}
