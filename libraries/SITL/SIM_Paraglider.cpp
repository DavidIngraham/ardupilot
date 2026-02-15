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

#include "SIM_config.h"
#include "SIM_Paraglider.h"

#include <AP_Math/AP_Math.h>
#include <AP_Filesystem/AP_Filesystem_config.h>
#include <AP_Filesystem/AP_Filesystem.h>

using namespace SITL;

static inline float clamp_preserve_sign(float x, float eps)
{
    if (fabsf(x) >= eps) {
        return x;
    }
    return (x >= 0.0f) ? eps : -eps;
}

static inline float smoothstep(float t)
{
    t = constrain_float(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

static inline float lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

// Explicit rotation about +Y, right-handed, for x-forward/y-right/z-down coordinates.
// This is an active rotation matrix that maps vector components in the original frame
// into the rotated frame.
static Matrix3f rot_y(float angle_rad)
{
    const float c = cosf(angle_rad);
    const float s = sinf(angle_rad);

    Matrix3f R;
    // rows (a,b,c) for Matrix3f in ArduPilot
    R.a = Vector3f{ c, 0.0f, s };
    R.b = Vector3f{ 0.0f, 1.0f, 0.0f };
    R.c = Vector3f{ -s, 0.0f, c };
    return R;
}


Paraglider::Paraglider(const char *frame_str) : Aircraft(frame_str)
{
    mass = model.mass_kg;

    // Try to load JSON coefficient file if specified
    const char *colon = strchr(frame_str, ':');
    const size_t slen = strlen(frame_str);
    if (colon != nullptr && slen > 5 && strcmp(&frame_str[slen - 5], ".json") == 0) {
        load_coeffs(colon + 1);
    }

    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;
    lock_step_scheduled = true;
    frame_height = 0.1f;

    if (strstr(frame_str, "-throw")) {
        have_launcher = true;
        launch_accel = 10.0f;
        launch_time = 2.0f;
    }
    if (strstr(frame_str, "-tow")) {
        have_launcher = true;
        launch_accel = 0.5f;
        launch_time = 30.0f;
    }
}

// Evaluate CL/CD with a simple blended stall model.
// - raw alpha is for reporting
// - alpha_eff is used for linear terms and moments (prevents blow-ups)
// - CL transitions toward sin(2a) behavior
// - CD transitions toward sin^2(a) up to CD_90
void Paraglider::eval_parafoil_coeffs(float alpha_rad,
                                      float &CL_out,
                                      float &CD_out,
                                      float &alpha_eff_out) const
{
    const float a_abs = fabsf(alpha_rad);

    const float a_stall = MAX(0.01f, model.stall.alpha_stall_rad);
    const float a_full  = MAX(a_stall + 0.01f, model.stall.alpha_full_rad);

    alpha_eff_out = constrain_float(alpha_rad, -a_stall, +a_stall);

    // Capped linear at stall boundary
    const float CL_lin_cap = model.aero.CL0_P + model.aero.CLa_P * alpha_eff_out;
    const float CD_lin_cap = model.aero.CD0_P + model.aero.CDa_P * sq(alpha_eff_out);

    // Pure linear region
    if (a_abs <= a_stall) {
        CL_out = model.aero.CL0_P + model.aero.CLa_P * alpha_rad;
        CD_out = model.aero.CD0_P + model.aero.CDa_P * sq(alpha_rad);
        return;
    }

    // Blend from stall -> fully stalled
    float t = (a_abs - a_stall) / (a_full - a_stall);
    t = smoothstep(t);

    // Flat-plate-ish lift shape: sin(2a), scaled to match CL_lin_cap at stall
    const float denom = MAX(0.05f, fabsf(sinf(2.0f * alpha_eff_out)));
    const float CL_fp = CL_lin_cap * (sinf(2.0f * alpha_rad) / denom);

    // Flat-plate-ish drag shape: sin^2(a) to CD_90, continuous at stall
    const float s_stall = sinf(a_stall);
    const float s_alpha = sinf(a_abs);

    const float CD90 = MAX(CD_lin_cap, model.stall.CD_90);
    const float s2_stall = sq(s_stall);
    const float s2_alpha = sq(s_alpha);
    const float denom_cd = MAX(1.0e-3f, 1.0f - s2_stall);

    float CD_fp = CD_lin_cap + (CD90 - CD_lin_cap) * ((s2_alpha - s2_stall) / denom_cd);
    CD_fp = MAX(0.0f, CD_fp);

    CL_out = lerp(CL_lin_cap, CL_fp, t);
    CD_out = lerp(CD_lin_cap, CD_fp, t);
}

Paraglider::ForceBreakdown Paraglider::compute_forces_bf(float brake_left_rad,
                                                         float brake_right_rad,
                                                         float throttle_norm)
{
    ForceBreakdown out{};

    // Air-relative velocity at system CG (body frame)
    const Vector3f vB_bf = velocity_air_bf;
    const Vector3f omega_bf = gyro;

    // ---------- Fuselage drag (computed in body, applied opposite local velocity vector) ----------
    const Vector3f vF_bf = vB_bf + (omega_bf % model.S_FB_B);

    const float uF = clamp_preserve_sign(vF_bf.x, 0.01f);
    const float wF = vF_bf.z;

    // Stevens/Lewis/Johnson convention with z-down: alpha = atan2(-w, u)
    const float alpha_F = atan2f(-wF, uF);
    const float CD_F = model.aero.CD0_F + model.aero.CDa_F * sq(alpha_F);

    const float VF = vF_bf.length();
    if (VF > 0.1f) {
        // drag vector is opposite velocity (no lift on fuselage here)
        const Vector3f vhat = vF_bf * (1.0f / VF);
        const float qbar = 0.5f * air_density * sq(VF);
        const float D = qbar * model.A_fuse_m2 * CD_F;
        out.F_fuse_bf = vhat * (-D);
    }

    // ---------- Body <-> Parafoil frame ----------
    // We want a positive canopy_pitch_rad to correspond to a positive AoA in parafoil frame
    // for a forward body velocity. Using the convention alpha = atan2(-w, u),
    // mapping body->parafoil with +chi gives w<0 for u>0, so alpha>0.
    const Matrix3f R_pf_b = rot_y(model.canopy_pitch_rad);   // body -> parafoil
    const Matrix3f R_b_pf = R_pf_b.transposed();             // parafoil -> body

    // Velocity at parafoil mass center, expressed in parafoil frame
    const Vector3f vP_bf = vB_bf + (omega_bf % model.S_PB_B);
    const Vector3f vP_pf = R_pf_b * vP_bf;

    const float uP = vP_pf.x;
    const float vP = vP_pf.y;
    const float wP = vP_pf.z;
    const float V = vP_pf.length();

    out.V_pf = V;

    // AoA / sideslip (Stevens/Lewis/Johnson style, with z-down)
    if (V > 0.1f) {
        const float u_safe = clamp_preserve_sign(uP, 0.01f);
        out.alpha_pf_rad = atan2f(-wP, u_safe);
        out.beta_pf_rad = atan2f(vP, sqrtf(sq(uP) + sq(wP)));
    } else {
        out.alpha_pf_rad = 0.0f;
        out.beta_pf_rad = 0.0f;
    }

    aoa_rad = out.alpha_pf_rad;
    beta_rad = out.beta_pf_rad;
    tas_mps = V;

    // ---------- Parafoil aerodynamics in wind axes ----------
    Vector3f F_para_pf{};
    if (V > 0.1f) {
        float CL = 0.0f;
        float CD = 0.0f;
        float alpha_eff = 0.0f;

        eval_parafoil_coeffs(out.alpha_pf_rad, CL, CD, alpha_eff);
        out.alpha_eff_rad = alpha_eff;

        const float qbar = 0.5f * air_density * sq(V);
        const float L = qbar * model.A_para_m2 * CL;
        const float D = qbar * model.A_para_m2 * CD;

        // Wind axes force: x along velocity (same direction as vP_pf),
        // z down in the plane defined by velocity and parafoil +z (down).
        // Aerodynamic force in wind axes: [-D, 0, -L]
        const Vector3f F_w{-D, 0.0f, -L};

        // Build wind->parafoil DCM from velocity direction (robust, avoids rotation-order pitfalls)
        Vector3f x_w = vP_pf * (1.0f / V);             // wind x axis in parafoil coords
        Vector3f z_ref{0.0f, 0.0f, 1.0f};              // parafoil down axis
        Vector3f z_w = z_ref - x_w * (z_ref * x_w);    // component of down orthogonal to x_w

        const float z_w_len = z_w.length();
        if (z_w_len < 1.0e-3f) {
            // Degenerate near-vertical flight: choose an alternate reference
            Vector3f y_ref{0.0f, 1.0f, 0.0f};
            z_w = y_ref - x_w * (y_ref * x_w);
        }

        z_w = z_w * (1.0f / MAX(1.0e-3f, z_w.length()));
        Vector3f y_w = z_w % x_w;                      // right-handed: y = z x x
        y_w = y_w * (1.0f / MAX(1.0e-3f, y_w.length()));
        z_w = x_w % y_w;                               // re-orthonormalize

        // Matrix with columns {x_w, y_w, z_w}, represented as row vectors for Matrix3f
        Matrix3f R_pf_w;
        R_pf_w.a = Vector3f{x_w.x, y_w.x, z_w.x};
        R_pf_w.b = Vector3f{x_w.y, y_w.y, z_w.y};
        R_pf_w.c = Vector3f{x_w.z, y_w.z, z_w.z};

        // Wind -> parafoil
        F_para_pf = R_pf_w * F_w;
    }

    // ---------- Brakes (kept in parafoil axes, but with no sign-breaking clamps) ----------

    Vector3f F_brake_pf{};
    if (V > 0.1f) {
        const float k = MAX(brake_left_rad, brake_right_rad);

        const float ax = (model.aero.CL_da * wP - model.aero.CD_da * uP);
        const float ay = (-model.aero.CD_da * vP);
        const float az = (-model.aero.CL_da * uP - model.aero.CD_da * wP);

        const float qbar = 0.5f * air_density * sq(V);
        F_brake_pf = Vector3f{ax * k, ay * k, az * k} * (qbar * model.A_para_m2);
    }

    // ---------- Transform parafoil forces to body ----------
    out.F_para_bf  = R_b_pf * F_para_pf;
    out.F_brake_bf = R_b_pf * F_brake_pf;

    // ---------- Thrust in +X body axis ----------
    throttle_norm = constrain_float(throttle_norm, 0.0f, 1.0f);
    const float thrust_N = throttle_norm * model.thrust_max_N;
    out.F_thrust_bf = Vector3f{thrust_N, 0.0f, 0.0f};

    // Debug (optional): prints with enough precision to see small brake forces
    #if 0
    static uint32_t last_ms;
    if (AP_HAL::millis() - last_ms > 1000) {
        last_ms = AP_HAL::millis();
        ::printf("V=%.2f a=%.2fdeg b=%.2fdeg a_eff=%.2fdeg "
                 "Fpara_z=%.2f Fbrk_z=%.2f brkL=%.3f brkR=%.3f alt=%.2f\n",
                 out.V_pf,
                 degrees(out.alpha_pf_rad),
                 degrees(out.beta_pf_rad),
                 degrees(out.alpha_eff_rad),
                 out.F_para_bf.z,
                 out.F_brake_bf.z,
                 brake_left_rad,
                 brake_right_rad,
                 position.z);
    }
    #endif

    return out;
}

Vector3f Paraglider::compute_torque_bf(float brake_left_rad,
                                       float brake_right_rad,
                                       const ForceBreakdown &F)
{
    const float V = F.V_pf;

    const float p = gyro.x;
    const float q = gyro.y;
    const float r = gyro.z;

    float phi = 0.0f;
    dcm.to_euler(&phi, nullptr, nullptr);

    // Use stall-limited alpha for moment evaluation (prevents deep-stall blowups)
    const float alpha_m = F.alpha_eff_rad;

    Vector3f M_aero_bf{};
    if (V > 0.1f) {
        const float qbarS = 0.5f * air_density * model.A_para_m2 * sq(V);

        const float L_roll =
            model.aero.Clp * (sq(model.b_span_m) * p / (2.0f * V)) +
            model.aero.Clphi * (model.b_span_m * phi);

        const float M_pitch =
            model.aero.Cmq * (sq(model.c_chord_m) * q / (2.0f * V)) +
            model.aero.Cm0 * model.c_chord_m +
            model.aero.Cmalpha * (model.c_chord_m * alpha_m);

        const float N_yaw =
            model.aero.Cnr * (sq(model.b_span_m) * r / (2.0f * V));

        M_aero_bf = Vector3f{L_roll, M_pitch, N_yaw} * qbarS;
    }

    // Brake asymmetric moment (Eq. 22-style)
    const float delta_a = 0.5f * (brake_right_rad - brake_left_rad);
    Vector3f M_brake_bf{};
    if (V > 0.1f) {
        const float qbarS = 0.5f * air_density * model.A_para_m2 * sq(V);
        const float scale = (model.b_span_m / model.d_brake_m) * delta_a;
        M_brake_bf = Vector3f{
            model.aero.Cl_da * scale,
            0.0f,
            model.aero.Cn_da * scale
        } * qbarS;
    }

    // Thrust torque about CG: r_MB x F_thrust
    const Vector3f r_MB_bf = model.S_FB_B + model.S_MF_F;
    const Vector3f M_thrust_bf = r_MB_bf % F.F_thrust_bf;

    // Lever arms from forces away from CG
    const Vector3f M_fuse_arm_bf = model.S_FB_B % F.F_fuse_bf;
    const Vector3f M_para_arm_bf = model.S_PB_B % (F.F_para_bf + F.F_brake_bf);

    // Roll damping
    const Vector3f M_roll_damp_bf{-model.roll_damp_Nm_per_rps * p, 0.0f, 0.0f};


    return M_aero_bf + M_brake_bf + M_thrust_bf + M_fuse_arm_bf + M_para_arm_bf + M_roll_damp_bf;
}

Vector3f Paraglider::inertia_mul(const Vector3f &w) const
{
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
    const float throttle = constrain_float(filtered_servo_range(input, 2), 0.0f, 1.0f);
    const float brake_left_rad =
        constrain_float(filtered_servo_range(input, 0), 0.0f, 1.0f) * model.brake_max_rad;
    const float brake_right_rad =
        constrain_float(filtered_servo_range(input, 1), 0.0f, 1.0f) * model.brake_max_rad;

    const ForceBreakdown F = compute_forces_bf(brake_left_rad, brake_right_rad, throttle);

    Vector3f force_bf = F.F_fuse_bf + F.F_para_bf + F.F_brake_bf + F.F_thrust_bf;
    const Vector3f torque_bf = compute_torque_bf(brake_left_rad, brake_right_rad, F);

    if (have_launcher) {
        const bool launch_triggered = input.servos[6] > 1700;
        if (launch_triggered) {
            const uint64_t now = AP_HAL::millis64();
            if (launch_start_ms == 0) {
                launch_start_ms = now;
            }
            const uint64_t launch_ms = uint64_t(launch_time * 1000.0f);
            if (now - launch_start_ms < launch_ms) {
                force_bf.x += mass * launch_accel;
                force_bf.z -= mass * launch_accel / 3.0f; // negative z is up
            }
        } else {
            launch_start_ms = 0;
        }
    }

    accel_body = force_bf / mass;

    // Rotational dynamics with Ixz coupling
    const Vector3f omega = gyro;
    const Vector3f Iomega = inertia_mul(omega);
    const Vector3f net_tau = torque_bf - (omega % Iomega);

    rot_accel = inertia_inv_mul(net_tau);

    motor_mask |= (1U << 2);
    rpm[2] = throttle * 8000.0f;
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

    // Aero coefficients
    auto aero_obj = obj->get("aero");
    if (!aero_obj.is<AP_JSON::null>()) {
#define LOAD_AERO_FLOAT(field) do { \
        auto v = aero_obj.get(#field); \
        if (!v.is<AP_JSON::null>()) { \
            if (!v.is<double>()) { AP_HAL::panic("Bad json type for aero.%s", #field); } \
            model.aero.field = v.get<double>(); \
        } \
    } while (0)

        LOAD_AERO_FLOAT(CD0_F);
        LOAD_AERO_FLOAT(CDa_F);
        LOAD_AERO_FLOAT(CL0_P);
        LOAD_AERO_FLOAT(CLa_P);
        LOAD_AERO_FLOAT(CD0_P);
        LOAD_AERO_FLOAT(CDa_P);
        LOAD_AERO_FLOAT(Clp);
        LOAD_AERO_FLOAT(Clphi);
        LOAD_AERO_FLOAT(Cmq);
        LOAD_AERO_FLOAT(Cm0);
        LOAD_AERO_FLOAT(Cmalpha);
        LOAD_AERO_FLOAT(Cnr);
        LOAD_AERO_FLOAT(CL_da);
        LOAD_AERO_FLOAT(CD_da);
        LOAD_AERO_FLOAT(Cl_da);
        LOAD_AERO_FLOAT(Cn_da);
#undef LOAD_AERO_FLOAT
    }

    // Stall parameters (optional)
    auto stall_obj = obj->get("stall");
    if (!stall_obj.is<AP_JSON::null>()) {
#define LOAD_STALL_FLOAT(field) do { \
        auto v = stall_obj.get(#field); \
        if (!v.is<AP_JSON::null>()) { \
            if (!v.is<double>()) { AP_HAL::panic("Bad json type for stall.%s", #field); } \
            model.stall.field = v.get<double>(); \
        } \
    } while (0)

        LOAD_STALL_FLOAT(alpha_stall_rad);
        LOAD_STALL_FLOAT(alpha_full_rad);
        LOAD_STALL_FLOAT(CD_90);
#undef LOAD_STALL_FLOAT
    }

    // Physical parameters
#define LOAD_PHYS_FLOAT(field) do { \
    auto v = obj->get(#field); \
    if (!v.is<AP_JSON::null>()) { \
        if (!v.is<double>()) { AP_HAL::panic("Bad json type for %s", #field); } \
        model.field = v.get<double>(); \
    } \
} while (0)

    LOAD_PHYS_FLOAT(mass_kg);
    LOAD_PHYS_FLOAT(Ixx);
    LOAD_PHYS_FLOAT(Iyy);
    LOAD_PHYS_FLOAT(Izz);
    LOAD_PHYS_FLOAT(Ixz);
    LOAD_PHYS_FLOAT(A_fuse_m2);
    LOAD_PHYS_FLOAT(A_para_m2);
    LOAD_PHYS_FLOAT(b_span_m);
    LOAD_PHYS_FLOAT(c_chord_m);
    LOAD_PHYS_FLOAT(d_brake_m);
    LOAD_PHYS_FLOAT(thrust_max_N);
    LOAD_PHYS_FLOAT(brake_max_rad);
    LOAD_PHYS_FLOAT(canopy_pitch_rad);
    LOAD_PHYS_FLOAT(roll_damp_Nm_per_rps);
#undef LOAD_PHYS_FLOAT

    mass = model.mass_kg;

    delete obj;

    ::printf("Loaded paraglider coefficients from %s\n", fname);
    free(fname);
#endif
}

void Paraglider::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    update_wind(input);
    calculate_forces(input, rot_accel);
    update_dynamics(rot_accel);
    update_external_payload(input);
    update_position();
    time_advance();
    update_mag_field_bf();
}
