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
#include <AP_Logger/AP_Logger.h>
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
        launch_guide_attitude = true;
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
    const float denom = clamp_preserve_sign(sinf(2.0f * alpha_eff_out), 0.05f);
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

    // Fuselage drag (computed in body, applied opposite local velocity vector)
    Vector3f rF = model.S_FB_B;
    Vector3f rP = model.S_PB_B;
    Vector3f relative_velocity{};
    if (pitch_joint_enabled()) {
        pitch_geometry(rF, rP);
        const Vector3f b = rot_y(joint_pitch_rad) * Vector3f{model.canopy_hinge_x_m, 0, model.canopy_hinge_z_m};
        relative_velocity = Vector3f{0, joint_pitch_rate, 0} % b;
    }
    const float canopy_fraction = model.canopy_mass_kg / model.mass_kg;
    const Vector3f vF_bf = vB_bf + (omega_bf % rF) - relative_velocity * canopy_fraction;

    const float uF = clamp_preserve_sign(vF_bf.x, 0.01f);
    const float wF = vF_bf.z;

    // Body axes are x-forward, z-down: positive w gives positive AoA.
    const float alpha_F = atan2f(wF, uF);
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
    // A positive incidence or joint angle pitches the canopy nose-up.
    // Transform body velocity into canopy axes with the inverse rotation;
    // positive canopy-frame w then gives positive angle of attack.
    const float canopy_pitch = model.canopy_pitch_rad + (pitch_joint_enabled() ? joint_pitch_rad : 0);
    const Matrix3f R_pf_b = rot_y(-canopy_pitch);            // body -> parafoil
    const Matrix3f R_b_pf = R_pf_b.transposed();             // parafoil -> body

    // Velocity at parafoil mass center, expressed in parafoil frame
    const Vector3f vP_bf = vB_bf + (omega_bf % rP) + relative_velocity * (1 - canopy_fraction);
    const Vector3f vP_pf = R_pf_b * vP_bf;

    const float uP = vP_pf.x;
    const float vP = vP_pf.y;
    const float wP = vP_pf.z;
    const float V = vP_pf.length();

    out.V_pf = V;

    // AoA / sideslip in x-forward, y-right, z-down canopy axes.
    if (V > 0.1f) {
        const float u_safe = clamp_preserve_sign(uP, 0.01f);
        out.alpha_pf_rad = atan2f(wP, u_safe);
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
        // z normal to airflow and canopy span, pointing down in forward flight.
        // Aerodynamic force in wind axes: [-D, 0, -L]
        const Vector3f F_w{-D, 0.0f, -L};

        // Build wind->parafoil DCM from velocity direction (robust, avoids rotation-order pitfalls)
        Vector3f x_w = vP_pf * (1.0f / V);             // wind x axis in parafoil coords
        // Lift is perpendicular to both airflow and the canopy span.
        // Projecting the down axis onto airflow's normal plane instead
        // introduces an artificial spanwise lift force in sideslip.
        Vector3f z_w = x_w % Vector3f{0, 1, 0};
        if (z_w.length() < 1.0e-3f) {
            // Spanwise flow has no uniquely defined lift plane.
            const Vector3f down{0, 0, 1};
            z_w = down - x_w * (down * x_w);
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

        // Eq. 20-21: ax/ay/az already contain one power of velocity.
        // Multiply by V, not V^2, so brake forces scale with dynamic pressure.
        const float scale = 0.5f * air_density * V * model.A_para_m2;
        F_brake_pf = Vector3f{ax * k, ay * k, az * k} * scale;
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
    const float q = gyro.y + (pitch_joint_enabled() ? joint_pitch_rate : 0);
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
        // Eq. 22: b^2/d supplies the moment arm in metres. Cl_da and
        // Cn_da are dimensionless derivatives per radian of brake deflection.
        const float scale = (sq(model.b_span_m) / model.d_brake_m) * delta_a;
        M_brake_bf = Vector3f{
            model.aero.Cl_da * scale,
            0.0f,
            model.aero.Cn_da * scale
        } * qbarS;
    }

    // Thrust torque about CG: r_MB x F_thrust
    Vector3f rF = model.S_FB_B;
    Vector3f rP = model.S_PB_B;
    if (pitch_joint_enabled()) {
        pitch_geometry(rF, rP);
    }
    const Vector3f motor_arm = pitch_joint_enabled() ? Vector3f{0, 0, model.thrust_payload_z_m} : model.S_MF_F;
    const Vector3f r_MB_bf = rF + motor_arm;
    const Vector3f M_thrust_bf = r_MB_bf % F.F_thrust_bf;

    // Propeller reaction torque is separate from the thrust lever-arm moment.
    const Vector3f M_prop_bf{model.prop_torque_per_thrust_m * F.F_thrust_bf.x, 0.0f, 0.0f};

    // Lever arms from forces away from CG
    const Vector3f M_fuse_arm_bf = rF % F.F_fuse_bf;
    const Vector3f M_para_arm_bf = rP % (F.F_para_bf + F.F_brake_bf);

    // Roll damping
    const Vector3f M_roll_damp_bf{-model.roll_damp_Nm_per_rps * p, 0.0f, 0.0f};


    return M_aero_bf + M_brake_bf + M_thrust_bf + M_fuse_arm_bf + M_para_arm_bf + M_roll_damp_bf + M_prop_bf;
}

// Eliminate hinge translation using the system CG. For hinge-to-CG
// vectors a (payload) and b (canopy), kinetic energy is
// 1/2 Ip*qp^2 + 1/2 Ic*qc^2 + 1/2 mu*|qp Yxa - qc Yxb|^2.
// Uniform gravity cancels from these relative equations in free flight;
// canopy lift supplies the suspension loading and pendulum restoring force.
void Paraglider::pitch_geometry(Vector3f &payload_arm, Vector3f &canopy_arm) const
{
    const Vector3f a{0, 0, model.payload_hinge_z_m};
    const Vector3f b = rot_y(joint_pitch_rad) * Vector3f{model.canopy_hinge_x_m, 0, model.canopy_hinge_z_m};
    payload_arm = (a - b) * (model.canopy_mass_kg / model.mass_kg);
    canopy_arm = (b - a) * (1 - model.canopy_mass_kg / model.mass_kg);
}

void Paraglider::pitch_accelerations(const ForceBreakdown &F, float &payload_accel, float &canopy_accel) const
{
    const float fraction = model.canopy_mass_kg / model.mass_kg;
    const float mu = model.canopy_mass_kg * (1 - fraction);
    const Vector3f a{0, 0, model.payload_hinge_z_m};
    const Vector3f b = rot_y(joint_pitch_rad) * Vector3f{model.canopy_hinge_x_m, 0, model.canopy_hinge_z_m};
    const Vector3f da{a.z, 0, -a.x};
    const Vector3f db{b.z, 0, -b.x};
    const float qp = gyro.y;
    const float qc = qp + joint_pitch_rate;
    const Vector3f payload_force = F.F_fuse_bf + F.F_thrust_bf;
    const Vector3f canopy_force = F.F_para_bf + F.F_brake_bf;
    const Vector3f differential_force = payload_force * fraction - canopy_force * (1 - fraction);
    const float joint_torque = model.pitch_joint_stiffness * joint_pitch_rad +
                               model.pitch_joint_damping * joint_pitch_rate;
    float Qp = da * differential_force + model.thrust_payload_z_m * F.F_thrust_bf.x + joint_torque;
    float Qc = -(db * differential_force) - joint_torque;
    if (F.V_pf > 0.1f) {
        const float qS = 0.5f * air_density * model.A_para_m2 * sq(F.V_pf);
        Qc += qS * (model.aero.Cmq * sq(model.c_chord_m) * qc / (2 * F.V_pf) +
                   model.aero.Cm0 * model.c_chord_m +
                   model.aero.Cmalpha * model.c_chord_m * F.alpha_eff_rad);
    }
    // Centrifugal terms from the configuration-dependent mass matrix.
    Qp -= mu * (da * b) * sq(qc);
    Qc -= mu * (db * a) * sq(qp);
    const float m11 = model.payload_pitch_inertia + mu * (da * da);
    const float m22 = model.canopy_pitch_inertia + mu * (db * db);
    const float m12 = -mu * (da * db);
    if (model.pitch_joint_locked > 0.5f) {
        // Constraint torque cancels between bodies. Use the articulated
        // model's geometry and inertia, rather than the legacy rigid model.
        payload_accel = (Qp + Qc) / (m11 + m22 + 2 * m12);
        canopy_accel = payload_accel;
        return;
    }
    const float det = m11 * m22 - sq(m12);
    payload_accel = (m22 * Qp - m12 * Qc) / det;
    canopy_accel = (m11 * Qc - m12 * Qp) / det;
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

    bool launch_guided = false;
    if (have_launcher) {
        const bool launch_triggered = input.servos[6] > 1700;
        if (launch_triggered) {
            const uint64_t now = AP_HAL::millis64();
            if (!launch_started) {
                launch_start_ms = now;
                launch_started = true;
                launch_released = false;
            }
            const uint64_t launch_ms = uint64_t(launch_time * 1000.0f);
            // Release the synthetic guide once free-flight forces support
            // the weight, rather than continuing to add launch energy.
            if (launch_guide_attitude && !on_ground() &&
                (dcm * force_bf).z <= -mass * GRAVITY_MSS) {
                launch_released = true;
            }
            if (!launch_released && now - launch_start_ms < launch_ms) {
                launch_guided = launch_guide_attitude;
                force_bf.x += mass * launch_accel;
                force_bf.z -= mass * launch_accel / 3.0f; // negative z is up
            }
        } else {
            launch_start_ms = 0;
            launch_started = false;
            launch_released = false;
        }
    }

    accel_body = force_bf / mass;

    // Rotational dynamics with Ixz coupling
    const Vector3f omega = gyro;
    const Vector3f Iomega = inertia_mul(omega);
    const Vector3f net_tau = torque_bf - (omega % Iomega);

    rot_accel = inertia_inv_mul(net_tau);
    if (pitch_joint_enabled()) {
        float canopy_accel;
        pitch_accelerations(F, rot_accel.y, canopy_accel);
        joint_pitch_accel = canopy_accel - rot_accel.y;
        // Keep integrated position at the system CG, but report the IMU's
        // specific force at the payload CG. The canopy's relative angular
        // velocity also rotates the hinge axis when the shared body rolls.
        const Vector3f a{0, 0, model.payload_hinge_z_m};
        const Vector3f b = rot_y(joint_pitch_rad) * Vector3f{model.canopy_hinge_x_m, 0, model.canopy_hinge_z_m};
        const Vector3f relative_omega{0, joint_pitch_rate, 0};
        const Vector3f canopy_omega = gyro + relative_omega;
        const Vector3f canopy_alpha = rot_accel + Vector3f{0, joint_pitch_accel, 0} + (gyro % relative_omega);
        payload_accel_offset = ((rot_accel % a) + (gyro % (gyro % a)) -
                                (canopy_alpha % b) - (canopy_omega % (canopy_omega % b))) *
                               (model.canopy_mass_kg / model.mass_kg);
    }

    if (launch_guided) {
        // The synthetic throw guide supplies reaction moments during its
        // acceleration interval. Release restores the free-flight equations.
        gyro.zero();
        rot_accel.zero();
        joint_pitch_rate = 0;
        joint_pitch_accel = 0;
        payload_accel_offset.zero();
    }

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
    LOAD_PHYS_FLOAT(prop_torque_per_thrust_m);
    LOAD_PHYS_FLOAT(brake_max_rad);
    LOAD_PHYS_FLOAT(canopy_pitch_rad);
    LOAD_PHYS_FLOAT(roll_damp_Nm_per_rps);
    LOAD_PHYS_FLOAT(pitch_joint_enabled);
    LOAD_PHYS_FLOAT(pitch_joint_locked);
    LOAD_PHYS_FLOAT(canopy_mass_kg);
    LOAD_PHYS_FLOAT(payload_pitch_inertia);
    LOAD_PHYS_FLOAT(canopy_pitch_inertia);
    LOAD_PHYS_FLOAT(payload_hinge_z_m);
    LOAD_PHYS_FLOAT(canopy_hinge_x_m);
    LOAD_PHYS_FLOAT(canopy_hinge_z_m);
    LOAD_PHYS_FLOAT(thrust_payload_z_m);
    LOAD_PHYS_FLOAT(pitch_joint_damping);
    LOAD_PHYS_FLOAT(pitch_joint_stiffness);
#undef LOAD_PHYS_FLOAT

    mass = model.mass_kg;
    if (pitch_joint_enabled() &&
        (!isfinite(model.mass_kg) || !isfinite(model.canopy_mass_kg) ||
         !isfinite(model.payload_pitch_inertia) || !isfinite(model.canopy_pitch_inertia) ||
         !isfinite(model.payload_hinge_z_m) || !isfinite(model.canopy_hinge_x_m) ||
         !isfinite(model.canopy_hinge_z_m) || !isfinite(model.thrust_payload_z_m) ||
         !isfinite(model.pitch_joint_damping) || !isfinite(model.pitch_joint_stiffness) ||
         !(model.mass_kg > model.canopy_mass_kg) || !(model.canopy_mass_kg > 0) ||
         !(model.payload_pitch_inertia > 0) || !(model.canopy_pitch_inertia > 0) ||
         !(model.pitch_joint_damping >= 0) || !(model.pitch_joint_stiffness >= 0))) {
        AP_HAL::panic("Invalid paraglider pitch joint mass, inertia or damping");
    }

    delete obj;

    ::printf("Loaded paraglider coefficients from %s\n", fname);
    free(fname);
#endif
}

void Paraglider::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    // Keep a guided aircraft supported until the launch command, rather
    // than allowing motor thrust to initiate a separate ground run.
    ground_behavior = launch_guide_attitude && !launch_started && input.servos[6] <= 1700 ?
                      GROUND_BEHAVIOR_NO_MOVEMENT : GROUND_BEHAVIOR_FWD_ONLY;
    update_wind(input);
    calculate_forces(input, rot_accel);
    update_dynamics(rot_accel);
    if (pitch_joint_enabled()) {
        if (on_ground()) {
            joint_pitch_rad = 0;
            joint_pitch_rate = 0;
        } else {
            accel_body += payload_accel_offset;
            const float dt = frame_time_us * 1.0e-6f;
            joint_pitch_rate += joint_pitch_accel * dt;
            joint_pitch_rad += joint_pitch_rate * dt;
        }
        if (time_now_us - joint_log_us >= 20000) {
            joint_log_us = time_now_us;
            AP::logger().WriteStreaming("PGJT", "TimeUS,Rel,QRel,QP,QC", "Qffff",
                                       AP_HAL::micros64(), degrees(joint_pitch_rad),
                                       degrees(joint_pitch_rate), degrees(gyro.y),
                                       degrees(gyro.y + joint_pitch_rate));
        }
    }
    update_external_payload(input);
    update_position();
    time_advance();
    update_mag_field_bf();
}
