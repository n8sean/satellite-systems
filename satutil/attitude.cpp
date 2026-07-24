#include "includes/attitude_utility.h"
#include <cmath>
#include <numbers>
#include <string>
#include <array>
#include <iostream>
#include <iomanip>
#include "includes/json.hpp"
using json = nlohmann::json;


// ============= PARSING =============
AttitudeParams GetAttitudeParams(int argc, char* argv[]) {
    AttitudeParams params, qtmp;
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--q" && i + 4 < argc) {
            qtmp.input_q.w = std::stod(argv[++i]);
            qtmp.input_q.x = std::stod(argv[++i]);
            qtmp.input_q.y = std::stod(argv[++i]);
            qtmp.input_q.z = std::stod(argv[++i]);
            qtmp.input_q.Normalize();
            if (params.num_q == 0) {
                params.input_q = qtmp.input_q;  // initial '--q' specified
            } else {
                params.input_q_meas = qtmp.input_q;  // second '--q' specified
            }
            params.num_q++;
        } else if (arg == "--deg") {
            params.use_deg = true;
        } else if (arg == "--euler" && i + 3 < argc) {
            params.input_euler.roll = std::stod(argv[++i]);
            params.input_euler.pitch = std::stod(argv[++i]);
            params.input_euler.yaw = std::stod(argv[++i]);
            params.mode = "euler";
        } else if (arg == "--dcm" && i + 9 < argc) {
            params.input_dcm.m[0][0] = std::stod(argv[++i]);
            params.input_dcm.m[0][1] = std::stod(argv[++i]);
            params.input_dcm.m[0][2] = std::stod(argv[++i]);
            // ---
            params.input_dcm.m[1][0] = std::stod(argv[++i]);
            params.input_dcm.m[1][1] = std::stod(argv[++i]);
            params.input_dcm.m[1][2] = std::stod(argv[++i]);
            // ---
            params.input_dcm.m[2][0] = std::stod(argv[++i]);
            params.input_dcm.m[2][1] = std::stod(argv[++i]);
            params.input_dcm.m[2][2] = std::stod(argv[++i]);
            params.mode = "dcm";
        } else if (arg == "--omega" && i + 3 < argc) {
            params.wx = std::stod(argv[++i]);
            params.wy = std::stod(argv[++i]);
            params.wz = std::stod(argv[++i]);
            params.mode = "omega";
        } else if (arg == "--error") {
            params.mode = "error";
        } else if (arg == "--dt" && i+1 < argc) {
            params.dt = std::stod(argv[++i]);
        } else if (arg == "--json") {
            params.json_output = true;
        } else if (arg == "--help" && params.str_help == "") {
            params.help = true;
            params.str_help = GetAttitudeHelp();
        } else {
            std::cout << "Unknown flag: " << arg << std::endl;
        }
    }
    return params;
}


// ============= CONVERSION CALCULATIONS =============
void Quaternion::Normalize() {
    // Calculate magnitude (norm)
    double norm_sq = w * w + x * x + y * y + z * z;
    double norm = std::sqrt(norm_sq);

    // Avoid division by zero (return identity quaternion)
    if (norm < 1e-12) {
        w = 1.0, x = 0.0, y = 0.0, z = 0.0;
        return;
    }

    double scale = 1.0 / norm;
    w = w * scale;
    x = x * scale;
    y = y * scale;
    z = z * scale;
}

Quaternion Normalize(const Quaternion& q) {
    Quaternion result = q;
    result.Normalize();
    return result;
}

Quaternion EulerToQuaternion(double roll, double pitch, double yaw) {
    /* Convert Euler frame to Quaternion frame */
    Quaternion q;  // instantiate new struct{}
    // w params
    double q0_1 = std::cos(yaw / 2) * std::cos(pitch / 2) * std::cos(roll / 2);
    double q0_2 = std::sin(yaw / 2) * std::sin(pitch / 2) * std::sin(roll / 2);
    q.w = q0_1 + q0_2;
    // x params
    double q1_1 = std::cos(yaw / 2) * std::cos(pitch / 2) * std::sin(roll / 2);
    double q1_2 = std::sin(yaw / 2) * std::sin(pitch / 2) * std::cos(roll / 2);
    q.x = q1_1 - q1_2;
    // y params
    double q2_1 = std::cos(yaw / 2) * std::sin(pitch / 2) * std::cos(roll / 2);
    double q2_2 = std::sin(yaw / 2) * std::cos(pitch / 2) * std::sin(roll / 2);
    q.y = q2_1 + q2_2;
    // z params
    double q3_1 = std::sin(yaw / 2) * std::cos(pitch / 2) * std::cos(roll / 2);
    double q3_2 = std::cos(yaw / 2) * std::sin(pitch / 2) * std::sin(roll / 2);
    q.z = q3_1 - q3_2;
    return q;
}

Quaternion DCMToQuaternion(const DCM& d) {
    Quaternion q;  // new object creation
    double trace = d.m[0][0] + d.m[1][1] + d.m[2][2];
    if (trace > 0.0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        q.w = 0.25 / s;
        q.x = (d.m[2][1] - d.m[1][2]) * s;
        q.y = (d.m[0][2] - d.m[2][0]) * s;
        q.z = (d.m[1][0] - d.m[0][1]) * s;
    } else if (d.m[0][0] > d.m[1][1] && d.m[0][0] > d.m[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + d.m[0][0] - d.m[1][1] - d.m[2][2]);
        q.w = (d.m[2][1] - d.m[1][2]) / s;
        q.x = 0.25 * s;
        q.y = (d.m[0][1] + d.m[1][0]) / s;
        q.z = (d.m[0][2] + d.m[2][0]) / s;
    } else if (d.m[1][1] > d.m[2][2]) {
        double s = 2.0 * std::sqrt(1.0 + d.m[1][1] - d.m[0][0] - d.m[2][2]);
        q.w = (d.m[0][2] - d.m[2][0]) / s;
        q.x = (d.m[0][1] + d.m[1][0]) / s;
        q.y = 0.25 * s;
        q.z = (d.m[1][2] + d.m[2][1]) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + d.m[2][2] - d.m[0][0] - d.m[1][1]);
        q.w = (d.m[1][0] - d.m[0][1]) / s;
        q.x = (d.m[0][2] + d.m[2][0]) / s;
        q.y = (d.m[1][2] + d.m[2][1]) / s;
        q.z = 0.25 * s;
    }
    return q;
}

EulerAngles QuaternionToEuler(const Quaternion& q) {
    /* Convert Quaternion frame to Euler frame */
    EulerAngles e;  // instantiate new struct{}
    // Roll params
    double sinRoll_cosPitch = 2 * (q.w * q.x + q.y * q.z);
    double cosRoll_cosPitch = 1 - 2 * (q.x * q.x + q.y * q.y);
    e.roll = std::atan2(sinRoll_cosPitch, cosRoll_cosPitch);
    // Pitch params
    double sinPitch = -2 * (q.x * q.z - q.w * q.y);
    if (std::abs(sinPitch) >= 1.0) {
        e.pitch = std::copysign(std::numbers::pi / 2.0, sinPitch); // gimbal lock
    } else {
        e.pitch = std::asin(sinPitch);
    }
    // Yaw params
    double sinY_cosPitch = 2 * (q.w * q.z + q.x * q.y);
    double cosY_cosPitch = 1 - 2 * (q.y * q.y + q.z * q.z);
    e.yaw = std::atan2(sinY_cosPitch, cosY_cosPitch);
    return e;
}

DCM QuaternionToDCM(const Quaternion& q) {
    DCM dcm;  // declare new object
    double qx2 = q.x * q.x;
    double qy2 = q.y * q.y;
    double qz2 = q.z * q.z;
    double qxw = q.x * q.w;
    double qyw = q.y * q.w;
    double qzw = q.z * q.w;
    double qxy = q.x * q.y;
    double qxz = q.x * q.z;
    double qyz = q.y * q.z;

    dcm.m[0][0] = 1.0 - 2.0 * (qy2 + qz2);
    dcm.m[0][1] = 2.0 * (qxy - qzw);
    dcm.m[0][2] = 2.0 * (qxz + qyw);

    dcm.m[1][0] = 2.0 * (qxy + qzw);
    dcm.m[1][1] = 1.0 - 2.0 * (qx2 + qz2);
    dcm.m[1][2] = 2.0 * (qyz - qxw);

    dcm.m[2][0] = 2.0 * (qxz - qyw);
    dcm.m[2][1] = 2.0 * (qyz + qxw);
    dcm.m[2][2] = 1.0 - 2.0 * (qx2 + qy2);
    return dcm;
}

DCM IdentityDCM() {
    DCM dcm;        // Constructor already initializes it to identity
    return dcm;
}

void PrintDCM(const DCM& dcm, const std::string& name) {
    std::cout << name << ":\n\t\t";
    for (int i = 0; i < 3; ++i) {
        if (i > 0) {
            std::cout << "\t\t";
        }
        for (int j = 0; j < 3; ++j) {
            std::cout << std::fixed << std::setprecision(6) << dcm.m[i][j] << " ";
        }
        std::cout << "\n";
    }
}


// ============= CONTROL LAWS =============
std::array<double, 3> ComputePDTorque(
    const AttitudeError& err,
    const std::array<double, 3>& omega_body_rad_s,
    double kp,
    double kd,
    const std::array<double, 3>& inertia_kgm2)
{
    /* Simple 3-axis PD controller for attitude control.
       tau = -Kp * error_vector - Kd * omega
       Uses small-angle approximation from error quaternion vector part. */
    // Approximate error vector (small angle)
    std::array<double, 3> error_vec = {
        err.error_quat.x * 2.0,
        err.error_quat.y * 2.0,
        err.error_quat.z * 2.0
    };
    std::array<double, 3> torque = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; ++i) {
        torque[i] = -kp * error_vec[i] * inertia_kgm2[i] 
                    - kd * omega_body_rad_s[i] * inertia_kgm2[i];
    }
    return torque;
}

std::array<double, 3> ComputeBDotTorque(
    const std::array<double, 3>& b_body,
    const std::array<double, 3>& b_dot_body,
    double gain)
{
    /* B-dot control law: m = -gain * (db/dt)
       tau = m × B  (torque produced by magnetorquers) */
    // Commanded magnetic moment
    std::array<double, 3> m_cmd = {
        -gain * b_dot_body[0],
        -gain * b_dot_body[1],
        -gain * b_dot_body[2]
    };
    // Resulting torque: tau = m × B
    std::array<double, 3> tau;
    tau[0] = m_cmd[1] * b_body[2] - m_cmd[2] * b_body[1];
    tau[1] = m_cmd[2] * b_body[0] - m_cmd[0] * b_body[2];
    tau[2] = m_cmd[0] * b_body[1] - m_cmd[1] * b_body[0];
    return tau;
}


// ============= KINEMATICS LAWS =============
Quaternion QuaternionKinematics(const Quaternion& q, const std::array<double, 3>& omega, double dt) {
    /*
    Quaternion kinematics for rigid-body attitude propagation.
        q̇ = 0.5 * q ⊗ ω_quat   (where ω_quat = [0, ωx, ωy, ωz])
        Uses simple first-order Euler integration + renormalization.

    INPUTS:
        - q: Current unit quaternion (body-to-inertial attitude)
        - omega: Body angular velocity vector [rad/s] (from gyros)
        - dt: Integration timestep [s] (e.g., 0.01–0.1 for control loops)

    OUTPUT: Propagated (normalized) quaternion at t + dt
    */
    Quaternion omega_quat{0.0, omega[0], omega[1], omega[2]};
    Quaternion q_dot = QuaternionMultiplyScalar(QuaternionMultiply(q, omega_quat), 0.5);
    Quaternion q_new;
    q_new.w = q.w + q_dot.w * dt;
    q_new.x = q.x + q_dot.x * dt;
    q_new.y = q.y + q_dot.y * dt;
    q_new.z = q.z + q_dot.z * dt;
    q_new.Normalize();  // Critical for unit-norm preservation
    return q_new;
}

std::array<double, 3> GetAngularVelocityFromQuat(const Quaternion& q_prev, const Quaternion& q_curr, double dt) {
    /*
    Approximate body angular velocity (ω) from consecutive unit quaternions.
    Formula (small-angle finite difference):
         ω ≈ (2 / dt) * vector_part( q_curr ⊗ q_prev* )

    INPUTS:
        - q_prev, q_curr: Consecutive normalized attitude quaternions
        - dt: Time between measurements [s] (e.g., sensor sample period)

    OUTPUT: Body-frame angular velocity vector [rad/s] (x, y, z).

    Use Cases in Embedded Satellite ADCS:
        - Rate estimation when gyros are unavailable (e.g., from star-tracker quaternions).
        - Sensor fusion / complementary filtering with gyro data.
        - Validation of integrated rates or detection of anomalies.

    Assumptions: Small rotation angle between samples (dt * |ω| << 1 rad); normalize inputs.
        For larger angles, use full quaternion logarithm methods.
    */
    Quaternion q_prev_conj = QuaternionConjugate(q_prev);
    Quaternion dq = QuaternionMultiply(q_curr, q_prev_conj);
    dq = Normalize(dq);  // Ensure unit norm for vector extraction
    std::array<double, 3> omega = {
        2.0 * dq.x / dt,
        2.0 * dq.y / dt,
        2.0 * dq.z / dt
    };
    return omega;
}

std::array<double, 3> CrossProduct(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    std::array<double, 3> result = {0.0, 0.0, 0.0};
    result[0] = a[1] * b[2] - a[2] * b[1];  // x-component: ay*bz - az*by
    result[1] = a[2] * b[0] - a[0] * b[2];  // y-component: az*bx - ax*bz
    result[2] = a[0] * b[1] - a[1] * b[0];  // z-component: ax*by - ay*bx
    return result;
}

Quaternion QuaternionMultiplyScalar(const Quaternion& q, double scalar) {
    /* 
       Scalar multiplication of a quaternion: result = q * scalar

        INPUTS:
        - q: Input quaternion (w + xi + yj + zk)
        - scalar: Real scalar value (e.g., 0.5 for kinematics derivative)

        OUTPUT: Scaled quaternion (same orientation, adjusted magnitude)

        Used primarily for q_dot scaling in quaternion kinematics:
         q̇ = 0.5 * (q ⊗ ω_quat)

        Properties:
        - O(1) time/memory — perfect for real-time embedded loops
        - Preserves structure for subsequent operations (e.g., integration)
        - No normalization here (handled after full update to avoid drift)
    */
    Quaternion result;
    result.w = q.w * scalar;
    result.x = q.x * scalar;
    result.y = q.y * scalar;
    result.z = q.z * scalar;
    return result;
}

std::array<double, 3> GetAngularAcceleration(const std::array<double, 3>& omega_prev, const std::array<double, 3>& omega_curr, double dt) {
    /* 
    Finite-difference approximation of angular acceleration (α).
        α = (ω_curr - ω_prev) / dt

    INPUTS:
        - omega_prev, omega_curr: Consecutive body angular velocity vectors [rad/s]
        - dt: Time step between samples [s]

    OUTPUT:
        - alpha: Angular acceleration vector [rad/s²] (x, y, z components).

    Primary Uses in Satellite Systems:
        - Derivative feedforward in PD/PID attitude controllers.
        - Disturbance torque estimation (e.g., from solar pressure, drag, magnetic residuals).
        - Momentum management and reaction wheel desaturation logic.
        - Stability analysis and fault detection in embedded flight software.

    Notes: Simple first-order difference — best with filtered/low-noise rates.
    For higher accuracy, use multi-point stencils or combine with torque models.
    */
    std::array<double, 3> alpha = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; ++i) {
        alpha[i] = (omega_curr[i] - omega_prev[i]) / dt;
    }
    return alpha;
}

double ComputeAngularMomentum(const std::array<double, 3>& omega, const std::array<double, 3>& inertia) {
    /* 
    Computes magnitude of angular momentum vector |H| = |I × ω| 
    for a diagonal (principal axes) inertia tensor.

    INPUTS:
        - omega: Body angular velocity [rad/s] (3-axis vector)
        - inertia: Diagonal inertia tensor [kg·m²] (Ixx, Iyy, Izz)

    OUTPUT: Scalar |H| in [kg·m²/s].

    Equation (diagonal case):
        Hx = Ixx * ωx
        Hy = Iyy * ωy
        Hz = Izz * ωz
        |H| = sqrt(Hx² + Hy² + Hz²)

    Uses in Satellite ADCS (Embedded Systems):
        - Momentum wheel / reaction wheel state estimation.
        - Desaturation logic and control torque budgeting.
        - Stability margin checks and safe-mode triggers.
        - Energy/momentum conservation verification in simulations.

    Assumptions: Principal-axis aligned inertia (common simplification).
    For full 3x3 inertia tensor, extend with matrix-vector multiply.

    Numerical: Robust for typical satellite values (I ~ 1-100 kg·m², ω ~ 0.001-0.1 rad/s).
    */
    double hx = inertia[0] * omega[0];
    double hy = inertia[1] * omega[1];
    double hz = inertia[2] * omega[2];
    return std::sqrt(hx*hx + hy*hy + hz*hz);
}


// ============= ERROR CALCULATIONS =============
double ComputePointingError(const DCM& dcm_ref, const DCM& dcm_meas) {
    // Boresight = body Z-axis [0,0,1]
    std::array<double, 3> boresight_ref = {dcm_ref.m[0][2], dcm_ref.m[1][2], dcm_ref.m[2][2]};
    std::array<double, 3> boresight_meas = {dcm_meas.m[0][2], dcm_meas.m[1][2], dcm_meas.m[2][2]};
    double dot = boresight_ref[0] * boresight_meas[0] + boresight_ref[1]*boresight_meas[1] + boresight_ref[2] * boresight_meas[2];
    dot = std::max(-1.0, std::min(1.0, dot));
    return std::acos(dot) * 180.0 / std::numbers::pi;  // degrees
}

double ComputePointingError(const Quaternion& q_ref, const Quaternion& q_meas) {
    /* Overload: Compute pointing error using quaternions by converting to DCM internally */
    DCM dcm_ref = QuaternionToDCM(q_ref);
    DCM dcm_meas = QuaternionToDCM(q_meas);
    return ComputePointingError(dcm_ref, dcm_meas);
}

AttitudeError ComputeAttitudeError(const Quaternion& q_ref, const Quaternion& q_meas) {
    /*Computes the relative attitude error between a reference (desired) quaternion and a measured (current) quaternion
    INPUTS
    q_ref: Reference (desired) attitude quaternion (unit norm)
    q_meas: Measured (current) attitude quaternion (unit norm)
    OUTPUT
    AttitudeError: struct
    */
    AttitudeError err;
    // Error quaternion
    Quaternion q_ref_conj = QuaternionConjugate(q_ref);
    Quaternion q_err = QuaternionMultiply(q_ref_conj, q_meas);
    // Normalize
    q_err = Normalize(q_err);
    // Error angle
    double cos_half_theta = std::abs(q_err.w);
    if (cos_half_theta > 1.0) cos_half_theta = 1.0;
    err.error_angle_rad = 2.0 * std::acos(cos_half_theta);
    err.error_angle_deg = err.error_angle_rad * 180.0 / std::numbers::pi;
    // Euler errors (approximate)
    auto e_ref = QuaternionToEuler(q_ref);
    auto e_meas = QuaternionToEuler(q_meas);
    err.roll_error_deg  = (e_meas.roll - e_ref.roll) * 180.0 / std::numbers::pi;
    err.pitch_error_deg = (e_meas.pitch - e_ref.pitch) * 180.0 / std::numbers::pi;
    err.yaw_error_deg   = (e_meas.yaw - e_ref.yaw) * 180.0 / std::numbers::pi;
    err.error_quat = q_err;
    return err;
}

Quaternion QuaternionConjugate(const Quaternion& q) {
    /*Returns the conjugate of a quaternion.*/
    Quaternion conj;
    conj.w = q.w;
    conj.x = -q.x;
    conj.y = -q.y;
    conj.z = -q.z;
    return conj;
}

Quaternion QuaternionMultiply(const Quaternion& q1, const Quaternion& q2) {
    Quaternion result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

double ComputeQuaternionErrorNorm(const Quaternion& q_err) {
    /*Compute the magnitude of the vector part of the error quaternion*/
    // Assume q_err is already normalized (from ComputeAttitudeError)
    double vec_norm_sq = q_err.x * q_err.x + q_err.y * q_err.y + q_err.z * q_err.z;
    // Clamp for numerical stability
    if (vec_norm_sq > 1.0) vec_norm_sq = 1.0;
    double vec_norm = std::sqrt(vec_norm_sq);
    return 2.0 * vec_norm;
}

void PrintAttitudeReport(const AttitudeError& err, const std::string& label="Attitude Error Report") {
    std::cout << "\n= " << label << " ---------------------------------------|\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  Error Angle\t\t" << err.error_angle_deg << " deg  ("<< err.error_angle_rad << " rad)\n";
    std::cout << "  Small-Angle Norm\t" << ComputeQuaternionErrorNorm(err.error_quat)<< " rad\n";
    std::cout << "  Roll  Error\t\t" << err.roll_error_deg << " deg\n";
    std::cout << "  Pitch Error\t\t" << err.pitch_error_deg << " deg\n";
    std::cout << "  Yaw   Error\t\t" << err.yaw_error_deg << " deg\n";
    std::cout << "  Error Quaternion\t["<< err.error_quat.w << ", " << err.error_quat.x << ", " << err.error_quat.y << ", " << err.error_quat.z << "]\n";
}


// ============= UTILITIES =============
void GetErrorMetrics(const Quaternion& q_ref, const Quaternion& q_meas) {
    /* Legacy aggregator: Computes primary attitude error metric.
       Returns the exact error angle in radians (most useful scalar value). */
    AttitudeError err = ComputeAttitudeError(q_ref, q_meas);
    PrintAttitudeReport(err);
    return;   // Primary output: error angle in radians
}

double ComputeStabilityMargin(const AttitudeError& err, 
                            const std::array<double, 3>& omega, 
                            const std::array<double, 3>& inertia_kgm2, 
                            double kp = 0.8, double kd = 2.5) {
    /* 
    Computes a stability margin metric for closed-loop attitude control.
        Based on simplified Lyapunov candidate V = 0.5 * (error_angle² + ωᵀ I ω) + PD energy terms.
    Higher positive value → better stability margin (distance to instability).

    INPUTS:
        - err: AttitudeError struct (error quaternion/angle from ComputeAttitudeError)
        - omega: Body angular rates [rad/s]
        - inertia_kgm2: Diagonal inertia tensor [kg·m²]
        - kp, kd: PD controller gains (tunable)

    OUTPUT: Scalar stability margin (positive = stable; larger = more robust).

    Key Terms:
        - Attitude potential: ~0.5 * kp * error_angle²
        - Kinetic energy: 0.5 * ωᵀ I ω
        - Damping contribution: kd * |ω| term (energy dissipation)
        - Overall: V + margin estimate (normalized for interpretability)

    Uses in Embedded Satellite ADCS:
        - Real-time stability monitoring during slew maneuvers.
        - Gain scheduling and adaptive control.
        - Safe-mode entry criteria (if margin < threshold → detumble).
        - Post-maneuver verification.

    Assumptions: PD-like control law; small-to-moderate errors. Extend with full Lyapunov analysis for nonlinear cases.
    */
    double error_norm = ComputeQuaternionErrorNorm(err.error_quat);  // Small-angle error [rad]
    double kinetic = 0.5 * (inertia_kgm2[0]*omega[0]*omega[0] +
                            inertia_kgm2[1]*omega[1]*omega[1] +
                            inertia_kgm2[2]*omega[2]*omega[2]);
    double potential = 0.5 * kp * error_norm * error_norm;
    double damping = kd * std::sqrt(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]);
    // Lyapunov-like candidate + margin (higher = more stable)
    double V = potential + kinetic;
    double margin = 1.0 / (1.0 + V) * (1.0 + damping);  // Normalized [0, ~2+]
    return margin;
}

double ComputePowerConsumption(const std::array<double, 3>& torque_nm, 
                                const std::array<double, 3>& omega_rad_s, 
                                double efficiency = 0.85) {
    /* 
    Estimates instantaneous power consumption for attitude control actuators 
        (reaction wheels, control moment gyros, or magnetorquers).

    INPUTS:
        - torque_nm: Commanded control torque vector [Nm] (from PD/B-dot/etc.)
        - omega_rad_s: Body/wheel angular rates [rad/s]
        - efficiency: Actuator efficiency (0.0-1.0, typical 0.8-0.9 for wheels)
        // - voltage: Bus voltage [V] (e.g., 28V for smallsats)

    OUTPUT: Estimated power draw [Watts].

    Calculation:
        - Mechanical power: |τ · ω| (ideal work rate)
        - Electrical power: mechanical / efficiency + quiescent losses
        - Total: P = ( |τ| * |ω| / efficiency ) + base_load (simplified model)

    Uses in Embedded Satellite ADCS:
        - Real-time power budgeting and energy management.
        - Battery state-of-charge prediction during slews.
        - Thermal/power constraint enforcement in safe modes.
        - Trade studies for actuator sizing.

    Assumptions: Dominant term is torque-rate product; magnetorquers use |m × B| approximation internally.
    Extend with detailed motor models (current/torque constants) for higher fidelity.
    */
    double tau_mag = std::sqrt(torque_nm[0]*torque_nm[0] +
                                torque_nm[1]*torque_nm[1] + 
                                torque_nm[2]*torque_nm[2]);
    double omega_mag = std::sqrt(omega_rad_s[0]*omega_rad_s[0] + 
                                omega_rad_s[1]*omega_rad_s[1] + 
                                omega_rad_s[2]*omega_rad_s[2]);
    double mech_power = tau_mag * omega_mag;  // Ideal mechanical power
    double elec_power = mech_power / std::max(0.01, efficiency);  // Avoid div-by-zero
    // Add representative quiescent/base load for electronics (tunable)
    const double base_load_w = 2.0;
    return elec_power + base_load_w;
}

// In attitude.cpp (declared in includes/attitude_utility.h)
void PropagateQuaternion(Quaternion& q, const std::array<double, 3>& omega, double dt) {
    /* 
    In-place quaternion propagation for efficient real-time attitude updates.
    Calls QuaternionKinematics internally and updates the input quaternion.

    INPUTS (by reference for efficiency):
        - q: Current unit quaternion (modified in-place to new attitude)
        - omega: Body angular velocity vector [rad/s]
        - dt: Integration timestep [s]

    OUTPUT: None (q is updated directly).

    Advantages for Embedded Satellite ADCS:
        - Zero-copy / minimal memory footprint.
        - Ideal for fixed-rate control loops (e.g., 10-100 Hz).
        - Simplifies state management in flight software.

    Usage Pattern:
        PropagateQuaternion(current_q, measured_omega, control_dt);

    See QuaternionKinematics for the underlying math (Euler integration + normalization).
    */
    q = QuaternionKinematics(q, omega, dt);
}

std::string GetAttitudeHelp() {
    return R"HELP(
Usage: satutil attitude [mode] [options...]

Subcommand for Attitude Determination & Control System (ADCS) utilities
optimized for embedded satellite systems.

MODES / COMMANDS:
quaternion                  Convert to/from quaternions (default)
euler                       Euler angle conversions (roll, pitch, yaw)
dcm                         Direction Cosine Matrix operations
omega                       Angular rate / kinematics propagation
error                       Compute attitude error metrics

COMMON OPTIONS:
--q w x y z                 Input quaternion (scalar-first, normalized)
--dcm m00 m01 m02 m10 m11 m12 m20 m21 m22
                            Input 3x3 DCM (row-major)
--euler roll pitch yaw      Input Euler angles in radians (or --deg)
--omega wx wy wz            Body angular rates [rad/s] for kinematics
--dt <seconds>              Time step for propagation (default: 0.01)
--deg                       Interpret angles as degrees
--json                      Output structured JSON instead of human-readable
--help                      Show this help

ERROR MODE (--error) OUTPUT:
    - Exact attitude error angle + small-angle norm
    - Roll / Pitch / Yaw error components
    - Error quaternion
    - PD control torque estimate [Nm]
    - B-dot (magnetorquer) torque estimate [Nm]
    - Kinematic quaternion propagation over multiple steps
    - Angular momentum |H|
    - Estimated angular acceleration
    - Stability margin (Lyapunov-inspired)
    - Estimated actuator power [W]

    Optional --omega and --dt values are used when supplied; otherwise
    representative demo rates/inertia are applied for the control & kinematics
    sections of the report.

EXAMPLES:
satutil attitude --q 0.7071 0.0 0.0 0.7071
satutil attitude --dcm 1 0 0  0 1 0  0 0 1
satutil attitude --euler 0.1 0.2 0.3 --deg
satutil attitude --omega 0.001 0.0 -0.0005 --dt 0.1
satutil attitude --error --q 1 0 0 0 --q 0.7071 0 0 0.7071
satutil attitude --error --q 1 0 0 0 --q 0.7071 0 0 0.7071 --omega 0.02 -0.01 0.005 --dt 0.05
)HELP";
}

std::string ToJson(AttitudeParams& params) {
    json data;
    data["mode"] = params.mode;
    // ---
    data["input_q"] = {{"w", params.input_q.w},
                        {"x", params.input_q.x},
                        {"y", params.input_q.y},
                        {"z", params.input_q.z}
                    };
    // ---
    data["input_q_meas"] = {{"w", params.input_q_meas.w},
                            {"x", params.input_q_meas.x},
                            {"y", params.input_q_meas.y},
                            {"z", params.input_q_meas.z}
                        };
    // ---
    data["input_euler"] = {{"roll", params.input_euler.roll},
                            {"pitch", params.input_euler.pitch},
                            {"yaw", params.input_euler.yaw}
                        };
    // ---
    data["num_q"] = params.num_q;
    data["wx"] = params.wx;
    data["wy"] = params.wy;
    data["wz"] = params.wz;
    data["dt"] = params.dt;
    data["use_deg"] = params.use_deg;
    data["json_output"] = params.json_output;
    data["help"] = params.help;
    data["str_help"] = params.str_help;
    return data.dump();
}
