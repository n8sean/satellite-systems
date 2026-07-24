#pragma once
#include <string>
#include <numbers>
#include <array>


/////////// STRUCTS ///////////
struct Quaternion {
    double w = 1.0;  // q0
    double x = 0.0;  // q1
    double y = 0.0;  // q2
    double z = 0.0;  // q3
    void Normalize();
};

struct EulerAngles {
    // φ, θ, ψ in radians
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

struct DCM {
    std::array<std::array<double, 3>, 3> m;  // 3x3 matrix
    DCM() {
        // Initialize to identity matrix
        m[0] = {1, 0, 0};
        m[1] = {0, 1, 0};
        m[2] = {0, 0, 1};
    }
};

struct AttitudeParams {
    std::string mode = "";        // quaternion | euler | dcm | omega
    // Input representations
    Quaternion input_q{1.0, 0.0, 0.0, 0.0};
    Quaternion input_q_meas{1.0, 0.0, 0.0, 0.0};  // q_meas for --error
    int num_q = 0;  // tracks how many '--q' flags were applied
    EulerAngles input_euler{0.0, 0.0, 0.0};
    DCM input_dcm;
    // Kinematics & control
    double wx = 0.0, wy = 0.0, wz = 0.0;    // body angular rates [rad/s]
    double dt = 0.01;                       // time step for integration [s]
    // Flags
    bool use_deg = false;
    bool json_output = false;
    bool help = false;
    std::string str_help = "";
};

struct AttitudeError {
    double error_angle_rad = 0.0;  // Exact rotation error angle [0, π]
    double error_angle_deg = 0.0;
    double roll_error_deg = 0.0;  // Euler angle differences (approximate)
    double pitch_error_deg = 0.0;
    double yaw_error_deg = 0.0;
    Quaternion error_quat;  // q_err = q_ref* ⊗ q_meas  (full error quaternion)
};


/////////// PARSING ///////////
AttitudeParams GetAttitudeParams(int argc, char* argv[]);


/////////// CONVERSIONS ///////////
// Euler Angles <-> Euler Parameters (Quaternions)
Quaternion EulerToQuaternion(double roll, double pitch, double yaw);
EulerAngles QuaternionToEuler(const Quaternion& q);
Quaternion Normalize(const Quaternion& q);


// DCM
DCM QuaternionToDCM(const Quaternion& q);
Quaternion DCMToQuaternion(const DCM& d);


/////////// CONTROL LAWS ///////////
// ADCS
std::array<double, 3> ComputePDTorque(
    const AttitudeError& err,
    const std::array<double, 3>& omega_body_rad_s,
    double kp,          // Proportional gain
    double kd,          // Derivative gain
    const std::array<double, 3>& inertia_kgm2  // Diagonal inertia tensor (Ixx, Iyy, Izz)
);
std::array<double, 3> ComputeBDotTorque(
    const std::array<double, 3>& b_body,
    const std::array<double, 3>& b_dot_body,
    double gain = 1e5);


/////////// KINEMATICS LAWS ///////////
Quaternion QuaternionKinematics(const Quaternion& q, const std::array<double, 3>& omega, double dt);
std::array<double, 3> GetAngularVelocityFromQuat(const Quaternion& q_prev, const Quaternion& q_curr, double dt);
std::array<double, 3> GetAngularAcceleration(const std::array<double, 3>& omega_prev, const std::array<double, 3>& omega_curr, double dt);
double ComputeAngularMomentum(const std::array<double, 3>& omega, const std::array<double, 3>& inertia);
std::array<double, 3> CrossProduct(const std::array<double, 3>& a, const std::array<double, 3>& b);
Quaternion QuaternionMultiplyScalar(const Quaternion& q, double scalar);


/////////// ERRORS ///////////
double ComputePointingError(const DCM& dcm_ref, const DCM& dcm_meas);  // Boresight / payload alignment
double ComputePointingError(const Quaternion& q_ref, const Quaternion& q_meas);  // overload
AttitudeError ComputeAttitudeError(const Quaternion& q_ref, const Quaternion& q_meas);
Quaternion QuaternionConjugate(const Quaternion& q);
Quaternion QuaternionMultiply(const Quaternion& q1, const Quaternion& q2);
double ComputeQuaternionErrorNorm(const Quaternion& q_err);  // ||q_err_vec|| for small-angle approx


/////////// UTILITIES ///////////
void GetErrorMetrics(const Quaternion& q_ref, const Quaternion& q_meas);  // Legacy aggregator → JSON output
void PrintAttitudeReport(const AttitudeError& err, const std::string& label);  // Human + machine readable
double ComputeStabilityMargin(const AttitudeError& err, 
                            const std::array<double, 3>& omega, 
                            const std::array<double, 3>& inertia_kgm2, 
                            double kp,
                            double kd);  // Lyapunov or eigenvalue-based
double ComputePowerConsumption(const std::array<double, 3>& torque_nm, 
                                const std::array<double, 3>& omega_rad_s, 
                                double efficiency);  // From torque + wheel speed
void PropagateQuaternion(Quaternion& q, const std::array<double, 3>& omega, double dt);
void PrintDCM(const DCM& dcm, const std::string& name = "DCM");
DCM IdentityDCM();
std::string GetAttitudeHelp();
std::string ToJson(AttitudeParams& params);

