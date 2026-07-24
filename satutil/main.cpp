#include <iostream>
#include <numbers>
#include <string>
#include <vector>
#include <cstdlib>
#include <iomanip>
#include "includes/orbital_utility.h"
#include "includes/attitude_utility.h"
#include "includes/json.hpp"
using json = nlohmann::json;


int handle_orbit(int argc, char* argv[]) {
    // CLI: satutil orbit --body earth --alt 550000
    // CLI: satutil orbit --body mars --hohmann --r1 6778000 --r2 13216400
    std::cout << "===============================================================" << std::endl;
    std::cout << "-=-+-=-+-=-+-=-  Orbital Mechanics Calculator  -=-+-=-+-=-+-=-\n";
    std::cout << "===============================================================" << std::endl;
    auto [body, radius, sma, alt, mu, use_hohmann, r1, r2, json_output, help] = GetOrbitalParams(argc, argv);
    if ((alt > 0.0 || use_hohmann) && help == "") {
        json data;
            data["Body"] = body;
            data["Radius"] = altitudetoRadius(radius, alt);
            data["Altitude"] = alt;
            data["SMA"] = sma;
            data["Period"] = OrbitalPeriod(altitudetoRadius(radius, alt), mu);
            data["Velocity"] = OrbitalVelocity(altitudetoRadius(radius, alt), mu);
            data["Energy"] = SpecificMechanicalEnergy(altitudetoRadius(radius, alt), mu);
            data["Mean Motion"] = MeanMotion(sma, mu);
            data["Escape Velocity"] = EscapeVelocity(altitudetoRadius(radius, alt), mu);
        // update string to all upper case
        for (char &ch: body) {
            ch = toupper(ch);
        }
        std::cout << "  Body\t\t\t\t" << static_cast<std::string>(body) << std::endl;
        std::cout << "  Body Radius\t\t\t" << radius * 10e-4 << " km" << std::endl;
        std::cout << "  Sattelite Altitude\t\t" << std::fixed << std::setprecision(2) << static_cast<double>(data["Altitude"]) * 10e-4 << " km" << std::endl;
        std::cout << "  Sattelite Radius\t\t" << std::fixed << std::setprecision(2) << static_cast<double>(data["Radius"]) * 10e-4 <<  " km" << std::endl;
        std::cout << "  Semi-major Axis\t\t" << std::fixed << std::setprecision(2) << static_cast<double>(data["SMA"]) * 10e-4 << " km" << std::endl;
        std::cout << "  ---------------\t\t---------" << std::endl;
        std::cout << "  Period\t\t\t" << std::fixed << std::setprecision(2) << static_cast<double>(data["Period"]) / 60 << " min" << std::endl;
        std::cout << "  Velocity\t\t\t" << std::fixed << std::setprecision(2) << static_cast<double>(data["Velocity"]) <<  " m/s" << std::endl;
        std::cout << "  Specific Mechanical Energy\t" << std::fixed << std::setprecision(2) << static_cast<double>(data["Energy"]) * 10e-7 <<  " MJ/kg" << std::endl;
        std::cout << "  Mean Motion\t\t\t" << std::fixed << std::setprecision(6) << static_cast<double>(data["Mean Motion"]) <<  " rad/s" << std::endl;
        std::cout << "  Escape Velocity\t\t" << std::fixed << std::setprecision(2) << static_cast<double>(data["Escape Velocity"]) * 10e-4 <<  " km/s" << std::endl;
        // derive Hohmann transfer calculations
        if (use_hohmann && r1 >= radius) {
            std::cout << "  ---------------\t\t---------\n";
            HohmannTransfer(r1, r2, mu);
        } else if (use_hohmann && r1 < radius) {
            std::cout << "   Error: Invalid radii for Hohmann transfer (r1 < r2 required)" << std::endl;
        }
        if (json_output) {
            std::cout << "JSON Output:\n" << data << std::endl;
        }
    } else if (help != "") {
        std::cout << help << std::endl;
    } else {
        std::cout << "ERROR: --alt parameter is required" << std::endl;
    }
    return 0;
}

int handle_attitude(int argc, char* argv[]) {
    std::cout << "=================================================================" << std::endl;
    std::cout << "-=-+-=-+-=-+-=-       Attitude & Kinematics       -=-+-=-+-=-+-=-\n";
    std::cout << "=================================================================" << std::endl;
    AttitudeParams attitude_params = GetAttitudeParams(argc, argv);
    Quaternion q;
    if (attitude_params.json_output == false) {
        if(attitude_params.help) {
            std::cout << attitude_params.str_help << std::endl;
            return 0;
        }
        if (attitude_params.mode == "dcm") {
            q = DCMToQuaternion(attitude_params.input_dcm);
            std::cout << "Input DCM parsed and converted to Quaternion\n";
            PrintDCM(attitude_params.input_dcm, "Input DCM");
        } else if (attitude_params.mode == "omega") {
            std::array<double, 3> omega = {attitude_params.wx, attitude_params.wy, attitude_params.wz};
            q = QuaternionKinematics(q, omega, attitude_params.dt);
            std::cout << "=== Angular Rate Propagation ===\n";
            std::cout << "Input omega (rad/s)\t[" << attitude_params.wx << ", " << attitude_params.wy << ", " << attitude_params.wz << "]\n";
            std::cout << "dt (seconds)\t\t" << attitude_params.dt << "\n";
            std::cout << "Updated Quaternion\t[" 
                        << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "]\n\n";
        } else if (attitude_params.mode == "euler" && attitude_params.use_deg == true) {
            double roll = attitude_params.input_euler.roll * std::numbers::pi / 180.0;
            double pitch= attitude_params.input_euler.pitch * std::numbers::pi / 180.0;
            double yaw = attitude_params.input_euler.yaw * std::numbers::pi / 180.0;
            q = EulerToQuaternion(roll, pitch, yaw);
            std::cout << "Input Euler (deg): Roll=" << attitude_params.input_euler.roll 
                        << " Pitch=" << attitude_params.input_euler.pitch 
                        << " Yaw=" << attitude_params.input_euler.yaw << "\n";
        } else if (attitude_params.mode == "euler" && attitude_params.use_deg == false) {
            double roll = attitude_params.input_euler.roll;
            double pitch= attitude_params.input_euler.pitch;
            double yaw = attitude_params.input_euler.yaw;
            q = EulerToQuaternion(roll, pitch, yaw);
            std::cout << "Input Euler (rad): Roll=" << roll << " Pitch=" << pitch << " Yaw=" << yaw << "\n";
        } else if (attitude_params.mode == "quaternion") {
            q = attitude_params.input_q;
            std::cout << "Input Quaternion (default)\n  [" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "]\n";
        } else if (attitude_params.mode != "error") {
            // Common output
            auto e = QuaternionToEuler(q);
            std::cout << "Quaternion: [" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "]\n";
            std::cout << "Euler Angles (rad): Roll=" << e.roll << " Pitch=" << e.pitch << " Yaw=" << e.yaw << "\n";
        } else if (attitude_params.mode == "error") {
            if (attitude_params.num_q < 2) {
                    std::cerr << "ERROR: --error requires two --q <w x y z> (q_ref then q_meas)\n"
                            << "Example: satutil attitude --error --q 1 0 0 0 --q 0.7071 0 0 0.7071\n";
                    return 1;
                }
                const Quaternion& q_ref  = attitude_params.input_q;
                const Quaternion& q_meas = attitude_params.input_q_meas;
                std::cout << "=== Attitude Error Metrics (ADCS) ===\n"
                        << std::fixed << std::setprecision(6)
                        << "  q_ref  = [" << q_ref.w  << ", " << q_ref.x  << ", "
                                            << q_ref.y  << ", " << q_ref.z  << "]\n"
                        << "  q_meas = [" << q_meas.w << ", " << q_meas.x << ", "
                                            << q_meas.y << ", " << q_meas.z << "]\n";
                GetErrorMetrics(q_ref, q_meas);
            // --- Exposed advanced ADCS report ---
            AttitudeError err = ComputeAttitudeError(q_ref, q_meas);
            // Prefer CLI-supplied rates; fall back to default values
            std::array<double, 3> omega = {
                (attitude_params.wx != 0.0 || attitude_params.wy != 0.0 || attitude_params.wz != 0.0)
                    ? attitude_params.wx : 0.01,
                (attitude_params.wx != 0.0 || attitude_params.wy != 0.0 || attitude_params.wz != 0.0)
                    ? attitude_params.wy : 0.005,
                (attitude_params.wx != 0.0 || attitude_params.wy != 0.0 || attitude_params.wz != 0.0)
                    ? attitude_params.wz : 0.002
            };
            std::array<double, 3> inertia = {10.0, 12.0, 8.0};   // representative diagonal inertia [kg·m²]
            auto torque = ComputePDTorque(err, omega, 0.8, 2.5, inertia);
            std::cout << "\n= PD Control Torque [Nm]  -------------------------------------|\n  ["
                    << torque[0] << ", " << torque[1] << ", " << torque[2] << "]\n";

            // --- CONTROL LAWS (magnetorquer B-dot)
            std::array<double, 3> b_body     = {20e-6, -15e-6, 35e-6};   // Measured B field (Tesla)
            std::array<double, 3> b_dot_body = {0.5e-6, -0.3e-6, 1.2e-6};
            auto b_dot_torque = ComputeBDotTorque(b_body, b_dot_body, 1.2e5);
            std::cout << "\n= B-dot Torque [Nm]  ------------------------------------------|\n  ["
                    << b_dot_torque[0] << ", " << b_dot_torque[1] << ", " << b_dot_torque[2] << "]\n";

            // --- KINEMATICS LAWS
            std::cout << "\n= Kinematic Propagation & Stability (Embedded ADCS)  ----------|\n";
            Quaternion q_kin = q_ref;                                 // start from reference attitude
            double dt = (attitude_params.dt > 0.0) ? attitude_params.dt : 0.05;
            int steps = 10;
            std::cout << "\n  Initial Quaternion  -----------------------------------------|\n    [" << std::fixed << std::setprecision(6)
                    << q_kin.w << ", " << q_kin.x << ", " << q_kin.y << ", " << q_kin.z << "]\n";
            std::cout << "\n  Body Rates [rad/s]  -----------------------------------------|\n    [" << omega[0] << ", " << omega[1] << ", " << omega[2] << "]\n";
            std::cout << "\n  Quaternion Timestep, dt  ------------------------------------|\n    " << dt << " seconds | Steps: " << steps << "\n";
            for (int s = 1; s <= steps; ++s) {
                PropagateQuaternion(q_kin, omega, dt);
                if (s % 2 == 0 || s == steps) {
                    std::cout << "    t=" << std::fixed << std::setprecision(2) << (s * dt) << "s  q=["
                            << std::setprecision(6) << q_kin.w << ", " << q_kin.x << ", "
                            << q_kin.y << ", " << q_kin.z << "]\n";
                }
            }
            double H = ComputeAngularMomentum(omega, inertia);
            std::cout << "\n  Angular Momentum |H| [kg*m^2/s]  ----------------------------|\n    "
                    << std::fixed << std::setprecision(4) << H << "\n";
            std::array<double, 3> omega_prev = omega;
            auto alpha = GetAngularAcceleration(omega_prev, omega, dt);
            std::cout << "\n  Angular Acceleration, estimated [rad/s^2]  ------------------|\n    ["
                    << alpha[0] << ", " << alpha[1] << ", " << alpha[2] << "]\n";
            double kp = 0.9, kd = 2.5;
            double margin = ComputeStabilityMargin(err, omega, inertia, kp, kd);
            std::cout << "\n  Stability Margin (higher = more robust)  --------------------|\n    "
                    << std::fixed << std::setprecision(2) << margin << "\n";
            double efficiency = 0.90;
            double voltage = 28.5;  // bus voltage
            std::cout << "\n= Bus Voltage [V]  --------------------------------------------|\n    " << voltage << "\n";
            double power = ComputePowerConsumption(torque, omega, efficiency);
            std::cout << "\n= Actuator Power, estimated [W]  ------------------------------|\n    "
                    << std::fixed << std::setprecision(2) << power << "\n";
            }

///////////////////////////////////////////////////////////////////////////////////////////////

    } else if (attitude_params.json_output) {
        std::cout << ToJson(attitude_params) << std::endl;
    }


    // AttitudeError err = ComputeAttitudeError(q_ref, q_meas);
    // auto torque = ComputePDTorque(err, omega, 0.8, 2.5, inertia);
    // std::cout << "\n= PD Control Torque [Nm]  --------------------------------------\n  [" 
    //         << torque[0] << ", " << torque[1] << ", " << torque[2] << "]\n";
    // // --- CONTROL LAWS
    // std::array<double, 3> b_body = {20e-6, -15e-6, 35e-6};     // Measured B field (Tesla)
    // std::array<double, 3> b_dot_body = {0.5e-6, -0.3e-6, 1.2e-6};
    // auto b_dot_torque = ComputeBDotTorque(b_body, b_dot_body, 1.2e5);
    // std::cout << "\n= B-dot Torque [Nm]  -------------------------------------------\n  [" 
    //         << b_dot_torque[0] << ", " << b_dot_torque[1] << ", " << b_dot_torque[2] << "]\n";
    // // --- KINEMATICS LAWS
    // std::cout << "\n= Kinematic Propagation & Stability (Embedded ADCS)  -----------\n";
    // Quaternion q_kin{1.0, 0.0, 0.0, 0.0};  // Initial attitude
    // std::array<double, 3> omega_kin = {0.01, 0.005, 0.002};  // Body rates [rad/s]
    // double dt = 0.05;  // Timestep [s]
    // int steps = 10;
    // std::array<double, 3> inertia_kin = {10.0, 12.0, 8.0};
    // std::cout << "  Initial Quaternion\n    [" << std::fixed << std::setprecision(6) 
    //             << q_kin.w << ", " << q_kin.x << ", " << q_kin.y << ", " << q_kin.z << "]\n";
    // std::cout << "  Body Rates [rad/s]\n    [" << omega_kin[0] << ", " << omega_kin[1] << ", " << omega_kin[2] << "]\n";
    // std::cout << "  Timestep dt\n    " << dt << " s | Steps: " << steps << "\n";
    // for (int s = 1; s <= steps; ++s) {
    //     PropagateQuaternion(q_kin, omega_kin, dt);
    //     if (s % 2 == 0 || s == steps) {
    //         std::cout << "    t=" << std::fixed << std::setprecision(2) << (s * dt) << "s  q=[" 
    //                 << std::setprecision(6) << q_kin.w << ", " << q_kin.x << ", " << q_kin.y << ", " << q_kin.z << "]\n";
    //     }
    // }
    // double H = ComputeAngularMomentum(omega_kin, inertia_kin);
    // std::cout << "  Angular Momentum |H| [kg*m^2/s]\n    " << std::fixed << std::setprecision(4) << H << "\n";
    // std::array<double, 3> omega_prev = omega_kin;
    // auto alpha = GetAngularAcceleration(omega_prev, omega_kin, dt);
    // std::cout << "  Est. Angular Accelleration [rad/s^2]\n    [" << alpha[0] << ", " << alpha[1] << ", " << alpha[2] << "]\n";
    // double kp = 0.9, kd = 2.5;
    // double margin = ComputeStabilityMargin(err, omega_kin, inertia_kin, kp, kd);
    // std::cout << "  Stability Margin (higher = more robust)\n    " << std::fixed << std::setprecision(2) << margin << "\n";
    // double efficiency = 0.90;
    // double voltage = 28.5;  // bus voltage
    // double power = ComputePowerConsumption(torque, omega_kin, efficiency, voltage);
    // std::cout << "  Est. Actuator Power [W]\n    " << std::fixed << std::setprecision(2) << power << "\n";
    return 0;
}

int handle_telemetry(const std::vector<std::string>& args) {
    std::cout << "---  Telemetry Statistics & Validation Processor  ---\n";
    return 0;
}

int handle_packet(const std::vector<std::string>& args) {
    std::cout << "---  Packet & Protocol Utility  ---\n";
    return 0;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <subcommand> [options...]\n\n";
        std::cerr << "  orbit\t\tOrbital mechanics calculator\n";
        std::cerr << "  attitude\tAttitude representation & kinematics converter\n";
        std::cerr << "  telemetry\tSensor telemetry statistics & validation processor\n";
        std::cerr << "  packet\tSpace packet integrity & protocol utility\n";
        return 1;
    }

    std::string subcmd = argv[1];
    std::vector<std::string> subargs(argv + 2, argv + argc);

    if (subcmd == "orbit") {
        return handle_orbit(argc, argv);
    } else if (subcmd == "attitude") {
        return handle_attitude(argc, argv);
    } else if (subcmd == "telemetry") {
        return handle_telemetry(subargs);
    } else if (subcmd == "packet") {
        return handle_packet(subargs);
    } else {
        std::cerr << "Unknown subcommand: " << subcmd << "\n";
        return 1;
    }
}
