#include <iostream>
#include <numbers>
#include <string>
#include <vector>
#include <cstdlib>
#include <iomanip>
#include "includes/utility.h"
#include "includes/json.hpp"
using json = nlohmann::json;


int handle_orbit(int argc, char* argv[]) {
    // CLI: satutil orbit --body earth --alt 550000
    // CLI: satutil orbit --body mars --hohmann --r1 6778000 --r2 13216400
    std::cout << "===============================================================" << std::endl;
    std::cout << "-=-+-=-+-=-+-=-  Orbital Mechanics Calculator  -=-+-=-+-=-+-=-\n";
    std::cout << "===============================================================" << std::endl;
    auto [body, radius, sma, alt, mu, use_hohmann, r1, r2, json_output, help] = getOrbitalParams(argc, argv);
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

int handle_attitude(const std::vector<std::string>& args) {
    std::cout << "---  Attitude & Kinematics Converter  ---\n";
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
        return handle_attitude(subargs);
    } else if (subcmd == "telemetry") {
        return handle_telemetry(subargs);
    } else if (subcmd == "packet") {
        return handle_packet(subargs);
    } else {
        std::cerr << "Unknown subcommand: " << subcmd << "\n";
        return 1;
    }
}