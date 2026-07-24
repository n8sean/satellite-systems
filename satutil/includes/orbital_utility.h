#pragma once
#include <string>
#include <numbers>

namespace satutil {
    // Phyiscal constants
    constexpr double MU_EARTH = 3.986004418e14;  // standard gravitational parameter; m³/s² for Earth
    constexpr double MU_MARS = 4.2828e13;
    constexpr double MU_MOON = 4.9048695e12;
    constexpr double R_EARTH = 6378137.0;  // meters
    constexpr double R_MARS = 3389500.0;  // meters
    constexpr double R_MOON = 1737400;  // meters
}

struct orbitParams {
    std::string body = "Earth";
    double radius = 0.0;
    double sma = 0.0;
    double alt = 0.0;
    double mu = 0.0;
    bool use_hohmann = false;
    double r1 = 0.0;
    double r2 = 0.0;
    bool json_output = false;
    std::string str_help = "";
};

orbitParams GetOrbitalParams(int argc, char* argv[]);

double OrbitalPeriod (double a, double mu);

double OrbitalVelocity(double r, double mu);

double SpecificMechanicalEnergy(double a, double mu);

double MeanMotion(double a, double mu);

double EscapeVelocity(double r, double mu);

double altitudetoRadius(double r, double alt);

void HohmannTransfer(double r1, double r2, double mu);

std::string GetOrbitHelp();