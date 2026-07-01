#pragma once
#include <string>

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

orbitParams getOrbitalParams(int argc, char* argv[]);

double OrbitalPeriod (double a, double mu);

double OrbitalVelocity(double r, double mu);

double SpecificMechanicalEnergy(double a, double mu);

double MeanMotion(double a, double mu);

double EscapeVelocity(double r, double mu);

double altitudetoRadius(double r, double alt);

void HohmannTransfer(double r1, double r2, double mu);
