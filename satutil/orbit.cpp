/*
Focused on circular and elliptical orbits around a central body (primarily Earth by default)
*/
#include "includes/orbital_utility.h"
#include <numbers>
#include <cmath>
#include <string>
#include <iostream>
#include <vector>
#include <iomanip>

// ============= PARSING =============
orbitParams getOrbitalParams(int argc, char* argv[]) {
    orbitParams params;
    params.body = "earth";  // default value
    params.mu = satutil::MU_EARTH;  // default value
    params.radius = satutil::R_EARTH;  //default value
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--body" && i+1 < argc) {
            std::string body = argv[++i];
            params.body = body;
            if (body == "earth" || body == "Earth") {
                params.mu = satutil::MU_EARTH;
                params.radius = satutil::R_EARTH;
            } else if (body == "mars" || body == "Mars") {
                params.mu = satutil::MU_MARS;
                params.radius = satutil::R_MARS;
            } else if (body == "moon" || body == "Moon") {
                params.mu = satutil::MU_MOON;
                params.radius = satutil::R_MOON;
            }
        } else if (arg == "--alt" && i+1 < argc) {
            params.alt = std::stod(argv[++i]);
        } else if (arg == "--hohmann") {
            params.use_hohmann = true;
            // expect --r1 and --r2 next or handle separately
        } else if (arg == "--r1" && i+1 < argc) {
            params.r1 = std::stod(argv[++i]);
        } else if (arg == "--r2" && i+1 < argc) {
            params.r2 = std::stod(argv[++i]);
        } else if (arg == "--json") {
            params.json_output = true;
        } else if (arg == "--help") {
            // show_orbit_help();
            params.str_help = "Usage: satutil orbit [--body <'earth'|'moon'|'mars'>] [--sma <meters>] [--alt <meters>] [--hohmann --r1 <meters> --r2 <meters>] [--json]";
        } else {
            std::cout << "Unknown flag: " << arg << std::endl;
        }
    }
    // derive the SMA
    if (params.alt > 0.0) {
        params.sma = params.radius + params.alt;
    } else if (params.alt == 0.0) {
        params.alt = params.radius;
    }
    else if (params.sma == 0.0) {
        params.sma = params.radius;
    }
    
    return params;
}

// ============= ORBITAL CALCULATIONS =============
double OrbitalPeriod (double a, double mu) {
    /* 
        Calculate orbital period for a circular or elliptical orbit
        a = semi-major axis in meters
        Returns period in seconds
    */
    return 2.0 * std::numbers::pi * std::sqrt((a * a * a) / mu);  // seconds
}

double OrbitalVelocity(double r, double mu) {
    /* 
        Calculate circular orbital velocity
        r = radius (or semi-major axis for circular orbits) in meters
        Returns velocity in m/s
    */
    return std::sqrt(mu / r);  // meters per second
}

double SpecificMechanicalEnergy(double a, double mu) {
    /* 
        Specific Mechanical Energy (ε) for an orbit
        a = semi-major axis in meters
        Returns energy in J/kg (negative for bound/closed orbits)
    */
    return -mu / (2.0 * a);  // negative for bound orbits
}

double MeanMotion(double a, double mu) {
    /* Calculate the mean motion */
    return 2 * std::numbers::pi / OrbitalPeriod(a, mu);
}

double EscapeVelocity(double r, double mu) {
    /* 
        Escape velocity from a given radius
        r = distance from center of Earth in meters
        Returns velocity in m/s
    */
    return std::sqrt(2.0 * mu / r);
}

double altitudetoRadius(double r, double alt) {
    /* Convert thealtitude to radius */
    return r + alt;
}

void HohmannTransfer(double r1, double r2, double mu) {
    /*
    Calculate the Hohmann transfer parameters;
    the transfer between 2 circular orbits;
    Required: r1 < r2
    */
    if (r1 <= 0.0 || r2 <= 0.0 || r1 >= r2) {
        std::cout << "   Error: Invalid radii for Hohmann transfer (r1 < r2 required)\n";
        return;
    } else {
        // Transfer semi-major axis:
        double a_trans = (r1 + r2) / 2.0;
        // First burn (at r1):
        double delta_v1 = std::sqrt(mu * ((2.0 / r1) - (1.0 / a_trans))) - std::sqrt(mu / r1);
        // Second burn (at r2):
        double delta_v2 = std::sqrt(mu / r2) - std::sqrt(mu * ((2.0 / r2) - (1.0 / a_trans)));
        // Transfer time: half the period of the ellipse
        double t_transfer = std::numbers::pi * std::sqrt(a_trans * a_trans * a_trans / mu);  // seconds
        
        std::cout << "  Hohmann Transfer Params\t\t" << std::endl;
        std::cout << std::fixed << std::setprecision(2) << "    Transfer SMA\t\t" << a_trans * 10e-4 << " km" << std::endl;
        std::cout << std::fixed << std::setprecision(2) << "    delta_v1 (at r1)\t\t" << delta_v1 << " m/s" << std::endl;  // meters per second
        std::cout << std::fixed << std::setprecision(2) << "    delta_v2 (at r2)\t\t" << delta_v2 << " m/s" << std::endl;  // meters per second
        std::cout << std::fixed << std::setprecision(2) << "    Transfer Time\t\t" << t_transfer / 3600.0 << " hr" << std::endl;
    }
}
