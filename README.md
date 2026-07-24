# **satutil**  
**Command-line utility for satellite calculations on embedded systems.**

- `satutil` provides lightweight, self-contained orbital mechanics and attitude determination & control (ADCS) calculations suitable for flight software prototyping, ground support tools, and embedded environments.  
- Written in modern C++20 with minimal dependencies.
## **Utility Overview**

- **Binary name**: `satutil`
- **Invocation style**: Subcommands
- **Implementation approach**:
> - Modern C++20, standard library only (`<cmath>`, `<vector>`, `<string>`, `<fstream>`, `<iomanip>`).
> - Core numerical routines in header-only or separate .cpp units that could later be adapted for bare-metal/RTOS targets (static buffers, avoid heavy STL containers in hot paths, consider fixed-point variants later).
> - Robust CLI parsing (manual argv style), clear --help, human + optional JSON output for scripting/CI/test-vector generation.
> - Error handling, input validation, and (TODO) versioning to reflect professional flight-software hygiene.

- **Value for embedded satellite work**: These functions mirror computations that run (or are validated) in flight software — orbit propagation initialization, attitude frame transformations, sensor data quality checks on payloads, and reliable packet handling for downlink/telecommand. Use satutil on the ground to generate test vectors, verify embedded implementations, analyze HIL/sim data, or prototype before optimizing for radiation-hardened targets.

## **Utility Features**
### **Orbital Mechanics (satutil orbit)**
- Circular / elliptical orbit parameters around Earth, Mars, or Moon
- Orbital period, velocity, specific mechanical energy, mean motion, escape velocity
- Hohmann transfer ΔV and transfer time
- Optional JSON output

### **Attitude & Kinematics (satutil attitude)**
- Quaternion, Euler angle, and Direction Cosine Matrix (DCM) conversions
- Angular-rate (ω) kinematic propagation
- Attitude error metrics between reference and measured quaternions
- Full ADCS report on --error:
> - Exact error angle + small-angle norm
> - Roll / Pitch / Yaw error components
> - Error quaternion
> - PD control torque estimate
> - B-dot (magnetorquer) torque estimate
> - Multi-step quaternion propagation
> - Angular momentum |H|
> - Estimated angular acceleration
> - Stability margin
> - Actuator power estimate
> - Optional --omega / --dt override for control & kinematics sections
> - JSON output support

### **Placeholders (future)**
- telemetry — sensor telemetry statistics & validation
- packet — space packet integrity & protocol utility

## **Building the Utility**
### **Requirements**
- C++20 compiler (g++ ≥ 10 or equivalent)
- Standard library only (plus the bundled single-header nlohmann/json)

        cd satutil
        make          # builds ./satutil
        make clean
        make rebuild

### **Compiler Flags**
        -Wall -Wextra -std=c++20 -O2

## **Utility Specifications**
### 1. **Orbital Mechanics Calculator** (`satutil orbit`)  
**Category**: Calculator  
**Synopsis**: Compute fundamental two-body/Keplerian parameters used in GNC initialization, orbit maintenance, and delta-v budgeting.   

#### **Examples**  
- Period (s), circular velocity (m/s), specific energy, etc.
- Hohmann transfer delta-v stubs or escape velocity.

        > satutil orbit --body earth --alt 550000
        > satutil orbit --body mars --hohmann --r1 6778000 --r2 13216400

#### **Why it supports satellite embedded systems**  
- Onboard orbit determination/propagation (even simplified analytic versions) and maneuver planning are core to flight software.
- `satutil` provides a reference implementation and test harness while learning C++ numerical code, that is directly relevant to GNC Engineer and embedded avionics roles.  
  
### 2. **Attitude Representation & Kinematics Converter** (`satutil attitude`)  
**Category**: Number Processor / Calculator  
**Synopsis**: Handle the quaternion/Euler/DCM conversions and basic operations that dominate ADCS flight software (sensor-to-body frames, control torque commands, etc.).  

#### **Examples**

- Normalize quaternion, compute principal rotation angle/axis, or simple propagation step (ω × q).
- Input validation for unit quaternions and singularity warnings (Euler).

        > satutil attitude --q 0.7071 0.0 0.0 0.7071
        > satutil attitude --dcm 1 0 0  0 1 0  0 0 1
        > satutil attitude --euler 0.1 0.2 0.3 --deg
        > satutil attitude --omega 0.001 0.0 -0.0005 --dt 0.1

#### **Why it supports satellite embedded systems**  
- Quaternions are the workhorse representation in embedded ADCS for numerical stability and efficiency.
- Building this demonstrates exactly the 3D math and frame-handling skills hiring managers look for in satellite embedded and GNC candidates.
- Ties very well with sensor fusion and real-time data obtained during flight test.  

### 3. **Sensor Telemetry Statistics & Validation Processor** (`satutil telemetry`)  
**Category**: Number Processor + Text Utility  
**Synopsis**: Parse text/CSV (or simple binary-hex) sensor or payload logs and compute engineering statistics plus basic quality checks.  

#### **Examples**

- `satutil telemetry --input imu_log.csv --cols 2,3,4 --stats mean,std,min,max --moving-avg 10`.  
- Flag outliers or compute simple residual stats (echoing your physics-informed fault detection work).
- Output formatted for quick-look or JSON for automated regression.

#### **Why it supports satellite embedded systems**  
Payloads and avionics generate continuous sensor streams (IMU, magnetometer, GPS, temperature, etc.). Ground support tools and even lightweight onboard processing need reliable stats, validation, and fault flagging. This directly leverages your flight test instrumentation and data acquisition background while giving you text parsing + numerical processing practice in C++.  

### 4. **Space Packet Integrity & Protocol Utility** (`satutil packet`)  
**Category**: Other Utility (Communications / Data Link Support)  
**Synopsis**: Compute standard CRCs used in satellite TM/TC and provide lightweight helpers for packet inspection.  

#### **Examples**

- `satutil packet --crc16-ccitt --hex "1A2B3C4D..."` (or file input).
- CCSDS-style CRC-16-CCITT or CRC-32 (common on many missions).
- Simple bit-field extraction or hex-dump with offset/length for TM frame validation.

#### **Why it supports satellite embedded systems**  
Reliable packet framing, error detection, and sequence checking are fundamental to avionics buses, radio interfaces, and payload data handling. Implementing CRC correctly in C++ shows you understand the data-link realities of flight software — a frequent interview topic and a practical skill for embedded roles.