# **satutil**  
A single, cohesive C++ command-line utility that consolidates four high-value functionalities directly supporting satellite embedded systems development for avionics and payload computations.
This tool is designed as a strategic career accelerator for your pivot into satellite GNC/ADCS, embedded avionics, and payload roles (e.g., at Muon Space, Stoke Space, NASA, or similar). It builds on your flight test instrumentation, real-time sensor/telemetry, and physics-informed modeling background while giving you concrete C++ practice (aligning with your current Coursera progress) and portfolio evidence of domain-relevant numerical and systems programming skills. The core math and logic can be structured in reusable headers that demonstrate an embedded mindset (minimal dependencies, predictable resource use, testable in isolation).
## **Tool Overview**

- **Binary name**: `satutil`
- **Invocation style**: Subcommands (like `git` or common embedded dev tools) for clean extensibility.
- **Implementation approach**:
> - Modern C++17/20, standard library only (`<cmath>`, `<vector>`, `<string>`, `<fstream>`, `<iomanip>`).
> - Core numerical routines in header-only or separate .cpp units that could later be adapted for bare-metal/RTOS targets (static buffers, avoid heavy STL containers in hot paths, consider fixed-point variants later).
> - Robust CLI parsing (manual argv or lightweight getopt-style), clear --help, human + optional JSON output for scripting/CI/test-vector generation.
> - Error handling, input validation, and versioning to reflect professional flight-software hygiene.

- **Value for embedded satellite work**: These functions mirror computations that run (or are validated) in flight software — orbit propagation initialization, attitude frame transformations, sensor data quality checks on payloads, and reliable packet handling for downlink/telecommand. Use satutil on the ground to generate test vectors, verify embedded implementations, analyze HIL/sim data, or prototype before optimizing for radiation-hardened targets.

## **The 4 Utility Functionalities**
1. **Orbital Mechanics Calculator** (`satutil orbit`)  
    **Category**: Calculator  
    **Synopsis**: Compute fundamental two-body/Keplerian parameters used in GNC initialization, orbit maintenance, and delta-v budgeting.   
  
    **Examples**  
    - `satutil orbit --sma 7000000 --mu 3.986004418e14`  
    period (s), circular velocity (m/s), specific energy, etc.
    - `satutil orbit --period 5400 --alt 500e3`  
    solve for SMA or eccentricity cases.
    - Hohmann transfer delta-v stubs or escape velocity.

    **Why it supports satellite embedded systems**  
    Onboard orbit determination/propagation (even simplified analytic versions) and maneuver planning are core to flight software. This gives you a reference implementation and test harness while you learn C++ numerical code — directly relevant to GNC Engineer and embedded avionics roles.  
  
2. **Attitude Representation & Kinematics Converter** (`satutil attitude`)  
    **Category**: Number Processor / Calculator  
    **Synopsis**: Handle the quaternion/Euler/DCM conversions and basic operations that dominate ADCS flight software (sensor-to-body frames, control torque commands, etc.).  
    
    **Examples**

    - `satutil attitude --quat 0.7071,0,0,0.7071 --to euler-321` (or `--to dcm`).  
    - Normalize quaternion, compute principal rotation angle/axis, or simple propagation step (ω × q).
    - Input validation for unit quaternions and singularity warnings (Euler).
  
    **Why it supports satellite embedded systems**  
    Quaternions are the workhorse representation in embedded ADCS for numerical stability and efficiency. Building this demonstrates exactly the 3D math and frame-handling skills hiring managers look for in satellite embedded and GNC candidates. Ties beautifully to your sensor fusion and real-time data experience from Boeing flight test.  
  
3. **Sensor Telemetry Statistics & Validation Processor** (`satutil telemetry`)  
    **Category**: Number Processor + Text Utility  
    **Synopsis**: Parse text/CSV (or simple binary-hex) sensor or payload logs and compute engineering statistics plus basic quality checks.  

    **Examples**

    - `satutil telemetry --input imu_log.csv --cols 2,3,4 --stats mean,std,min,max --moving-avg 10`.  
    - Flag outliers or compute simple residual stats (echoing your physics-informed fault detection work).
    - Output formatted for quick-look or JSON for automated regression.

    **Why it supports satellite embedded systems**  
    Payloads and avionics generate continuous sensor streams (IMU, magnetometer, GPS, temperature, etc.). Ground support tools and even lightweight onboard processing need reliable stats, validation, and fault flagging. This directly leverages your flight test instrumentation and data acquisition background while giving you text parsing + numerical processing practice in C++.  

4. **Space Packet Integrity & Protocol Utility** (`satutil packet`)  
    **Category**: Other Utility (Communications / Data Link Support)  
    **Synopsis**: Compute standard CRCs used in satellite TM/TC and provide lightweight helpers for packet inspection.  

    **Examples**

    - `satutil packet --crc16-ccitt --hex "1A2B3C4D..."` (or file input).
    - CCSDS-style CRC-16-CCITT or CRC-32 (common on many missions).
    - Simple bit-field extraction or hex-dump with offset/length for TM frame validation.

    **Why it supports satellite embedded systems**  
    Reliable packet framing, error detection, and sequence checking are fundamental to avionics buses, radio interfaces, and payload data handling. Implementing CRC correctly in C++ shows you understand the data-link realities of flight software — a frequent interview topic and a practical skill for embedded roles.