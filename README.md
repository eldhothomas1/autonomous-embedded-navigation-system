# Autonomous Embedded Navigation System

An embedded robotics platform written in **C** for autonomous coordinate-based navigation, real-time sensor processing, closed-loop motor correction, and low-level hardware control on a **MicroBlaze** soft processor using the AMD/Xilinx Vitis toolchain.

The system was designed to operate without tethered control: target coordinates are entered through hardware switches, converted into encoder-based motion targets, and executed through a finite-state navigation controller. Quadrature encoder feedback continuously monitors left/right wheel motion so the controller can correct drift during translation and turning, while ultrasonic ranging provides real-time environmental sensing.

## System Highlights

- **Embedded C control stack** running on a standalone MicroBlaze platform
- **Finite-state autonomous navigation** for coordinate-based movement and directional transitions
- **Closed-loop wheel correction** using dual quadrature encoder feedback
- **PWM motor control** with independent left/right drive correction
- **Ultrasonic time-of-flight sensing** with hardware-timer-based echo measurement and timeout handling
- **Memory-mapped I/O** for direct interaction with GPIO, motor drivers, switches, LEDs, buttons, and seven-segment displays
- **Encoder-calibrated turning and distance control** for repeatable motion without relying on fixed time delays
- **~98% obstacle-detection accuracy during final system testing**

## Control Architecture

The software is organized around a state-driven navigation loop:

```text
Target Coordinates
       |
       v
+------------------+
| Input / Display  |
+------------------+
       |
       v
+------------------+
| Navigation FSM   |
| start -> move_y  |
| -> turn -> move_x|
| -> end           |
+------------------+
       |
       v
+---------------------------+
| Closed-Loop Motion Control|
| Encoder Feedback + PWM    |
+---------------------------+
       |
       v
+---------------------------+
| Motors / Physical Robot   |
+---------------------------+
```

Rather than driving the motors open-loop for a fixed amount of time, the controller reads both wheel encoders while the robot is moving. If one wheel advances faster than the other, the software selectively throttles that motor until the two sides converge. The same feedback strategy is used during calibrated turns to reduce accumulated heading error.

## Hardware Interfaces

The application communicates directly with memory-mapped peripherals, including:

- Left and right DC motor control channels
- PWM motor-enable signals
- Dual quadrature encoders
- Ultrasonic trigger/echo interface
- Eight hardware timers
- Push buttons and hardware switches
- LEDs and four-digit seven-segment display

The low-level register interface provides deterministic control of the robot without depending on high-level robotics frameworks.

## Ultrasonic Sensing

Distance measurement is implemented as a non-blocking sensor state machine with four phases:

1. Send a timed trigger pulse
2. Wait for the return echo
3. Measure echo pulse duration using hardware timers
4. Enter a cooldown period before the next measurement

Timeout handling prevents the navigation software from hanging when an echo is lost or an object is outside the measurable range.

## Motion Control

The motion subsystem supports:

- Forward travel
- Reverse travel
- Encoder-controlled left and right turns
- Full rotation
- Independent left/right motor correction
- Position targets derived from physical X/Y coordinates

Encoder counts are continuously compared during movement. The faster side is temporarily throttled, providing a lightweight closed-loop controller that compensates for differences between the two motors and improves straight-line tracking.

## Embedded Platform

- **Language:** C
- **Processor:** MicroBlaze
- **Toolchain:** AMD/Xilinx Vitis
- **Build System:** CMake
- **Runtime:** Standalone embedded environment
- **Core Concepts:** Embedded Systems, Finite-State Machines, Closed-Loop Control, PWM, Hardware Timers, Memory-Mapped I/O, Sensor Integration, Quadrature Encoders

## Repository Structure

```text
.
├── src/
│   └── main.c                 # Navigation, sensing, motor control, and hardware I/O
├── CMakeLists.txt             # Vitis/CMake application build configuration
├── Empty_applicationExample.cmake
├── UserConfig.cmake
├── lscript.ld                 # MicroBlaze linker script
├── vitis-comp.json            # Vitis component configuration
├── .clangd                    # Editor/toolchain configuration
└── .gitignore
```

## What This Project Demonstrates

This project combines software and physical-system constraints that are often hidden by higher-level application development: direct register access, real-time sensor timing, feedback-driven actuation, hardware/software debugging, finite-state control, and calibration against imperfect physical components.

The result is a complete embedded navigation stack that translates high-level target coordinates into low-level motor commands while continuously incorporating sensor and encoder feedback.

## Notes

This repository contains the embedded software and build configuration for the completed navigation platform. Running the application requires the corresponding MicroBlaze hardware platform and peripheral mappings used during development.