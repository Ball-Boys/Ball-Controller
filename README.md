# Ball-Controller

ESP32-based magnetic ball control system with IMU feedback and real-time telemetry.

## Overview

This repository contains two main components:
1. **ESP Controller** - Embedded firmware for ESP32 that controls magnetic actuators based on IMU sensor data
2. **Telemetry Dashboard** - Real-time monitoring and control interface (TBD)

## Repository Structure

```
Ball-Controller/
├── esp-controller/          # ESP32 firmware
└── telemetry-dashboard/     # Web-based dashboard (TBD)
```

## ESP Controller Architecture

### Directory Layout

```
esp-controller/
├── platformio.ini           # PlatformIO configuration
├── include/                 # Auto-generated headers
├── lib/                     # Third-party libraries
├── src/                     # Source code
├── test/                    # Unit tests
└── scripts/                 # Build and deployment scripts
```

### Source Code Modules

#### `src/core/`
**Global State Management**
- `global_state.h/cpp` - Thread-safe singleton for shared memory between tasks
- Provides mutex-protected access to sensor data, control outputs, and system state
- Central communication hub for all system components

#### `src/control/`
**Control Algorithm**
- `control_algorithm.h/cpp` - Standalone control logic module
- **Inputs:** Current orientation (q), ideal direction vector
- **Outputs:** Magnet steering commands (which magnets to activate)
- Pure algorithmic component with no external dependencies

#### `src/tasks/`
**FreeRTOS Task Implementations**

Tasks are continuous loops running on separate cores:

- **`imu_task.h/cpp`** - IMU Polling Loop
  - Polls external IMU sensor continuously
  - Updates global state with orientation data
  - Triggers control algorithm execution
  - Stores computed steering values to global state

- **`magnet_task.h/cpp`** - Magnet Controller Loop
  - Reads steering commands from global state
  - Actuates physical magnets
  - Writes current magnet status back to global state

- **`comms_task.h/cpp`** - Communication Loop
  - Sends telemetry data to external dashboard (WiFi/Bluetooth - TBD)
  - Receives steering commands from external sources
  - Writes received commands to global state
  - No direct interaction with other tasks (state-mediated only)

#### `src/calibration/`
**System Initialization**
- `calibration.h/cpp` - One-time startup calibration routine
- Interfaces with communication module to receive calibration commands
- Sets initial orientation/zero-point for the ball
- Handles peripheral initialization and setup
- Runs once at startup before main task loops begin

#### `src/ota/`
**Over-The-Air Updates**
- `ota_update.h/cpp` - Remote code flashing capability (TBD)
- Leverages communication module infrastructure
- Allows firmware updates without physical access

#### `src/main.cpp`
**Entry Point**
- System initialization and startup sequence
- Spawns FreeRTOS tasks for all continuous loops
- Manages task lifecycle and error handling

## Design Principles

### Thread Safety
All shared state access is protected by FreeRTOS mutexes to ensure safe concurrent access from multiple tasks.

### Separation of Concerns
- Tasks communicate **only** through global state
- No direct task-to-task interaction
- Each module has a single, well-defined responsibility

### Modularity
- Control algorithm is standalone and testable independent of hardware
- Tasks are independent loops with clear interfaces
- Calibration is isolated from runtime logic

## Getting Started

### Prerequisites
- PlatformIO IDE or CLI
- ESP32 development board
- IMU sensor (model TBD)
- Magnetic actuators

### Building
```bash
cd esp-controller
pio run
```

### Flashing
```bash
pio run --target upload
```

### Testing
```bash
pio test
```

## Contributing

### Where to Add New Code

- **New sensor integration?** → Create new task in `src/tasks/`
- **New control logic?** → Modify `src/control/control_algorithm.cpp`
- **New state variables?** → Update `src/core/global_state.h/cpp`
- **Startup configuration?** → Modify `src/calibration/`
- **Communication protocol changes?** → Update `src/tasks/comms_task.cpp`
- **Test scripts?** → Add to `test/` directory

### Naming Conventions
- Files: `snake_case.cpp/h`
- Classes: `PascalCase`
- Functions/variables: `camelCase`

## Telemetry Dashboard

Dashboard implementation is currently in planning. Structure and technology stack TBD.

## License

[TBD]