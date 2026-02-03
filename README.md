# Drone Autonomous Mission Project

## Overview

This project is a simulation and control system developed for TEKNOFEST 25/26 Rotary-Winged UAV Category. The project includes color perception-based tasks, state machine management, and security protocols.

## Features

- 🎯 Multi-mission autonomous drone operations
- 🔍 Computer vision based object detection (red, blue, white)
- 🛡️ Failsafe mechanisms and emergency protocols
- 🗺️ State machine based mission management
- 🌍 Gazebo simulation environment
- 📊 Configurable parameters and scenarios

## Project Structure

```
.
├── 📁 config/                           # Configuration files
│   ├── drone_params.yaml               # Drone parameters and settings
│   └── world_params.yaml               # World/environment parameters
│
├── 📁 docs/                             # Documentation
│   ├── 📁 setup/                       # Setup guides
│   │   ├── ubuntu_22.04.md            # Ubuntu 22.04 installation guide
|   |   └── ubuntu_24.04.md            # Ubuntu 24.04 installation guide
│   ├── architecture.md                 # System architecture documentation
│   ├── checklist.md                    # Development and testing checklist
│   └── simulation_notes.md             # Simulation setup notes
│
├── 📁 environments/                     # Gazebo simulation environments
│   ├── 📁 models/                      # 3D models for simulation
│   │   ├── 📁 blue_altigen/            # Blue hexagon shaped objects
│   │   ├── 📁 blue_square/             # Blue square objects
│   │   ├── 📁 blue_triangle/           # Blue triangle objects
│   │   ├── 📁 red_altigen/             # Red hexagon shaped objects
│   │   ├── 📁 red_square/              # Red square objects
│   │   ├── 📁 red_triangle/            # Red triangle objects
│   │   └── 📁 long_cylinder/           # Cylindrical objects
│   ├── 📁 worlds/                      # Gazebo world files
│   │   └── iris_arducopter_runway.world # Main simulation world
│   └── how-to-use.md                   # Environment usage guide
│
├── 📁 scenarios/                        # Mission scenarios
│   ├── 📁 comm_loss/                   # Communication loss scenarios
│   ├── 📁 gps_loss/                    # GPS loss scenarios
│   ├── 📁 normal_mission/              # Normal mission scenarios
│   │   ├── 📁 first/                   # First mission scenario
│   │   ├── 📁 second/                  # Second mission scenario
│   │   └── 📁 third/                   # Third mission scenario
│   └── 📁 weather_degraded/            # Weather degraded scenarios
│
├── 📁 src/                              # Source code
│   ├── 📁 app/                         # Application entry point
│   │   └── run.py                      # Main application runner
│   └── 📁 core/                        # Core functionality modules
│       ├── 📁 detection/               # Object detection algorithms
│       │   ├── blue_detection.py       # Blue object detection
│       │   ├── red_detection.py        # Red object detection
│       │   └── white_detection.py      # White object detection
│       ├── 📁 failsafe/                # Safety and failsafe mechanisms
│       │   └── failsafe_example.py     # Failsafe implementation example
│       ├── 📁 missions/                # Mission definitions
│       │   ├── first_mission.py        # First mission implementation
│       │   ├── second_mission.py       # Second mission implementation
│       │   └── third_mission.py        # Third mission implementation
│       └── 📁 state-machine/           # State machine management
│           ├── states.py               # State definitions
│           └── state_machine.py        # State machine logic
│
├── requirements.txt                     # Python dependencies
└── README.md                           # This file
```

## Prerequisites

- Ubuntu 22.04 LTS or Ubuntu 24.04 LTS
- Python 3.8+
- ROS 2 (Humble recommended)
- Gazebo simulation environment
- ArduPilot SITL

## Installation

1. **Clone the repository:**

   ```bash
   git clone <repository-url>
   cd <project-directory>
   ```

2. **Install Python dependencies:**

   ```bash
   pip install -r requirements.txt
   ```

3. **Setup simulation environment:**
   - Follow one of the guides in `docs/setup/`
     ( Simulation is built on 24.04 LTS )
   - Configure Gazebo models and worlds

## Usage

### Running the Application

```bash
# Navigate to source directory
cd src/app

# Run the main application
python run.py
```

### Configuration

- Modify `config/drone_params.yaml` for drone-specific settings
- Adjust `config/world_params.yaml` for environment parameters
- Select scenarios from the `scenarios/` directory

### Mission Modes

- **First Mission**: Basic navigation and object detection
- **Second Mission**: Advanced maneuvers and multi-target tracking
- **Third Mission**: Complex autonomous operations

## Architecture

The system follows a modular architecture:

- **Detection Module**: Computer vision algorithms for object recognition
- **State Machine**: Mission flow control and state management
- **Mission Manager**: High-level mission logic and coordination
- **Failsafe System**: Emergency protocols and safety mechanisms

## Development

### Adding New Missions

1. Create mission file in `src/core/missions/`
2. Define states in `src/core/state-machine/states.py`
3. Add scenario configuration in `scenarios/`
4. Update documentation

### Testing

- Use different scenarios in `scenarios/` directory
- Test failsafe mechanisms under various conditions
- Validate detection algorithms with different object configurations

## Documentation

- 📖 [System Architecture](docs/architecture.md)
- ⚙️ [Setup Guide](docs/setup/ubuntu_22.04.md)
- ✅ [Development Checklist](docs/checklist.md)
- 🔧 [Simulation Notes](docs/simulation_notes.md)
- 🌍 [Environment Usage](environments/how-to-use.md)

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

MIT

## Support

For questions and support, please refer to the documentation or open an issue in the repository.



