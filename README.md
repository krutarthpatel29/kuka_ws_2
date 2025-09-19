# KUKA Workspace 2 (kuka_ws_2)

## Overview
This repository contains a ROS/ROS2 workspace for KUKA robot development, simulation, and control. The workspace includes packages for robot modeling, simulation, motion planning, and control algorithms.

## Table of Contents
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Package Structure](#package-structure)
- [Demonstration](#demonstration)
- [Contributing](#contributing)
- [License](#license)

## Features
- KUKA robot simulation in Gazebo
- MoveIt! integration for motion planning
- Robot control and trajectory execution
- Custom launch files for different scenarios
- Visualization tools in RViz

## Prerequisites

### System Requirements
- Ubuntu 20.04 LTS or Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill (recommended) or ROS2 Galactic
- Python 3.8+

### Dependencies
```bash
# Core ROS2 packages
sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-moveit
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-xacro

# Build tools
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
```

## Installation

### 1. Clone the Repository
```bash
# Create workspace directory
mkdir -p ~/kuka_ws_2
cd ~/kuka_ws_2

# Clone this repository
git clone https://github.com/krutarthpatel29/kuka_ws_2.git .
```

### 2. Install Dependencies
```bash
# Initialize rosdep (if not already done)
sudo rosdep init
rosdep update

# Install package dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Simulation
follow run script to run hybrid communication frmework step by step 

## Package Structure
```
kuka_ws_2/
├── src/
│   ├── [package_1]/              # Robot description package
│   │   ├── urdf/                 # Robot URDF files
│   │   ├── meshes/               # 3D mesh files
│   │   ├── launch/               # Launch files
│   │   └── config/               # Configuration files
│   ├── [package_2]/              # Control package
│   │   ├── src/                  # Source code
│   │   ├── include/              # Header files
│   │   └── config/               # Control parameters
│   └── [package_3]/              # Custom algorithms/applications
├── README.md
└── package.xml
```

## Key Components

### Robot Description
- **URDF Models**: Detailed robot models with accurate kinematics
- **Mesh Files**: High-quality 3D meshes for visualization
- **Joint Configurations**: Proper joint limits and dynamics

### Simulation Environment
- **Rviz Integration**: Physics-based simulation
- **World Files**: Custom simulation environments
- **Sensor Integration**: Camera, LiDAR, and force sensors

### Motion Planning
- **MoveIt! Configuration**: Pre-configured motion planning
- **Path Planning**: Collision-free trajectory generation
- **Execution**: Real-time trajectory execution


## MSc Thesis Project: Multi-Robot Coordination Using ROS2

This repository also contains my **MSc thesis project** at the University of Stuttgart on multi-robot coordination with KUKA robots and ROS2. The project focuses on real-time communication, task execution, and integration of PLCs with AI-based monitoring for industrial 4.0 environments.

### Thesis PDF
- [Download Thesis PDF](Krutarth_Patel_MT_Thesis_v1.2.pdf) – Full 92-page MSc thesis detailing methodology, experiments, and results.

### Demonstration Videos
- [Multi-Robot Demo](videos/Multi_robot_demo.MP4) – Shows multi-robot coordination in simulation with ROS2.
- [Single-Robot Demo](videos/Single_robot_demo.MP4) – Shows single-robot tasks in simulation with ROS2.

> These materials provide a complete overview of my MSc thesis project and the hybrid communication framework developed for multi-robot systems.

## Demonstration

For the MSc thesis project, see the demonstration videos above in the **MSc Thesis Project** section.



## Troubleshooting

### Common Issues
1. **Build Errors**:
   ```bash
   # Clean and rebuild
   rm -rf build/ install/ log/
   colcon build
   ```

2. **Missing Dependencies**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Development

### Adding New Packages
```bash
# Create new package
cd src/
ros2 pkg create --build-type ament_cmake your_package_name
```

### Testing
```bash
# Run tests
colcon test
colcon test-result --verbose
```

## Performance Metrics
- **Simulation Rate**: 1000 Hz physics update
- **Control Frequency**: 100 Hz
- **Planning Time**: < 5 seconds for typical trajectories
- **Execution Accuracy**: ±0.1mm positioning accuracy

## Hardware Compatibility
- KUKA KR360 R2830
- Custom KUKA configurations

## Future Enhancements
- [ ] Real robot hardware integration
- [ ] Advanced collision detection
- [ ] Machine learning-based control
- [ ] Multi-robot coordination
- [ ] Industrial automation scenarios

## Contributing
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-feature`)
3. Commit changes (`git commit -am 'Add new feature'`)
4. Push to branch (`git push origin feature/new-feature`)
5. Create Pull Request

## Author
**Krutarth Patel**
- GitHub: [@krutarthpatel29](https://github.com/krutarthpatel29)
- Email: [krutarthp81@gmail.com]

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments
- ROS2 Community for excellent documentation
- MoveIt! team for motion planning framework
- KUKA for robot specifications and documentation
- Open source robotics community

---

## Quick Start Commands
```bash
# One-command setup (after cloning)
source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y && colcon build && source install/setup.bash

# Launch simulation
ros2 launch [your_main_package] simulation.launch.py


---

*For detailed documentation and tutorials, please refer to the individual package README files in the src/ directory.*
