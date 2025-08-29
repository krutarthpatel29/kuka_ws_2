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
Launch the KUKA robot simulation in Gazebo:
```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch [your_package_name] gazebo_simulation.launch.py

# Terminal 2: Launch RViz for visualization
ros2 launch [your_package_name] rviz.launch.py
```

### Motion Planning with MoveIt!
```bash
# Launch MoveIt! planning interface
ros2 launch [your_package_name] moveit_planning.launch.py
```

### Custom Control
```bash
# Launch custom control nodes
ros2 launch [your_package_name] robot_control.launch.py
```

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
- **Gazebo Integration**: Physics-based simulation
- **World Files**: Custom simulation environments
- **Sensor Integration**: Camera, LiDAR, and force sensors

### Motion Planning
- **MoveIt! Configuration**: Pre-configured motion planning
- **Path Planning**: Collision-free trajectory generation
- **Execution**: Real-time trajectory execution

## Demonstration

### 📊 Presentation
*[Add your presentation file here]*
- **File**: `presentation/KUKA_Project_Presentation.pdf`
- **Description**: Detailed overview of the project, methodology, and results
- **Topics Covered**:
  - Project objectives and scope
  - Technical approach and implementation
  - Simulation results and analysis
  - Future work and improvements

### 🎥 Screen Recording
*[Add your screen recording here]*
- **File**: `demo/KUKA_Demo_Recording.mp4`
- **Description**: Live demonstration of the KUKA robot simulation and control
- **Demonstration Includes**:
  - Robot spawning in Gazebo simulation
  - Motion planning with MoveIt!
  - Trajectory execution and visualization
  - Custom control algorithms in action

#### Quick Demo Preview
```bash
# To reproduce the demo locally:
source install/setup.bash
ros2 launch [main_package] full_demo.launch.py
```

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

3. **Gazebo Issues**:
   ```bash
   # Reset Gazebo
   killall gzserver gzclient
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
- KUKA iiwa 7 R800
- KUKA iiwa 14 R820
- KUKA LBR Med series
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
- Email: [your.email@example.com]

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

# Launch planning interface
ros2 launch [your_main_package] moveit.launch.py
```

---

*For detailed documentation and tutorials, please refer to the individual package README files in the src/ directory.*
