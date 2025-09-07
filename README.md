# RCI KIMM Wheel Legged Robot
This repository contains the URDF model and software for a humanoid robot with wheels on its knees, enabling it to switch between walking and driving modes.

## Overview
This repository contains the software and models for a unique legged robot developed in collaboration with the Korea Institute of Machinery and Materials (KIMM). The robot is a bipedal humanoid with an innovative design: wheels are integrated into its knees, allowing it to seamlessly switch between two modes of locomotion: walking and driving.

## Dependencies
- ROS 2 Humble (>= 2022.05)
- Python 3.10 / C++17

```bash
mkdir -p ~/kimm_wheel_legged_robot_ws/src && cd ~/kimm_wheel_legged_robot_ws/src
git clone
```

## Installation
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/<org>/<repo>.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage
```bash
ros2 launch <package_name> sim.launch.py world:=lab_world
ros2 run <package_name> mpc_controller --ros-args -p horizon:=30
```

## Examples
1. **MPC Tracking** – Humanoid walking in Gazebo  
   ```bash
   ros2 launch mpc_humanoid walking.launch.py
   ```
2. **Impedance Control** – 7-DoF Arm tracking trajectory  
   ```bash
   ros2 launch impedance_control demo.launch.py
   ```

## Citation
If you use this code in your research, please cite:

```bibtex
@inproceedings{kim2025mpc,
  title     = {MPC-based Whole-Body Control for Humanoid Robots},
  author    = {Kim, Sanghyun and Others},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  year      = {2025}
}
```

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact
Maintainer: [이름] (<email>)  
Lab: [RCI Lab @ Kyung Hee University](https://rcilab.khu.ac.kr)
