# <img src="vizanti_server/public/assets/icon/512.png" alt="Icon" title="Grid" width="50" height="50"/> Vizanti Ostirion - Web Visualizer & Mission Planner for ROS 2

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.png)](https://opensource.org/licenses/BSD-3-Clause)

Vizanti Ostirion is an Ostirion-maintained ROS 2 browser interface for robot visualization, monitoring, mission planning, and controlled robot operation.

The project builds on the original Vizanti concept created by Vid Rijavec / MoffKalast: a browser-based interface inspired by RViz for interacting with robots from desktop and mobile browsers.

Vizanti Ostirion preserves the BSD 3-Clause license and original attribution while documenting the Ostirion-specific runtime, binary transport, deployment, security, and Nav2 integration work.

<img src="vizanti_server/public/assets/icon/preview.jpg" alt=""/> 

## Installation

This repository is intended for ROS 2 Humble on Ubuntu 22.04 and ROS 2 Jazzy on Ubuntu 24.04.

```bash
cd ~/colcon_ws/src
git clone https://github.com/Ostirion-net/vizanti_ostirion.git vizanti

cd ..
rosdep install -i --from-path src/vizanti -y
colcon build
source install/setup.bash
