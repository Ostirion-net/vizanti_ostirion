# <img src="vizanti_server/public/assets/icon/512.png" alt="Icon" title="Grid" width="50" height="50"/> Vizanti Ostirion - Web Visualizer & Mission Planner for ROS 2

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.png)](https://opensource.org/licenses/BSD-3-Clause)

Vizanti Ostirion is a web-based visualization, monitoring, and mission planning tool for robots running ROS 2.

The project builds on the original Vizanti concept created by Vid Rijavec / MoffKalast: a browser-based robot interface inspired by RViz, designed to provide a convenient 2D visualization and control surface from desktop and mobile browsers.

Vizanti Ostirion preserves the original BSD 3-Clause license and attribution while adding Ostirion-specific runtime, transport, deployment, security, and Nav2 integration work for ROS 2 robot systems.

The application provides a smartphone-friendly interface for maps, TF, scans, odometry, paths, markers, images, goals, waypoints, parameters, buttons, robot status, and mission interaction from a web browser.

<img src="vizanti_server/public/assets/icon/preview.jpg" alt=""/> 

## Installation

This repository is intended for [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) on Ubuntu 22.04 and [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) on Ubuntu 24.04.

For the original upstream Vizanti project, see [MoffKalast/vizanti](https://github.com/MoffKalast/vizanti).

```bash
cd ~/colcon_ws/src
git clone https://github.com/Ostirion-net/vizanti_ostirion.git vizanti

cd ..
rosdep install -i --from-path src/vizanti -y
colcon build
source install/setup.bash
```

### Docker

Alternatively, you can also containerize Vizanti Ostirion.

Set `ROS_VERSION` to either `humble` or `jazzy`.

```bash
git clone https://github.com/Ostirion-net/vizanti_ostirion.git vizanti
cd vizanti
docker build -f docker/Dockerfile -t vizanti:ostirion . --build-arg ROS_VERSION=$ROS_DISTRO
```

## Run

```bash
ros2 launch vizanti_server vizanti_server.launch.py
```

Or with Docker:

```bash
docker run --rm -it --net=host --name vizanti-ros2 \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  -e RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION \
  -v /dev/shm:/dev/shm \
  vizanti:ostirion
```

The web app can be accessed at `http://<host_ip>:5000`.

By default, the web application is served on port `5000` and the Vizanti Ostirion WebSocket bridge uses port `5001`.

Client settings are saved in browser localStorage. Offline map tiles and satellite imagery can also use browser-side storage, depending on the selected widgets and configuration.

If you're using a mobile device connected to a robot hotspot without internet access and the page does not load correctly, disable mobile data. This prevents the browser from sending packets through the wrong gateway.

## Ostirion Runtime

The standard Ostirion launch path starts the Vizanti web server, TF consolidator, service handler, and Nav2 bridge.

Vizanti Ostirion uses a binary WebSocket bridge between the browser and the ROS 2 server. This bridge handles topic discovery, subscriptions, serialized ROS 2 message forwarding, and selected browser-side publishing.

The server subscribes to ROS 2 topics using raw serialized message transport and forwards serialized payloads to the browser.

The browser decodes selected serialized ROS 2 messages using CDR decoding support from the bundled `zenoh_foxglove_bundle.js` module.

The server also accepts selected browser-published messages, deserializes them into ROS 2 messages, and publishes them into the ROS 2 graph.

### Supported Decoded Messages

The browser-side bridge includes direct decoding support for selected ROS 2 message families:

```text
tf2_msgs/msg/TFMessage
sensor_msgs/msg/LaserScan
nav_msgs/msg/OccupancyGrid
nav_msgs/msg/Path
sensor_msgs/msg/CompressedImage
sensor_msgs/msg/Imu
sensor_msgs/msg/BatteryState
```

### Supported Browser-Published Messages

The browser-side bridge includes publishing support for selected command and goal messages:

```text
geometry_msgs/msg/Twist
geometry_msgs/msg/PoseStamped
```

## Nav2 Integration

Vizanti Ostirion includes a Nav2 mission bridge based on `nav2_simple_commander`.

The bridge listens for waypoint arrays from the Vizanti browser interface, converts them to stamped Nav2 goals, and dispatches them through the Nav2 `BasicNavigator`.

The bridge supports looped patrol behavior by reversing the waypoint order after a successful mission when loop mode is enabled.

## Web and Remote Deployment

Vizanti Ostirion supports deployment behind controlled web infrastructure.

The browser selects `ws://` or `wss://` according to the page protocol, allowing operation behind HTTP or HTTPS front ends.

The server exposes separate internal and client-facing WebSocket port parameters, supporting deployments where a proxy exposes a different public port than the robot-side process.

For remote or internet-reachable deployments, place Vizanti behind the robot system access-control layer, such as VPN access, TLS termination, authenticated reverse proxy, firewall rules, private routing, or container-level network isolation.

## Security

Vizanti Ostirion includes server-side hardening for robot-facing web operation.

The service handler validates node names before process operations, restricts server-started process requests to structured `ros2 run` and `ros2 launch` forms, checks that requested packages exist, and starts subprocesses with structured argument lists.

Map and bag operations resolve user-provided paths and restrict file access to the user home directory.

Bag recording validates requested topic names before passing them to `ros2 bag record`.

The web server limits uploaded configuration size and exposes configuration save/load endpoints for browser layouts.

## Repository Layout

```text
vizanti/
  Top-level ROS 2 package entry point.

vizanti_server/
  Web server, browser client, launch files, widgets, WebSocket bridge,
  service handlers, and Nav2 integration.

vizanti_msgs/
  Custom ROS 2 service definitions used by Vizanti.

vizanti_cpp/
  C++ support nodes, including TF consolidation.

vizanti_demos/
  Demo and test publishers for widgets, maps, markers, parameters, paths, and TF.
```

## Backend Notes

The repository contains rosbridge and RWS-related package, launch, and Docker material.

The standard Ostirion server launch path uses the Vizanti Ostirion binary WebSocket bridge.

## Attribution and License

Vizanti Ostirion is derived from the original Vizanti project by Vid Rijavec / MoffKalast.

Original project:

```text
https://github.com/MoffKalast/vizanti
```

This repository preserves the BSD 3-Clause license and original attribution while maintaining Ostirion-specific ROS 2 development.

## Contributing

Please see [Contributing.md](Contributing.md) for more information.
