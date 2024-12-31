# Automata Deployment Tools ROS2

Automata Deployment Tools ROS2 is a ros2 package used to develop and deploy edge robotics in a minimal, autonomous framework.

## Installation
1. Install the AARDK prerequisities.
2. Clone the AARDK.
5. Clone the project under ./Projects/Deployment.
3. Run ./CC.sh -b auv-deployment.
4. Run ./CC.sh -s auv-deployment.
6. Compile the project with the following command.

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --parallel-workers $(nproc) \
    --symlink-install --event-handlers console_direct+ --base-paths src --cmake-args \
    ' -DCMAKE_BUILD_TYPE=Debug' ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
    ' -DCMAKE_CXX_FLAGS=-Wall -Wextra -Wno-unused-parameter' ' --no-warn-unused-cli' \
    --packages-select automata_deployment_tools_ros2 && source install/setup.bash
```

## Usage
If you want to visualize data collection and control the device via GUI, install
Foxglove Studio using the following command.

```bash
sudo snap install foxglove-studio #x86_64 only
```

### AUVMonitor
Run the following command to start the AUV Monitor, the main entrypoint for the program

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash && source install/setup.bash && \
    ros2 run auv_deployment_tools_ros2 auv-monitor
```

The AUVMonitor node can be interacted with by sending requests of type std_srvs::Trigger
to the following topics.

1. toggle_daq
2. toggle_system

After launching, the AUVMonitor will report the status of all modules. The following topics
are available at a configurable refresh rate (default = 4Hz).

1. /auv_monitor/status/daq
2. /auv_monitor/status/system

## Documentation
Documentation for code can be generated using the included Doxyfile with [Doxygen](https://www.doxygen.nl/). The resultant webpage should contain documentation for all component .hpp and .py files.

To generate documentation for configuration options, the [yamldoc](http://chrisbcole.me/yamldoc/) pip package should be used

## Contributing
To ensure Doxygen output is complete and precise, please follow Doxygen comment styling to ensure generated output contains all documentation

For specific styling preferences, refer to AUVMonitor.hpp for an example

This is currently a closed-source project maintained by Robert Fudge, 2025 -

## Frequently Asked Questions
If pytorch encounters any CUDA errors, it could be the container is having trouble accessing the hardware due to some unexpected condition (sleep, etc.), to fix, restart the device.

## License
[Apache-2.0](https://choosealicense.com/licenses/apache-2.0/)