# omniscan_sidescan

Driver for the [Cerulean Omniscan 450 Side Scan
Sonar](https://bluerobotics.com/store/the-reef/cerulean-sidescan-sonar/), for
ROS 1 and ROS 2.

| | Directory | Tested on | Notes |
| --- | --- | --- | --- |
| **ROS 2** | [`ros2/`](ros2) | Jazzy | Standalone — no dependencies outside a ROS 2 install |
| **ROS 1** | [`ros1/`](ros1) | Noetic | The original driver, unchanged |

Both publish `/omniscan450/range` and `/omniscan450/range_raw` with the same
message fields, so downstream consumers see the same data either way.

## Which to use

Start with [`ros2/`](ros2) unless you are tied to an existing ROS 1 stack.
Besides being the supported distribution, the ROS 2 version needs no
`ping-python` source install, ships a sonar simulator so it can run without
hardware, and fixes several bugs in the original — most importantly a shared
receive buffer that could attribute a profile to the wrong sonar when more
than one is connected. See [ros2/README.md](ros2/README.md#differences-from-the-ros-1-package)
for the full list.

## Layout

```
ros1/omniscan_sidescan/     catkin package (ROS 1)
ros2/omniscan_sidescan/     ament_python package: driver, nodes, launch, simulator
ros2/omniscan_sidescan_msgs/   ament_cmake package: message definitions
notes                       vehicle addresses and recorded topic list
```

The two halves are independent; each builds without the other. `ros1/`
carries a `COLCON_IGNORE` so a `colcon build` at the repository root picks up
only the ROS 2 packages. There is no equivalent marker that hides the ROS 2
packages from catkin, so ROS 1 users should link `ros1/omniscan_sidescan` into
their catkin workspace rather than the whole repository — see
[ros1/README.md](ros1/README.md).

## Hardware setup

Either version expects Blue Robotics BlueOS with the ROS and SonarView
extensions. Follow the [Guide: Integrating the Omniscan 450 SS on the
BlueBoat](https://bluerobotics.com/learn/omniscan-450-ss-integration/),
including the SonarView software steps.

## Authors

[Monika Roznere](https://monikaroznere.com) — Marine Robotics Lab, Binghamton
University.

## License

MIT — see [LICENSE](LICENSE).
