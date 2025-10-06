# omniscan_sidescan

ROS 1 driver for the [Cerulean Omniscan 450 Side Scan Sonar](https://bluerobotics.com/store/the-reef/cerulean-sidescan-sonar/)

Tested on ROS Noetic

## Getting started

### Prerequisites

Requires [ping-python](https://github.com/bluerobotics/ping-python/tree/master) installed from **source** (at the time of this package, pip does not install the necessary omniscan450 sonar definitions).

Blue Robotics BlueOS as well as the following extensions:
- ROS
- SonarView

Make sure to follow the [Guide: Integrating the Omnniscan 450 SS on the BlueBoat](https://bluerobotics.com/learn/omniscan-450-ss-integration/), including the software steps on SonarView.

### Running

Connect to the BlueOS.


[optional] On your host computer (laptop), in terminal:

```export ROS_MASTER_URI=http://192.168.2.2:11311```

***Adjust the ip address for each sonar in the launch file.***

Run omniscan_sonar for one pair of sonars:

```roslaunch omniscan_sidescan sidescansonar.launch```

Run omniscan_sonar for two pairs of sonars:

```roslaunch omniscan_sidescan sidescansonarall.launch```

### Output

#### Topics

The following topics are published:

- /omniscan450/range_raw
- /omniscan450/range

Note that both topics are identical except for the sonar results. `range_raw` publishes the raw unscaled power results, while `raw` publishes the scaled (corrected) power results.

#### Dynamic Reconfiguration

The following are parameters that you can change on-the-run. Changing a parameter will update the corresponding parameter for both the port and starboard sonars. We recommend referring to [ping-protocol definitions](https://github.com/bluerobotics/ping-protocol/blob/master/src/definitions/omniscan450.json)

- speed_of_sound_mm
- start_mm
- length_mm
- msec_per_ping
- pulse_len_percent
- filter_duration_percent
- gain_index
- num_results

## Authors

[Monika Roznere](https://monikaroznere.com) - Marine Robotics Lab, Binghamton University

## License

This project is licensed under the MIT License -- see the [LICENSE](LICENSE) file for details.