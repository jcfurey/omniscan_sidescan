#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publishes readings from a port/starboard pair of Omniscan 450 sonars.

Originally authored for ROS 1 by Monika Roznere (Binghamton University).
"""

from omniscan_sidescan_msgs.msg import SideScanSonar, SideScanSonarRaw

from .sonar_node import run_sonar_node, SideScanSonarNode


class SonarNode(SideScanSonarNode):
    """The two-sonar node."""

    CHANNELS = ('port', 'starboard')
    DEFAULT_IP_ADDRESSES = {
        'port': '192.168.2.94',
        'starboard': '192.168.2.95',
    }
    SONAR_MSG = SideScanSonar
    SONAR_RAW_MSG = SideScanSonarRaw


def main(args=None):
    run_sonar_node(SonarNode, 'omniscan', args=args)


if __name__ == '__main__':
    main()
