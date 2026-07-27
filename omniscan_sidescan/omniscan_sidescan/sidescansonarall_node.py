#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publishes readings from all four Omniscan 450 sonars.

Originally authored for ROS 1 by Monika Roznere (Binghamton University).
"""

from omniscan_sidescan_msgs.msg import SideScanSonarAll, SideScanSonarAllRaw

from .sonar_node import run_sonar_node, SideScanSonarNode


class SonarAllNode(SideScanSonarNode):
    """The four-sonar node."""

    CHANNELS = ('port', 'starboard', 'bow', 'stern')
    DEFAULT_IP_ADDRESSES = {
        'port': '192.168.2.94',
        'starboard': '192.168.2.95',
        'bow': '192.168.2.93',
        'stern': '192.168.2.92',
    }
    SONAR_MSG = SideScanSonarAll
    SONAR_RAW_MSG = SideScanSonarAllRaw


def main(args=None):
    run_sonar_node(SonarAllNode, 'omniscanall', args=args)


if __name__ == '__main__':
    main()
