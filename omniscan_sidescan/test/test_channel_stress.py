"""Stress the native worker API independently of ROS and its executor."""

from contextlib import ExitStack
import json
import os
import subprocess

from omniscan_sidescan.omniscan450_simulator import Omniscan450Simulator


def test_concurrent_snapshots_configuration_and_worker_restarts():
    """Exercise immutable snapshots during rapid updates and thread restarts."""
    binary = os.environ.get('OMNISCAN_CHANNEL_STRESS_EXECUTABLE')
    assert binary, 'Run through colcon test or set OMNISCAN_CHANNEL_STRESS_EXECUTABLE'
    with ExitStack() as stack:
        devices = [stack.enter_context(Omniscan450Simulator(
            port=0, channel_number=index, ping_interval=0.02)) for index in range(4)]
        result = subprocess.run(
            [binary, *(str(device.address[1]) for device in devices)],
            capture_output=True, text=True, timeout=40)
        assert result.returncode == 0, result.stdout + result.stderr
        report = json.loads(result.stdout)
        assert report['violations'] == 0
        assert report['channels'] == 4
        assert report['rounds'] == 3
        assert report['generation'] == 60
        assert report['observations'] >= 100
        assert report['notifications'] > 0
