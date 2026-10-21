"""Compare installed native and Python drivers with identical TCP/DDS loads.

Run after sourcing the workspace: python3 test/benchmark_driver.py --seconds 10.
Linux /proc accounting measures only the driver process, including its threads.
The simulators and DDS observer run in this separate parent process.
"""

import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import platform
import signal
import statistics
import subprocess
import tempfile
import time
import uuid

from ament_index_python.packages import get_package_prefix
from omniscan_sidescan.omniscan450_simulator import Omniscan450Simulator
from omniscan_sidescan_msgs.msg import SideScanSonarAll, SideScanSonarAllRaw, SonarProfile
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import get_rmw_implementation_identifier

CHANNELS = ('port', 'starboard', 'bow', 'stern')


def accounting(pid):
    """Return cumulative CPU seconds and resident MiB for the child process."""
    directory = Path('/proc') / str(pid)
    fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
    cpu = (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')
    resident = int((directory / 'statm').read_text().split()[1])
    return cpu, resident * os.sysconf('SC_PAGE_SIZE') / (1024 * 1024)


def trial(implementation, seconds, samples, rate):
    """Observe the same four profiles and two legacy topics for each driver."""
    with ExitStack() as stack:
        devices = [stack.enter_context(Omniscan450Simulator(
            port=0, channel_number=index, ping_interval=1.0 / rate)) for index in range(4)]
        namespace = '/benchmark_' + uuid.uuid4().hex[:10]
        observer = rclpy.create_node('observer', namespace=namespace)
        stack.callback(observer.destroy_node)
        counts = dict.fromkeys((*CHANNELS, 'combined', 'raw'), 0)

        def counter(name):
            def receive(message):
                counts[name] += 1
            return receive

        for name in CHANNELS:
            observer.create_subscription(
                SonarProfile, namespace + '/omniscan450/' + name + '/profile',
                counter(name), qos_profile_sensor_data)
        for name, msg_type, topic in (
                ('combined', SideScanSonarAll, 'range'),
                ('raw', SideScanSonarAllRaw, 'range_raw')):
            observer.create_subscription(
                msg_type, namespace + '/omniscan450/' + topic,
                counter(name), qos_profile_sensor_data)
        binary = 'sidescansonarall_node' + ('_python' if implementation == 'python' else '')
        prefix = Path(get_package_prefix('omniscan_sidescan'))
        command = [str(prefix / 'lib/omniscan_sidescan' / binary), '--ros-args',
                   '-r', '__ns:=' + namespace, '-p', 'num_results:=' + str(samples)]
        for name, device in zip(CHANNELS, devices):
            command.extend(['-p', name + '_ip_address:=127.0.0.1',
                            '-p', name + '_port:=' + str(device.address[1])])
        log = stack.enter_context(tempfile.TemporaryFile(mode='w+'))
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 8
            while min(counts.values()) < 5 and time.monotonic() < deadline:
                if process.poll() is not None:
                    break
                rclpy.spin_once(observer, timeout_sec=0.01)
            if min(counts.values()) < 5:
                log.seek(0)
                raise RuntimeError('driver failed to stream: ' + log.read())
            initial = counts.copy()
            cpu_before, _ = accounting(process.pid)
            started = time.monotonic()
            while time.monotonic() - started < seconds:
                rclpy.spin_once(observer, timeout_sec=0.005)
            elapsed = time.monotonic() - started
            cpu_after, resident_mib = accounting(process.pid)
            return {
                'implementation': implementation,
                'wall_seconds': elapsed,
                'cpu_percent_of_one_core': 100 * (cpu_after - cpu_before) / elapsed,
                'resident_mib': resident_mib,
                'received_hz': {name: (counts[name] - initial[name]) / elapsed for name in counts},
            }
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                    raise RuntimeError('driver failed to stop')
            if process.returncode != 0:
                log.seek(0)
                raise RuntimeError('driver exited unsuccessfully: ' + log.read())


def main():
    """Print reproducible workload metadata, each run, and median summaries."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10.0)
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--samples', type=int, choices=range(200, 1201), default=1200)
    parser.add_argument('--rate', type=float, default=20.0)
    args = parser.parse_args()
    if args.seconds <= 0 or args.runs < 1 or args.rate <= 0:
        parser.error('seconds, runs and rate must be positive')
    rclpy.init()
    try:
        runs = [trial(implementation, args.seconds, args.samples, args.rate)
                for _ in range(args.runs) for implementation in ('cpp', 'python')]
    finally:
        rclpy.shutdown()
    summary = {}
    for implementation in ('cpp', 'python'):
        subset = [run for run in runs if run['implementation'] == implementation]
        summary[implementation] = {
            name: statistics.median(run[name] for run in subset)
            for name in ('cpu_percent_of_one_core', 'resident_mib')}
        summary[implementation]['combined_hz'] = statistics.median(
            run['received_hz']['combined'] for run in subset)
    print(json.dumps({
        'platform': platform.platform(), 'ros_distro': os.environ.get('ROS_DISTRO'),
        'rmw_implementation': get_rmw_implementation_identifier(),
        'channels': 4, 'samples': args.samples, 'requested_rate_hz': args.rate,
        'subscribers': 6, 'runs': runs, 'median': summary,
    }, indent=2))


if __name__ == '__main__':
    main()
