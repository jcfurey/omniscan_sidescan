"""Run repeatable four-head fault/recovery cycles and report bounded resources."""

import argparse
import json
from pathlib import Path
import tempfile
import time

from lifecycle_msgs.msg import Transition
import rclpy
from rclpy.utilities import get_rmw_implementation_identifier

from test_cpp_driver import NativeHarness


def exercise(harness, seconds):
    """Rotate faults, change settings, and restart workers for the requested duration."""
    harness.wait(lambda: len(harness.combined) >= 10)
    baseline = harness.resources()
    peak = list(baseline)
    generation = 0
    started = time.monotonic()
    next_progress = started + 60
    while time.monotonic() - started < seconds:
        failed_name = harness.names[generation % 4]
        faulty = harness.devices[failed_name]
        healthy = [name for name in harness.names if name != failed_name]
        connections = {name: device.connections for name, device in harness.devices.items()}
        pings = {name: harness.profiles[name][-1].ping_number for name in healthy}
        faulty.bad_checksum = True
        harness.wait(lambda: faulty.connections > connections[failed_name], timeout=2)
        harness.wait(lambda: all(harness.profiles[name][-1].ping_number > pings[name]
                                 for name in healthy), timeout=1)
        assert all(harness.devices[name].connections == connections[name] for name in healthy)
        harness.transition(Transition.TRANSITION_DEACTIVATE)
        harness.wait(lambda: not any(device.enabled for device in harness.devices.values()),
                     timeout=0.5)
        faulty.bad_checksum = False
        generation += 1
        count = (200, 600, 1200)[generation % 3]
        # Cycle within documented application ranges even on multi-hour runs.
        length = 5000 + generation % 1000 * 100
        speed = 1500000 if generation % 2 else 1482000
        gain = (-1, 0, 4, 7)[generation % 4]
        assert harness.update(length_mm=length, num_results=count,
                              speed_of_sound_mm=speed, gain_index=gain).successful
        harness.transition(Transition.TRANSITION_ACTIVATE)
        harness.wait(lambda: all(messages[-1].configuration_generation == generation
                                 for messages in harness.profiles.values()), timeout=2)
        harness.wait(lambda: harness.combined[-1].length_mm == length, timeout=2)
        for messages in harness.profiles.values():
            profile = messages[-1]
            assert len(profile.pwr_data) == len(profile.scaled_data) == count
            assert profile.start_mm == 0 and profile.length_mm == length
            assert profile.sos_dmps == speed // 100
            if gain >= 0:
                assert profile.gain_index == gain
        current = harness.resources()
        peak = [max(before, after) for before, after in zip(peak, current)]
        assert current[0] <= baseline[0] + 1, (baseline, current)
        assert current[1] <= baseline[1] + 1, (baseline, current)
        assert current[2] < baseline[2] + 12, (baseline, current)
        if time.monotonic() >= next_progress:
            print(json.dumps({'elapsed_sec': round(time.monotonic() - started, 1),
                              'cycles': generation, 'fd_threads_rss_mib': current}), flush=True)
            next_progress += 60
    return {
        'seconds': round(time.monotonic() - started, 2),
        'channels': 4,
        'rmw_implementation': get_rmw_implementation_identifier(),
        'fault_recovery_cycles': generation,
        'configuration_generation': generation,
        'healthy_head_reconnects_during_faults': 0,
        'baseline_fd_threads_rss_mib': baseline,
        'peak_fd_threads_rss_mib': peak,
        'final_fd_threads_rss_mib': harness.resources(),
    }


def main():
    """Select duration/container and retain a JSON report for comparison or CI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=3600)
    parser.add_argument('--component', choices=['standalone', 'isolated', 'multi-threaded'],
                        default='standalone')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if not 0 < args.seconds <= 86400:
        parser.error('--seconds must be in (0, 86400]')
    component = {'standalone': False, 'isolated': True,
                 'multi-threaded': 'multi-threaded'}[args.component]
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory(prefix='omniscan-soak-') as directory:
            harness = NativeHarness(Path(directory), count=4, component=component)
            try:
                if component:
                    harness.transition(Transition.TRANSITION_CONFIGURE)
                    harness.transition(Transition.TRANSITION_ACTIVATE)
                report = exercise(harness, args.seconds)
                report['component'] = args.component
            except BaseException:
                print(harness.log_path.read_text(), flush=True)
                raise
            finally:
                harness.close()
            if args.report:
                args.report.parent.mkdir(parents=True, exist_ok=True)
                args.report.write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(report, indent=2))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
