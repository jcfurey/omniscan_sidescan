# Vehicle and timing validation

The simulator verifies host logic and wire layouts. It does not reproduce
acoustic interference, synchronization over the vehicle Ethernet network,
firmware-specific limits, or the relation between sonar uptime and the ROS clock.
Run this procedure on the target computer, with the actual sonars in their
normal operating environment, before treating the conversion as qualified.

## Record the setup

Record the git revision, ROS distribution, RMW implementation, CPU/architecture,
sonar model and firmware per head, network interface/subnet, settings, and
expected mission duration. Retain the ROS log and a bag containing all profile
topics, both combined topics, diagnostics, vehicle clock/odometry, and TF.

Use `start_mm=0`, as specified by the vendor. Verify sound speed and range
against a target at a measured distance. Exercise the intended result count,
manual/automatic gain, range, and period, including `msec_per_ping=100` for
10 Hz. Set read/configuration deadlines above the intended period. Confirm
profile-reported geometry, sound speed, gain, and channel fields against the
requests. Ping period and pulse/filter fractions remain commanded-only values;
measure actual ping cadence rather than inferring it from parameter success.

## Check synchronization and timestamps

1. With `sync_enabled=false`, record current assignments if another application
   manages them. The driver leaves those assignments untouched.
2. To let this driver manage the group, configure `sync_enabled=true` at startup.
   Port is channel 0/master; starboard, bow, and stern are 1, 2, and 3. The
   two-head node uses group size 2; the four-head node uses 4. Confirm
   `sync_assignment_verified=1` in every healthy head's diagnostics and the
   assigned channel in every profile, including after reconnection/power cycling.
3. Check that all heads share a subnet and exchange the vendor's UDP traffic
   on port 51201. A capture on a mirrored switch port can confirm connectivity;
   for example, `tcpdump -i <interface> -w sync.pcap udp port 51201`.
4. Measure acoustic/transmit timing with appropriate device instrumentation or
   a hydrophone/oscilloscope. The vendor describes typical synchronization below
   100 microseconds; millisecond uptime fields and host receipt times cannot
   establish that timing. Test a missing master and missing group member: the
   device synchronization protocol can couple the group's availability.
5. Measure ping-to-host latency, jitter, and clock drift under the vehicle's
   normal Ethernet/CPU/bagging load. `header.stamp` is the complete packet's ROS
   receipt time. `timestamp_ms` is each device's independent boot-time counter.
   Neither the readback flag nor receive skew calibrates the device clock to ROS.

Navigation requiring absolute acquisition timestamps needs measured clock
calibration or hardware clock discipline first. Do not relabel host receipt
timestamps as acoustic time based on an unmeasured fixed offset.

## Run a mission-length workload

Run streaming and normal bagging for at least the intended mission duration,
including thermal and network load. Check profile rate/gaps, combined-set
availability, receive ages/skew, parser errors, duplicates, generations,
publication coalescing/failures, and driver CPU/RSS/thread/descriptor counts.
Resource counts should settle after discovery and remain bounded.

During a separate recovery run, disconnect/reconnect and power-cycle one head
at a time, change parameters during outages, pause a consumer, and cycle
deactivate/activate/cleanup. Without device synchronization coupling, healthy
heads should retain their TCP sessions and profile topics; parameter services
should remain responsive. Verify that deactivate, unload, SIGINT, and SIGTERM
stop device pinging, including when configuration is still unconfirmed.

Exercise the intended deployment container/executor and RMW implementation.
The native publication workers keep application locks out of DDS calls, but
an already-running middleware call can still delay its own worker's shutdown.
Record measured stop latency with the target middleware/network configuration.

Archive the setup, logs, bag/captures, measured timing, and resource report with
explicit pass/fail limits for the vehicle. Until those measurements exist,
acoustic timing and mission-length target operation remain unverified.

## Protocol references

- [Omniscan API](https://docs.ceruleansonar.com/c/omniscan-450/application-programming-interface)
- [Synchronization assignment](https://docs.ceruleansonar.com/c/cerulean-ping-protocol/general-packet-definitions/set_sync_channel_number)
- [Synchronization readback](https://docs.ceruleansonar.com/c/cerulean-ping-protocol/general-packet-definitions/sync_channel_number)
- [Machine-readable wire definitions](https://github.com/bluerobotics/ping-protocol/blob/master/src/definitions/omniscan450.json)
