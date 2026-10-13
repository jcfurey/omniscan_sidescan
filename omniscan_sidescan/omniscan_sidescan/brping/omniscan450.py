# -*- coding: utf-8 -*-
"""Device API for the Cerulean Sonar Omniscan 450 side scan sonar.

Vendored and trimmed from ping-python (Blue Robotics, MIT) -- see LICENSE in
this directory.  The .svlog recording helpers are omitted; ROS 2 bags cover
that need here.
"""

from array import array
import math
import sys
import time

from . import definitions
from . import pingmessage
from .device import PingDevice


class Omniscan450(PingDevice):
    """Talks to an Omniscan 450 over ping-protocol."""

    def initialize(self):
        """Return True if the device answers a device information request."""
        return self.readDeviceInformation() is not None

    def get_os_mono_profile(self):
        """Get an os_mono_profile message from the device.

        A profile is produced from a single acoustic measurement: an array of
        response strength, in dB, at even intervals across the scan region.
        The scan region runs from ``start_mm`` to ``start_mm + length_mm``
        away from the transducer.  Amplitudes are in dB purely to keep high
        dynamic range in u16 sized elements; normally they are converted back
        to linear power or signal levels.

        Returns None if there is no reply from the device, otherwise a dict.
        """
        if self.request(definitions.OMNISCAN450_OS_MONO_PROFILE) is None:
            return None
        return {
            "ping_number": self._ping_number,
            "start_mm": self._start_mm,
            "length_mm": self._length_mm,
            "timestamp_ms": self._timestamp_ms,
            "ping_hz": self._ping_hz,
            "gain_index": self._gain_index,
            "num_results": self._num_results,
            "sos_dmps": self._sos_dmps,
            "channel_number": self._channel_number,
            "reserved": self._reserved,
            "pulse_duration_sec": self._pulse_duration_sec,
            "analog_gain": self._analog_gain,
            "max_pwr_db": self._max_pwr_db,
            "min_pwr_db": self._min_pwr_db,
            "transducer_heading_deg": self._transducer_heading_deg,
            "vehicle_heading_deg": self._vehicle_heading_deg,
            "pwr_results": self._pwr_results,
        }

    def control_os_ping_params(self, start_mm=0, length_mm=5000,
                               msec_per_ping=0, reserved_1=0, reserved_2=0,
                               pulse_len_percent=0.002,
                               filter_duration_percent=0.0015, gain_index=-1,
                               num_results=600, enable=True, reserved_3=0,
                               reserved_4=0, reserved_5=0):
        """Send the ping parameters, including the enable/disable flag."""
        m = pingmessage.PingMessage(definitions.OMNISCAN450_OS_PING_PARAMS)
        m.start_mm = start_mm
        m.length_mm = length_mm
        m.msec_per_ping = msec_per_ping
        m.reserved_1 = reserved_1
        m.reserved_2 = reserved_2
        m.pulse_len_percent = pulse_len_percent
        m.filter_duration_percent = filter_duration_percent
        m.gain_index = gain_index
        m.num_results = num_results
        m.enable = enable
        m.reserved_3 = reserved_3
        m.reserved_4 = reserved_4
        m.reserved_5 = reserved_5
        m.pack_msg_data()
        self.write(m.msg_data)

    def control_set_speed_of_sound(self, speed_of_sound):
        """Set the speed of sound used for range calculation, in mm/s."""
        m = pingmessage.PingMessage(definitions.OMNISCAN450_SET_SPEED_OF_SOUND)
        m.speed_of_sound = speed_of_sound
        m.pack_msg_data()
        self.write(m.msg_data)

    def readDeviceInformation(self):
        """Request the common device information message."""
        return self.request(definitions.COMMON_DEVICE_INFORMATION)

    @staticmethod
    def calc_msec_per_ping(ping_rate):
        """Convert a ping rate, in Hz, to milliseconds per ping."""
        return math.floor(1000.0 / ping_rate)

    @staticmethod
    def calc_pulse_length_pc(percent):
        """Convert a pulse length percentage to the wire representation."""
        if percent > 0:
            return 0.01 * percent
        return 1.5 / 1200

    @staticmethod
    def scale_power(msg):
        """Scale u16 power results back to dB using the message's dB range."""
        span = msg.max_pwr_db - msg.min_pwr_db
        return array(
            'f',
            (msg.min_pwr_db + (result / 65535.0) * span
             for result in msg.pwr_results)
        )

    @staticmethod
    def _unpack_profile(msg):
        """Turn a profile's raw power bytes into a tuple of u16 counts.

        Returns False and leaves the message alone if the payload length does
        not match the advertised result count, which would otherwise raise out
        of struct.unpack on a truncated or corrupt profile.
        """
        num_results = int(msg.num_results)
        if len(msg.pwr_results) != 2 * num_results:
            return False

        results = array('H')
        results.frombytes(msg.pwr_results)
        if results.itemsize != 2:
            return False
        if sys.byteorder != 'little':
            results.byteswap()
        msg.pwr_results = results
        return True

    def wait_message(self, message_ids, timeout=0.5, latest=False):
        """As PingDevice.wait_message, but unpack profile power results.

        With ``latest`` set, keep consuming whatever is already buffered and
        return the most recent matching message instead of the first one.  The
        sonar streams continuously, so returning the first buffered profile
        lets the socket backlog -- and the reported range -- fall further and
        further behind real time.
        """
        deadline = time.monotonic() + timeout
        found = None

        while True:
            msg = self.read()

            if msg is None:
                # Buffer is dry; stop here if we already have something.
                if found is not None:
                    return found
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                if not self._input_buffer:
                    self.wait_readable(remaining)
                continue

            if msg.message_id == definitions.OMNISCAN450_OS_MONO_PROFILE:
                if not self._unpack_profile(msg):
                    continue

            if msg.message_id in message_ids:
                if not latest:
                    return msg
                found = msg

            if time.monotonic() >= deadline:
                return found
