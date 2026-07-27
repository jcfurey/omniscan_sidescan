# -*- coding: utf-8 -*-
"""Driver for a single Cerulean Omniscan 450 side scan sonar.

Deliberately free of any rclpy import so that it can be exercised without a
ROS graph; the nodes pass in a logger.

Originally authored for ROS 1 by Monika Roznere (Binghamton University).
"""

from dataclasses import dataclass, fields
import time

from .brping import definitions, Omniscan450

# Sonar defaults.
SONAR_IP_ADDRESS = '192.168.2.92'
SONAR_PORT = 51200
SPEED_OF_SOUND_MM = 1482000
START_MM = 0
LENGTH_MM = 5000  # 5 meters
MSEC_PER_PING = 0  # 0 gives the best ping rate
PULSE_LEN_PERCENT = 0.002
FILTER_DURATION_PERCENT = 0.0015
GAIN_INDEX = -1
NUM_RESULTS = 600


class SonarError(RuntimeError):
    """Raised when the sonar cannot be reached or returns nothing usable."""


@dataclass
class SonarParams:
    """The tunable ping parameters, mirroring the ROS 1 dynamic_reconfigure cfg.

    ``speed_of_sound_mm`` is sent on its own message; every other field is part
    of the os_ping_params message.
    """

    speed_of_sound_mm: int = SPEED_OF_SOUND_MM
    start_mm: int = START_MM
    length_mm: int = LENGTH_MM
    msec_per_ping: int = MSEC_PER_PING
    pulse_len_percent: float = PULSE_LEN_PERCENT
    filter_duration_percent: float = FILTER_DURATION_PERCENT
    gain_index: int = GAIN_INDEX
    num_results: int = NUM_RESULTS

    @classmethod
    def field_names(cls):
        """Names of the tunable parameters, in declaration order."""
        return tuple(f.name for f in fields(cls))


class Omniscan450Driver(object):
    """Owns the connection to one sonar and converts its profiles."""

    def __init__(self, ip_address=SONAR_IP_ADDRESS, port=SONAR_PORT,
                 params=None, name=None, logger=None, connect_timeout=5.0):
        """Connect to the sonar and push the initial parameters.

        Args:
            ip_address: sonar address on the vehicle network.
            port: sonar TCP port.
            params: a SonarParams; defaults are used when omitted.
            name: label used in log messages, e.g. "port".
            logger: object with info/warn/error methods, e.g. a rclpy logger.
            connect_timeout: TCP connect timeout, in seconds.

        Raises:
            SonarError: if the sonar cannot be reached or does not initialize.
        """
        self.ip_address = ip_address
        self.port = port
        self.name = name or ip_address
        self.params = params if params is not None else SonarParams()
        self._logger = logger

        self.omniscan450 = Omniscan450()

        try:
            self.omniscan450.connect_tcp(ip_address, port,
                                         timeout=connect_timeout)
        except ConnectionError as exception:
            raise SonarError(
                'Omniscan 450 (%s) at %s:%d could not be opened: %s'
                % (self.name, ip_address, port, exception))

        if not self.omniscan450.initialize():
            self.omniscan450.close()
            raise SonarError(
                'Omniscan 450 (%s) at %s:%d did not respond to the device '
                'information request' % (self.name, ip_address, port))

        info = self.omniscan450.readDeviceInformation()
        if info is not None:
            self._log('info',
                      'Omniscan 450 (%s) at %s:%d ready, device type %s, '
                      'firmware %d.%d.%d'
                      % (self.name, ip_address, port, info.device_type,
                         info.firmware_version_major,
                         info.firmware_version_minor,
                         info.firmware_version_patch))

        # Push the starting configuration to the sonar.
        self._apply(change_speed_of_sound=True, change_os_ping_params=True)

    def get_data(self, timeout=0.5, latest=True):
        """Read one profile from the sonar.

        Args:
            timeout: seconds to wait for a profile.
            latest: consume the whole backlog and return the newest profile.
                The sonar streams continuously whether or not anything is
                reading, so taking the oldest buffered profile each time lets
                the published range drift further and further behind.

        Returns:
            (data, scaled_results): the raw os_mono_profile message and its
            power results scaled to dB.

        Raises:
            SonarError: if no profile arrives within the timeout, or the
                profile carries no results.
        """
        data = self.omniscan450.wait_message(
            [definitions.OMNISCAN450_OS_MONO_PROFILE], timeout=timeout,
            latest=latest)

        if data is None:
            raise SonarError(
                'Omniscan 450 (%s) at %s did not return a profile within %.3f s'
                % (self.name, self.ip_address, timeout))

        if not data.pwr_results:
            raise SonarError(
                'Omniscan 450 (%s) at %s returned an empty profile'
                % (self.name, self.ip_address))

        return data, Omniscan450.scale_power(data)

    def set_parameters(self, params):
        """Apply a new SonarParams, sending only what actually changed.

        Returns the parameters now in effect.
        """
        change_speed_of_sound = (
            self.params.speed_of_sound_mm != params.speed_of_sound_mm)

        # Every remaining field lives in the os_ping_params message, so any
        # difference means the whole message is resent.
        change_os_ping_params = any(
            getattr(self.params, name) != getattr(params, name)
            for name in SonarParams.field_names()
            if name != 'speed_of_sound_mm'
        )

        self.params = params
        self._apply(change_speed_of_sound, change_os_ping_params)

        return self.params

    def _apply(self, change_speed_of_sound=False, change_os_ping_params=False):
        """Send the parameter messages the caller asked for."""
        if change_speed_of_sound:
            self.omniscan450.control_set_speed_of_sound(
                speed_of_sound=self.params.speed_of_sound_mm)

        if change_os_ping_params:
            self._send_ping_params(enable=1)

        time.sleep(0.01)

    def _send_ping_params(self, enable):
        """Send os_ping_params with the current settings and enable flag."""
        self.omniscan450.control_os_ping_params(
            # Start of the ping range; normally 0.
            start_mm=self.params.start_mm,
            # Length of the returned profile; range ends at start + length.
            length_mm=self.params.length_mm,
            # Limits the ping rate; 0 gives the best ping rate.
            msec_per_ping=self.params.msec_per_ping,
            # % of total ping time for the current range, 0.002 typical.
            pulse_len_percent=self.params.pulse_len_percent,
            # 0.0015 typical.
            filter_duration_percent=self.params.filter_duration_percent,
            # -1 for auto gain, otherwise 0-7 sets the gain.
            gain_index=self.params.gain_index,
            # Signal data points in the returned profile (200-1200).
            num_results=self.params.num_results,
            # 1 or 0 to enable or disable pinging.
            enable=enable,
            reserved_1=0,
            reserved_2=0,
            reserved_3=0,
        )

    def close_connection(self):
        """Stop the sonar pinging and close the socket."""
        try:
            self._send_ping_params(enable=0)
        except (ConnectionError, OSError) as exception:
            self._log('warn',
                      'Omniscan 450 (%s) at %s could not be told to stop '
                      'pinging: %s' % (self.name, self.ip_address, exception))

        try:
            self.omniscan450.close()
        except OSError as exception:
            self._log('warn',
                      'Omniscan 450 (%s) at %s failed to close its socket: %s'
                      % (self.name, self.ip_address, exception))

    def _log(self, level, message):
        if self._logger is not None:
            getattr(self._logger, level)(message)
