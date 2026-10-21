# -*- coding: utf-8 -*-
"""Driver for a single Cerulean Omniscan 450 side scan sonar.

Deliberately free of any rclpy import so that it can be exercised without a
ROS graph; the nodes pass in a logger.

Originally authored for ROS 1 by Monika Roznere (Binghamton University).
"""

from dataclasses import dataclass, fields
import math
import time

from .brping import definitions, Omniscan450
from .brping.omniscan450 import NackError

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


class SonarConnectionError(SonarError):
    """Raised when communication with a sonar fails."""


class SonarTimeoutError(SonarError):
    """Raised when a connected sonar does not provide a profile in time."""


class SonarDataError(SonarError):
    """Raised when a sonar provides a structurally invalid profile."""


class SonarConfigurationError(SonarError):
    """Raised when a command is rejected or profile settings do not converge."""


PARAMETER_LIMITS = {
    'speed_of_sound_mm': (1482000, 1500000),
    'start_mm': (0, 5000),
    'length_mm': (1, 300000),
    'msec_per_ping': (0, 10),
    'pulse_len_percent': (0.002, 0.004),
    'filter_duration_percent': (0.0015, 0.0016),
    'gain_index': (-1, 7),
    'num_results': (200, 1200),
}


@dataclass(frozen=True)
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

    def __post_init__(self):
        """Reject values the sonar protocol does not support."""
        for field in fields(self):
            value = getattr(self, field.name)
            minimum, maximum = PARAMETER_LIMITS[field.name]
            expected_type = int if isinstance(field.default, int) else float

            if isinstance(value, bool) or not isinstance(value, expected_type):
                raise TypeError(
                    '%s must be %s, got %r'
                    % (field.name, expected_type.__name__, value))
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError('%s must be finite' % field.name)
            if not minimum <= value <= maximum:
                raise ValueError(
                    '%s must be between %s and %s, got %s'
                    % (field.name, minimum, maximum, value))

    @classmethod
    def field_names(cls):
        """Names of the tunable parameters, in declaration order."""
        return tuple(f.name for f in fields(cls))


class Omniscan450Driver(object):
    """Owns the connection to one sonar and converts its profiles."""

    def __init__(self, ip_address=SONAR_IP_ADDRESS, port=SONAR_PORT,
                 params=None, name=None, logger=None, connect_timeout=5.0,
                 configuration_timeout=2.0, cancel_event=None, clock_ns=None):
        """Connect to the sonar and verify the initial profile settings.

        Args:
            ip_address: sonar address on the vehicle network.
            port: sonar TCP port.
            params: a SonarParams; defaults are used when omitted.
            name: label used in log messages, e.g. "port".
            logger: object with info/warn/error methods, e.g. a rclpy logger.
            connect_timeout: TCP connect timeout, in seconds.
            configuration_timeout: deadline for settings to appear in profiles.
            cancel_event: optional event interrupting network waits.
            clock_ns: clock returning nanoseconds for host receive timestamps.

        Raises:
            SonarError: if the sonar cannot be reached or does not initialize.
        """
        if not isinstance(ip_address, str) or not ip_address.strip():
            raise ValueError('ip_address must be a non-empty string')
        if isinstance(port, bool) or not isinstance(port, int):
            raise TypeError('port must be an integer')
        if not 1 <= port <= 65535:
            raise ValueError('port must be between 1 and 65535')
        if not isinstance(connect_timeout, (int, float)) \
                or isinstance(connect_timeout, bool) \
                or not math.isfinite(connect_timeout) or connect_timeout <= 0:
            raise ValueError('connect_timeout must be greater than zero')
        if isinstance(configuration_timeout, bool) \
                or not isinstance(configuration_timeout, (int, float)) \
                or not math.isfinite(configuration_timeout) \
                or configuration_timeout <= 0:
            raise ValueError('configuration_timeout must be finite and positive')
        if params is not None and not isinstance(params, SonarParams):
            raise TypeError('params must be a SonarParams instance')

        self.ip_address = ip_address.strip()
        self.port = port
        self.name = name or ip_address
        self.params = params if params is not None else SonarParams()
        self._logger = logger
        self._closed = True
        self.configuration_timeout = configuration_timeout

        self.omniscan450 = Omniscan450()
        self.omniscan450.cancel_event = cancel_event
        if clock_ns is not None:
            self.omniscan450.clock_ns = clock_ns

        try:
            self.omniscan450.connect_tcp(self.ip_address, port,
                                         timeout=connect_timeout)
            self._closed = False

            # A single device-information request both confirms the protocol
            # connection and provides the details used in the ready log.
            info = self.omniscan450.readDeviceInformation(timeout=configuration_timeout)
            if info is None:
                raise SonarConnectionError(
                    'device information request timed out')

            self._log(
                'info',
                'Omniscan 450 (%s) at %s:%d connected, device type %s, '
                'firmware %d.%d.%d'
                % (self.name, self.ip_address, port, info.device_type,
                   info.firmware_version_major,
                   info.firmware_version_minor,
                   info.firmware_version_patch))

            # Push the starting configuration to the sonar.
            self._apply(
                self.params, change_speed_of_sound=True,
                change_os_ping_params=True)
        except NackError as exception:
            self.close_connection()
            raise SonarConfigurationError(str(exception)) from exception
        except SonarError:
            self.close_connection()
            raise
        except (ConnectionError, OSError) as exception:
            self.close_connection()
            raise SonarConnectionError(
                'Omniscan 450 (%s) at %s:%d could not be initialized: %s'
                % (self.name, self.ip_address, port, exception)) from exception

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
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) \
                or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be greater than zero')

        try:
            data = self.omniscan450.wait_message(
                [definitions.OMNISCAN450_OS_MONO_PROFILE], timeout=timeout,
                latest=latest)
        except NackError as exception:
            raise SonarConfigurationError(str(exception)) from exception
        except (ConnectionError, OSError) as exception:
            raise SonarConnectionError(
                'Omniscan 450 (%s) at %s:%d connection failed: %s'
                % (self.name, self.ip_address, self.port, exception)) \
                from exception

        if data is None:
            raise SonarTimeoutError(
                'Omniscan 450 (%s) at %s:%d did not return a profile within '
                '%.3f s' % (self.name, self.ip_address, self.port, timeout))

        self._validate_profile(data)
        if not self._profile_matches(data, self.params):
            raise SonarConfigurationError(
                'Omniscan 450 (%s) profile does not match confirmed settings'
                % self.name)
        return data, Omniscan450.scale_power(data)

    def _validate_profile(self, data):
        """Validate all data used in ROS messages before exposing a profile."""
        if not 200 <= data.num_results <= definitions.OMNISCAN450_MAX_RESULTS:
            raise SonarDataError(
                'Omniscan 450 (%s) returned unsupported result count %d'
                % (self.name, data.num_results))

        if data.num_results != len(data.pwr_results):
            raise SonarDataError(
                'Omniscan 450 (%s) at %s advertised %d results but returned %d'
                % (self.name, self.ip_address, data.num_results,
                   len(data.pwr_results)))

        finite_fields = (
            'min_pwr_db', 'max_pwr_db', 'pulse_duration_sec', 'analog_gain',
            'transducer_heading_deg', 'vehicle_heading_deg',
        )
        if any(not math.isfinite(getattr(data, field)) for field in finite_fields):
            raise SonarDataError(
                'Omniscan 450 (%s) returned nonfinite profile metadata' % self.name)
        if data.min_pwr_db > data.max_pwr_db:
            raise SonarDataError(
                'Omniscan 450 (%s) at %s returned invalid dB bounds %r..%r'
                % (self.name, self.ip_address, data.min_pwr_db,
                   data.max_pwr_db))

        if data.length_mm <= 0 or data.sos_dmps <= 0 or data.ping_hz <= 0 \
                or not 0 <= data.gain_index <= 7 \
                or data.pulse_duration_sec <= 0:
            raise SonarDataError(
                'Omniscan 450 (%s) returned invalid geometry or gain' % self.name)

    @staticmethod
    def _profile_matches(data, params):
        """Compare only settings actually reported by the device."""
        return (data.start_mm == params.start_mm
                and data.length_mm == params.length_mm
                and data.num_results == params.num_results
                and abs(data.sos_dmps * 100 - params.speed_of_sound_mm) < 100
                and (params.gain_index == -1
                     or data.gain_index == params.gain_index))

    def set_parameters(self, params):
        """Apply a new SonarParams, sending only what actually changed.

        Returns the parameters now in effect.
        """
        if not isinstance(params, SonarParams):
            raise TypeError('params must be a SonarParams instance')

        previous = self.params
        change_speed_of_sound = (
            previous.speed_of_sound_mm != params.speed_of_sound_mm)

        # Every remaining field lives in the os_ping_params message, so any
        # difference means the whole message is resent.
        change_os_ping_params = any(
            getattr(previous, name) != getattr(params, name)
            for name in SonarParams.field_names()
            if name != 'speed_of_sound_mm'
        )

        if not change_speed_of_sound and not change_os_ping_params:
            return self.params

        try:
            self._apply(params, change_speed_of_sound, change_os_ping_params)
        except NackError as exception:
            self.close_connection()
            raise SonarConfigurationError(str(exception)) from exception
        except SonarError:
            self.close_connection()
            raise
        except (ConnectionError, OSError) as exception:
            # A partially applied configuration is uncertain. Close this
            # connection; the owner can reconnect and reapply desired settings.
            self.close_connection()
            raise SonarConnectionError(
                'Omniscan 450 (%s) failed to apply parameters: %s'
                % (self.name, exception)) from exception

        self.params = params

        return self.params

    def _apply(self, params, change_speed_of_sound=False,
               change_os_ping_params=False):
        """Send settings and confirm the fields observable in later profiles."""
        if change_speed_of_sound:
            self.omniscan450.control_set_speed_of_sound(
                speed_of_sound=params.speed_of_sound_mm)

        if change_os_ping_params:
            self._send_ping_params(params, enable=1)

        deadline = time.monotonic() + self.configuration_timeout
        # This ordered request consumes buffered profiles and command NACKs.
        # A subsequent profile cannot predate the reply on this TCP stream.
        if self.omniscan450.readDeviceInformation(
                timeout=max(0.001, deadline - time.monotonic())) is None:
            raise SonarConfigurationError('configuration barrier timed out')
        while time.monotonic() < deadline:
            data = self.omniscan450.wait_message(
                [definitions.OMNISCAN450_OS_MONO_PROFILE],
                timeout=deadline - time.monotonic())
            if data is None:
                break
            self._validate_profile(data)
            if self._profile_matches(data, params):
                return
        raise SonarConfigurationError(
            'Omniscan 450 (%s) settings did not appear in profiles within %.3f s'
            % (self.name, self.configuration_timeout))

    def _send_ping_params(self, params, enable):
        """Send os_ping_params with the current settings and enable flag."""
        self.omniscan450.control_os_ping_params(
            # Start of the ping range; normally 0.
            start_mm=params.start_mm,
            # Length of the returned profile; range ends at start + length.
            length_mm=params.length_mm,
            # Limits the ping rate; 0 gives the best ping rate.
            msec_per_ping=params.msec_per_ping,
            # % of total ping time for the current range, 0.002 typical.
            pulse_len_percent=params.pulse_len_percent,
            # 0.0015 typical.
            filter_duration_percent=params.filter_duration_percent,
            # -1 for auto gain, otherwise 0-7 sets the gain.
            gain_index=params.gain_index,
            # Signal data points in the returned profile (200-1200).
            num_results=params.num_results,
            # 1 or 0 to enable or disable pinging.
            enable=enable,
            reserved_1=0,
            reserved_2=0,
            reserved_3=0,
        )

    def close_connection(self):
        """Stop the sonar pinging and close the socket."""
        if self._closed:
            self.omniscan450.close()
            return

        # Cancellation interrupts acquisition. Permit one short best-effort
        # stop command during cleanup, owned by the same connection thread.
        self.omniscan450.cancel_event = None
        self.omniscan450._write_timeout = 0.1
        try:
            self._send_ping_params(self.params, enable=0)
        except (ConnectionError, OSError) as exception:
            self._log('warning',
                      'Omniscan 450 (%s) at %s could not be told to stop '
                      'pinging: %s' % (self.name, self.ip_address, exception))

        try:
            self.omniscan450.close()
        except OSError as exception:
            self._log('warning',
                      'Omniscan 450 (%s) at %s failed to close its socket: %s'
                      % (self.name, self.ip_address, exception))
        finally:
            self._closed = True

    def __enter__(self):
        """Return this connected driver for use as a context manager."""
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Close the sonar when leaving a context manager."""
        self.close_connection()

    def _log(self, level, message):
        if self._logger is not None:
            method = getattr(self._logger, level, None)
            if method is None and level == 'warning':
                method = getattr(self._logger, 'warn')
            method(message)
