"""Independent, cancellable acquisition and recovery for one sonar."""

from dataclasses import dataclass, replace
import threading

from .omniscan450_driver import (
    Omniscan450Driver,
    SonarConfigurationError,
    SonarConnectionError,
    SonarDataError,
    SonarError,
    SonarTimeoutError,
)


@dataclass(frozen=True)
class ProfileSample:
    """A validated profile and its immutable host receive metadata."""

    data: object
    scaled: object
    generation: int
    sequence: int
    received_monotonic: float
    timestamp_ns: int


@dataclass(frozen=True)
class ChannelSnapshot:
    """A consistent view of channel health and the newest valid profile."""

    state: str = 'disconnected'
    error: str = ''
    sample: object = None
    desired_generation: int = 0
    applied_generation: int = -1
    applied_params: object = None
    connection_attempts: int = 0
    connections: int = 0
    profiles_received: int = 0
    duplicate_profiles: int = 0
    parser_errors: int = 0


class SonarChannel:
    """Own a sonar socket in one worker; never block a ROS callback."""

    def __init__(self, name, ip_address, port, params, *, read_timeout=0.5,
                 connect_timeout=5.0, configuration_timeout=2.0,
                 max_consecutive_failures=10, reconnect_initial_delay=0.25,
                 reconnect_max_delay=10.0, logger=None, clock_ns=None,
                 notify=None):
        """Prepare an independent channel; start acquisition with start()."""
        self.name = name
        self.ip_address = ip_address
        self.port = port
        self._logger = logger
        self._clock_ns = clock_ns
        self._notify = notify
        self._connect_timeout = connect_timeout
        self._configuration_timeout = configuration_timeout
        self._max_failures = max_consecutive_failures
        self._initial_delay = reconnect_initial_delay
        self._max_delay = reconnect_max_delay
        self._settings = (params, 0, read_timeout)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._snapshot = ChannelSnapshot()
        self._sequence = 0
        self._thread = threading.Thread(
            target=self._run, name='omniscan-' + name, daemon=True)

    def start(self):
        """Start connecting and reading without waiting for the device."""
        self._thread.start()

    def configure(self, params, generation, read_timeout):
        """Queue committed desired settings for the owning worker."""
        with self._lock:
            self._settings = (params, generation, read_timeout)
            changed = generation != self._snapshot.desired_generation
            state = self._snapshot.state
            if changed and state in ('streaming', 'waiting'):
                state = 'configuring'
            self._snapshot = replace(
                self._snapshot, state=state, desired_generation=generation,
                sample=None if changed else self._snapshot.sample)

    def snapshot(self):
        """Return one consistent snapshot without waiting for network I/O."""
        with self._lock:
            return self._snapshot

    def stop(self):
        """Interrupt connection, configuration, read, and retry waits."""
        self._stop.set()

    def join(self):
        """Wait for the worker to stop before destroying its ROS resources."""
        if self._thread.ident is not None:
            self._thread.join()

    def _desired(self):
        with self._lock:
            return self._settings

    def _status(self, state, error='', **changes):
        with self._lock:
            previous = self._snapshot
            self._snapshot = replace(
                previous, state=state, error=error, **changes)
        if self._logger is not None and (
                state != previous.state or error != previous.error):
            message = '%s sonar: %s%s' % (
                self.name, state, ': ' + error if error else '')
            if error:
                self._logger.warning(message)
            else:
                self._logger.info(message)
        if self._notify is not None:
            self._notify()

    def _run(self):
        delay = self._initial_delay
        try:
            while not self._stop.is_set():
                driver = None
                sequence_before = self._sequence
                try:
                    previous = self.snapshot()
                    self._status(
                        'connecting', sample=None, applied_generation=-1,
                        applied_params=None,
                        connection_attempts=previous.connection_attempts + 1)
                    params, generation, _ = self._desired()
                    driver = Omniscan450Driver(
                        ip_address=self.ip_address, port=self.port,
                        params=params, name=self.name, logger=self._logger,
                        connect_timeout=self._connect_timeout,
                        configuration_timeout=self._configuration_timeout,
                        cancel_event=self._stop, clock_ns=self._clock_ns)
                    self._status(
                        'configuring', applied_generation=generation,
                        applied_params=params,
                        connections=self.snapshot().connections + 1)
                    self._acquire(driver, generation)
                except SonarError as exception:
                    if not self._stop.is_set():
                        self._status(
                            'retrying', str(exception), sample=None,
                            applied_generation=-1, applied_params=None)
                finally:
                    if driver is not None:
                        driver.close_connection()
                # A channel that successfully streamed starts with the small
                # delay again; repeated setup failures back off independently.
                streamed = self._sequence > sequence_before
                if streamed:
                    delay = self._initial_delay
                if self._stop.wait(delay):
                    break
                if not streamed:
                    delay = min(delay * 2.0, self._max_delay)
        except Exception as exception:
            # Programming errors must be visible instead of leaving a dead
            # worker that still appears healthy to the node.
            self._status('error', str(exception), sample=None,
                         applied_generation=-1, applied_params=None)
        else:
            self._status('stopped', sample=None, applied_generation=-1,
                         applied_params=None)

    def _acquire(self, driver, applied_generation):
        failures = 0
        last_ping = last_timestamp = None
        while not self._stop.is_set():
            params, generation, read_timeout = self._desired()
            if applied_generation != generation:
                self._status('configuring', sample=None)
                driver.set_parameters(params)
                applied_generation = generation
                self._status('configuring', applied_generation=generation,
                             applied_params=params)
            try:
                data, scaled = driver.get_data(timeout=read_timeout)
                snapshot = self.snapshot()
                received = snapshot.profiles_received + 1
                if last_ping is not None:
                    ping_delta = (data.ping_number - last_ping) & 0xffffffff
                    time_delta = (data.timestamp_ms - last_timestamp) & 0xffffffff
                    if ping_delta >= 0x80000000 or time_delta >= 0x80000000:
                        raise SonarConnectionError(
                            'device counters moved backwards; reconnecting')
                    if ping_delta == 0:
                        with self._lock:
                            self._snapshot = replace(
                                self._snapshot, profiles_received=received,
                                duplicate_profiles=snapshot.duplicate_profiles + 1)
                        raise SonarDataError('duplicate ping counter')
                last_ping, last_timestamp = data.ping_number, data.timestamp_ms
                failures = 0
                self._sequence += 1
                sample = ProfileSample(
                    data, scaled, generation, self._sequence,
                    data.received_monotonic, data.received_timestamp_ns)
                with self._lock:
                    # A parameter callback may have queued a newer generation
                    # while this socket read was in flight.
                    if generation != self._settings[1]:
                        continue
                    previous = self._snapshot
                    self._snapshot = replace(
                        previous, state='streaming', error='', sample=sample,
                        profiles_received=received,
                        parser_errors=driver.omniscan450.parser.errors)
                if self._logger is not None and previous.state != 'streaming':
                    self._logger.info('%s sonar: streaming' % self.name)
                if self._notify is not None:
                    self._notify()
            except (SonarTimeoutError, SonarDataError) as exception:
                failures += 1
                self._status('waiting', str(exception), sample=None,
                             parser_errors=driver.omniscan450.parser.errors)
                if self._max_failures and failures >= self._max_failures:
                    raise SonarConnectionError(
                        'persistent profile failures: %s' % exception) from exception
            except SonarConfigurationError:
                # A geometry change or NACK requires reconfiguration, not an
                # indefinitely suppressed combined stream.
                raise
