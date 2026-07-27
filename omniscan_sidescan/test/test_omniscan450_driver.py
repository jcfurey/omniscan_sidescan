# -*- coding: utf-8 -*-
"""Tests for the sonar driver, run against the simulator."""

import time

from omniscan_sidescan.brping import definitions, Omniscan450
from omniscan_sidescan.omniscan450_driver import (
    Omniscan450Driver,
    SonarError,
    SonarParams,
)
from omniscan_sidescan.omniscan450_simulator import Omniscan450Simulator

import pytest


def wait_for(predicate, timeout=5.0, interval=0.01):
    """Poll until predicate() is true; returns whether it became true.

    The simulator runs in its own thread, so a message the driver has written
    is not necessarily processed by the time the write returns.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


@pytest.fixture
def simulator():
    """Run a simulator bound to an ephemeral port."""
    sonar = Omniscan450Simulator(port=0, ping_interval=0.01)
    sonar.start()
    yield sonar
    sonar.stop()


@pytest.fixture
def driver(simulator):
    """Connect a driver to the simulator."""
    host, port = simulator.address
    connected = Omniscan450Driver(ip_address=host, port=port, name='test')
    yield connected
    connected.close_connection()


def test_connect_pushes_the_initial_parameters(simulator):
    params = SonarParams(length_mm=7000, num_results=400, gain_index=4)
    host, port = simulator.address

    sonar = Omniscan450Driver(ip_address=host, port=port, params=params,
                              name='test')
    try:
        # The driver sends both parameter messages on connect.
        assert wait_for(lambda: simulator.length_mm == 7000)
        assert simulator.num_results == 400
        assert simulator.gain_index == 4
        assert simulator.enabled is True
        assert simulator.speed_of_sound_mm == params.speed_of_sound_mm
    finally:
        sonar.close_connection()


def test_connect_to_a_closed_port_raises_sonar_error():
    with pytest.raises(SonarError):
        Omniscan450Driver(ip_address='127.0.0.1', port=1, name='test',
                          connect_timeout=0.5)


def test_get_data_returns_a_profile_and_scaled_results(driver):
    data, scaled = driver.get_data(timeout=2.0)

    assert data.message_id == definitions.OMNISCAN450_OS_MONO_PROFILE
    assert data.num_results == len(data.pwr_results) == len(scaled)
    # Scaling maps u16 counts onto the message's own dB range.
    assert min(scaled) >= data.min_pwr_db
    assert max(scaled) <= data.max_pwr_db


def test_scale_power_maps_the_extremes_of_the_range():
    class FakeProfile(object):
        min_pwr_db = -90.0
        max_pwr_db = 0.0
        pwr_results = (0, 65535, 32768)

    scaled = Omniscan450.scale_power(FakeProfile())

    assert scaled[0] == pytest.approx(-90.0)
    assert scaled[1] == pytest.approx(0.0)
    assert scaled[2] == pytest.approx(-45.0, abs=0.01)


def test_get_data_times_out_when_the_sonar_stops_pinging(driver, simulator):
    driver.get_data(timeout=2.0)
    simulator.enabled = False

    # get_data drains the backlog, so one further read clears whatever was
    # already in flight and the next one has nothing left to return.
    try:
        driver.get_data(timeout=0.2)
    except SonarError:
        pass

    with pytest.raises(SonarError):
        driver.get_data(timeout=0.2)


def test_get_data_returns_the_newest_buffered_profile(driver, simulator):
    driver.get_data(timeout=2.0)

    # Let a backlog build up, then check the read skips to the newest profile
    # rather than working through the queue one ping at a time.
    time.sleep(0.3)
    first, _ = driver.get_data(timeout=2.0)
    second, _ = driver.get_data(timeout=2.0)

    assert second.ping_number > first.ping_number
    # The backlog was several pings deep, but the reads stay adjacent to it.
    assert second.ping_number - first.ping_number < 10


def test_set_parameters_only_resends_what_changed(driver, simulator):
    driver.get_data(timeout=2.0)

    updated = SonarParams(length_mm=9000)
    driver.set_parameters(updated)

    assert wait_for(lambda: simulator.length_mm == 9000)
    assert driver.params.length_mm == 9000


def test_profiles_track_the_requested_result_count(driver, simulator):
    driver.set_parameters(SonarParams(num_results=256))

    for _ in range(100):
        data, scaled = driver.get_data(timeout=2.0)
        if data.num_results == 256:
            break
    assert data.num_results == 256
    assert len(data.pwr_results) == 256
    assert len(scaled) == 256


def test_each_driver_keeps_its_own_receive_buffer():
    """Two sonars in one process must not read each other's bytes.

    Upstream ping-python declares the receive buffer at class scope, so every
    device shares it and profiles get attributed to the wrong sonar.
    """
    first = Omniscan450Simulator(port=0, channel_number=1, ping_interval=0.01)
    second = Omniscan450Simulator(port=0, channel_number=2, ping_interval=0.01)
    first.start()
    second.start()

    first_driver = second_driver = None
    try:
        first_driver = Omniscan450Driver(
            ip_address=first.address[0], port=first.address[1], name='first')
        second_driver = Omniscan450Driver(
            ip_address=second.address[0], port=second.address[1], name='second')

        assert (first_driver.omniscan450._input_buffer
                is not second_driver.omniscan450._input_buffer)

        # Each simulator stamps its own channel number into every profile, so
        # a shared buffer shows up as a profile from the wrong channel.
        for _ in range(10):
            first_data, _ = first_driver.get_data(timeout=2.0)
            second_data, _ = second_driver.get_data(timeout=2.0)
            assert first_data.channel_number == 1
            assert second_data.channel_number == 2
    finally:
        if first_driver is not None:
            first_driver.close_connection()
        if second_driver is not None:
            second_driver.close_connection()
        first.stop()
        second.stop()


def test_close_connection_disables_pinging(simulator):
    host, port = simulator.address
    sonar = Omniscan450Driver(ip_address=host, port=port, name='test')
    assert wait_for(lambda: simulator.enabled is True)

    sonar.close_connection()

    assert wait_for(lambda: simulator.enabled is False)


def test_sonar_params_field_names_match_the_dataclass():
    names = SonarParams.field_names()

    assert names[0] == 'speed_of_sound_mm'
    assert set(names) == {
        'speed_of_sound_mm', 'start_mm', 'length_mm', 'msec_per_ping',
        'pulse_len_percent', 'filter_duration_percent', 'gain_index',
        'num_results',
    }
