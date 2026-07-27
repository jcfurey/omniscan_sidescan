# -*- coding: utf-8 -*-
"""Tests for the vendored ping-protocol encoder/decoder."""

import struct

from omniscan_sidescan.brping import definitions, PingMessage, PingParser

import pytest


def feed(parser, buffer):
    """Feed a buffer to a parser and return the final parse state."""
    state = None
    for byte in buffer:
        state = parser.parse_byte(byte)
    return state


def build_profile(num_results=8, min_pwr_db=-90.0, max_pwr_db=0.0,
                  results=None):
    """Build a packed os_mono_profile message."""
    msg = PingMessage(definitions.OMNISCAN450_OS_MONO_PROFILE)
    msg.ping_number = 42
    msg.start_mm = 0
    msg.length_mm = 5000
    msg.timestamp_ms = 1234
    msg.ping_hz = 450000
    msg.gain_index = 3
    msg.num_results = num_results
    msg.sos_dmps = 14820
    msg.channel_number = 1
    msg.reserved = 0
    msg.pulse_duration_sec = 0.0002
    msg.analog_gain = 1.0
    msg.max_pwr_db = max_pwr_db
    msg.min_pwr_db = min_pwr_db
    msg.transducer_heading_deg = 90.0
    msg.vehicle_heading_deg = 12.5

    if results is None:
        results = list(range(num_results))
    msg.pwr_results = struct.pack('<%dH' % num_results, *results)
    msg.pack_msg_data()
    return msg


def test_fixed_length_message_round_trips():
    msg = PingMessage(definitions.COMMON_DEVICE_INFORMATION)
    msg.device_type = 3
    msg.device_revision = 1
    msg.firmware_version_major = 1
    msg.firmware_version_minor = 2
    msg.firmware_version_patch = 3
    msg.reserved = 0
    msg.pack_msg_data()

    decoded = PingMessage(msg_data=msg.msg_data)

    assert decoded.message_id == definitions.COMMON_DEVICE_INFORMATION
    assert decoded.device_type == 3
    assert decoded.firmware_version_patch == 3
    assert decoded.verify_checksum()


def test_request_carries_the_requested_id_in_its_payload():
    msg = PingMessage(definitions.COMMON_GENERAL_REQUEST)
    msg.requested_id = definitions.OMNISCAN450_OS_MONO_PROFILE
    msg.pack_msg_data()

    decoded = PingMessage(msg_data=msg.msg_data)

    assert decoded.message_id == definitions.COMMON_GENERAL_REQUEST
    assert decoded.requested_id == definitions.OMNISCAN450_OS_MONO_PROFILE
    assert decoded.verify_checksum()


def test_ping_params_round_trips_with_negative_gain():
    msg = PingMessage(definitions.OMNISCAN450_OS_PING_PARAMS)
    msg.start_mm = 0
    msg.length_mm = 6000
    msg.msec_per_ping = 0
    msg.pulse_len_percent = 0.002
    msg.filter_duration_percent = 0.0015
    msg.gain_index = -1
    msg.num_results = 600
    msg.enable = 1
    msg.pack_msg_data()

    decoded = PingMessage(msg_data=msg.msg_data)

    assert decoded.length_mm == 6000
    # gain_index is a signed short, so auto gain has to survive the round trip.
    assert decoded.gain_index == -1
    assert decoded.num_results == 600
    assert decoded.enable == 1
    assert decoded.verify_checksum()


def test_variable_length_profile_round_trips():
    msg = build_profile(num_results=8)

    decoded = PingMessage(msg_data=msg.msg_data)

    assert decoded.message_id == definitions.OMNISCAN450_OS_MONO_PROFILE
    assert decoded.num_results == 8
    assert len(decoded.pwr_results) == 16  # 8 u16 values
    assert decoded.verify_checksum()
    # The static portion is 52 bytes plus the variable field.
    assert decoded.payload_length == 52 + 16


def test_parser_decodes_a_message_from_a_byte_stream():
    msg = build_profile(num_results=4)
    parser = PingParser()

    assert feed(parser, msg.msg_data) == PingParser.NEW_MESSAGE
    assert parser.rx_msg.ping_number == 42
    assert parser.parsed == 1
    assert parser.errors == 0


def test_parser_recovers_after_leading_garbage():
    msg = build_profile(num_results=4)
    parser = PingParser()

    # 'B' without a following 'R' has to send the parser back to the start.
    assert feed(parser, b'\x00BXjunk' + msg.msg_data) == PingParser.NEW_MESSAGE
    assert parser.parsed == 1


def test_parser_reports_a_bad_checksum():
    msg = build_profile(num_results=4)
    corrupted = bytearray(msg.msg_data)
    corrupted[-1] ^= 0xff
    parser = PingParser()

    assert feed(parser, corrupted) == PingParser.ERROR
    assert parser.errors == 1


def test_parser_handles_back_to_back_messages():
    first = build_profile(num_results=4)
    second = build_profile(num_results=6)
    parser = PingParser()

    states = []
    for byte in first.msg_data + second.msg_data:
        states.append(parser.parse_byte(byte))

    assert states.count(PingParser.NEW_MESSAGE) == 2
    assert parser.parsed == 2
    assert parser.errors == 0


def test_unknown_message_id_raises_on_construction():
    with pytest.raises(KeyError):
        PingMessage(9999)
