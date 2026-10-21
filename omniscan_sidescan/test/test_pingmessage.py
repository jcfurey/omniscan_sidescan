# -*- coding: utf-8 -*-
"""Tests for the vendored ping-protocol encoder/decoder."""

import struct

from omniscan_sidescan.brping import definitions, Omniscan450, PingMessage, PingParser

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


def test_parser_recovers_when_start_marker_is_repeated():
    msg = build_profile(num_results=4)
    parser = PingParser()

    assert feed(parser, b'B' + msg.msg_data) == PingParser.NEW_MESSAGE
    assert parser.rx_msg.ping_number == 42


def test_parser_rejects_a_known_message_with_the_wrong_payload_length():
    msg = PingMessage(definitions.COMMON_DEVICE_INFORMATION)
    msg.device_type = 3
    msg.device_revision = 1
    msg.firmware_version_major = 1
    msg.firmware_version_minor = 2
    msg.firmware_version_patch = 3
    msg.reserved = 0
    msg.pack_msg_data()

    body = bytearray(msg.msg_data[:8 + 5])
    body[2:4] = struct.pack('<H', 5)
    malformed = body + struct.pack('<H', sum(body) & 0xffff)
    parser = PingParser()

    feed(parser, malformed)
    assert parser.parsed == 0
    assert parser.errors >= 1
    assert feed(parser, msg.msg_data) == PingParser.NEW_MESSAGE


def test_variable_length_text_message_round_trips():
    msg = PingMessage(definitions.COMMON_NACK)
    msg.nacked_id = definitions.OMNISCAN450_OS_PING_PARAMS
    msg.nack_message = b'unsupported value'
    msg.pack_msg_data()

    decoded = PingMessage(msg_data=msg.msg_data)

    assert decoded.valid
    assert decoded.nacked_id == definitions.OMNISCAN450_OS_PING_PARAMS
    assert decoded.nack_message == b'unsupported value'
    assert decoded.verify_checksum()


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


def test_speed_of_sound_matches_an_independent_cerulean_wire_fixture():
    class CaptureSocket:

        def __init__(self):
            self.data = bytearray()

        def send(self, data):
            self.data.extend(data)
            return len(data)

    sonar = Omniscan450()
    sonar.iodev = CaptureSocket()
    sonar.control_set_speed_of_sound(1500000)
    # BR, payload size 4, command 116, source/destination 0, 1500000 mm/s,
    # and checksum 613. These bytes are independent of the local definitions.
    assert sonar.iodev.data == bytes.fromhex('425204007400000060e316006502')


def test_invalid_length_does_not_consume_the_next_valid_profile():
    parser = PingParser()
    damaged = b'BR' + struct.pack('<HHBB', 65535, 2198, 0, 0)
    profile = build_profile(num_results=1200)
    assert feed(parser, damaged + profile.msg_data) == PingParser.NEW_MESSAGE
    assert parser.parsed == 1
    assert parser.errors >= 1


def test_incomplete_frame_expires_before_a_new_packet_is_read():
    parser = PingParser()
    profile = build_profile(num_results=200)
    feed(parser, profile.msg_data[:15])
    assert parser.expire(now=parser.frame_started + 1.1)
    assert feed(parser, profile.msg_data) == PingParser.NEW_MESSAGE
    assert parser.errors == 1
