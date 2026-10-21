# -*- coding: utf-8 -*-
"""Ping-protocol message definitions needed by the Omniscan 450.

Vendored and trimmed from ping-python (Blue Robotics, MIT) -- see LICENSE in
this directory.  Only the ``common`` messages and the ``omniscan450`` messages
are kept; the Ping1D / Ping360 / S500 / Surveyor240 definitions are omitted.

The wire values here mirror ping-protocol's ``common.json`` and
``omniscan450.json`` and must not be changed without a corresponding firmware
change.
"""

# Common messages, shared by every ping-protocol device.
COMMON_ACK = 1
COMMON_NACK = 2
COMMON_ASCII_TEXT = 3
COMMON_DEVICE_INFORMATION = 4
COMMON_PROTOCOL_VERSION = 5
COMMON_GENERAL_REQUEST = 6
COMMON_SET_DEVICE_ID = 100

# Variable length fields are formatted with 's', and always occur at the end of
# the payload.  The format string for those messages is adjusted at runtime --
# see PingMessage.get_payload_format().
payload_dict_common = {
    COMMON_ACK: {
        "name": "ack",
        "format": "H",
        "field_names": (
            "acked_id",
        ),
        "payload_length": 2,
    },

    COMMON_NACK: {
        "name": "nack",
        "format": "H",
        "field_names": (
            "nacked_id",
            "nack_message",
        ),
        "payload_length": 2,
    },

    COMMON_ASCII_TEXT: {
        "name": "ascii_text",
        "format": "",
        "field_names": (
            "ascii_message",
        ),
        "payload_length": 0,
    },

    COMMON_DEVICE_INFORMATION: {
        "name": "device_information",
        "format": "BBBBBB",
        "field_names": (
            "device_type",
            "device_revision",
            "firmware_version_major",
            "firmware_version_minor",
            "firmware_version_patch",
            "reserved",
        ),
        "payload_length": 6,
    },

    COMMON_PROTOCOL_VERSION: {
        "name": "protocol_version",
        "format": "BBBB",
        "field_names": (
            "version_major",
            "version_minor",
            "version_patch",
            "reserved",
        ),
        "payload_length": 4,
    },

    COMMON_GENERAL_REQUEST: {
        "name": "general_request",
        "format": "H",
        "field_names": (
            "requested_id",
        ),
        "payload_length": 2,
    },

    COMMON_SET_DEVICE_ID: {
        "name": "set_device_id",
        "format": "B",
        "field_names": (
            "device_id",
        ),
        "payload_length": 1,
    },
}

# Omniscan 450 specific messages.
OMNISCAN450_JSON_WRAPPER = 10
# Cerulean API and ping-protocol/src/definitions/omniscan450.json.
# 1002 belongs to other Ping devices, not the Omniscan 450.
OMNISCAN450_SET_SPEED_OF_SOUND = 116
OMNISCAN450_SYNC_CHANNEL_NUMBER = 169
OMNISCAN450_SET_SYNC_CHANNEL_NUMBER = 170
OMNISCAN450_OS_PING_PARAMS = 2197
OMNISCAN450_OS_MONO_PROFILE = 2198
OMNISCAN450_MAX_RESULTS = 1200

payload_dict_omniscan450 = {
    OMNISCAN450_SYNC_CHANNEL_NUMBER: {
        "name": "sync_channel_number",
        "format": "BB",
        "field_names": ("channel_number", "number_of_channels"),
        "payload_length": 2,
    },
    OMNISCAN450_SET_SYNC_CHANNEL_NUMBER: {
        "name": "set_sync_channel_number",
        "format": "BB",
        "field_names": ("channel_number", "number_of_channels"),
        "payload_length": 2,
    },
    OMNISCAN450_JSON_WRAPPER: {
        "name": "JSON_WRAPPER",
        "format": "",
        "field_names": (
            "string",
        ),
        "payload_length": 0,
    },

    OMNISCAN450_SET_SPEED_OF_SOUND: {
        "name": "set_speed_of_sound",
        "format": "I",
        "field_names": (
            "speed_of_sound",
        ),
        "payload_length": 4,
    },

    OMNISCAN450_OS_PING_PARAMS: {
        "name": "os_ping_params",
        "format": "IIIffffhHBBBB",
        "field_names": (
            "start_mm",
            "length_mm",
            "msec_per_ping",
            "reserved_1",
            "reserved_2",
            "pulse_len_percent",
            "filter_duration_percent",
            "gain_index",
            "num_results",
            "enable",
            "reserved_3",
            "reserved_4",
            "reserved_5",
        ),
        "payload_length": 36,
    },

    OMNISCAN450_OS_MONO_PROFILE: {
        "name": "os_mono_profile",
        "format": "IIIIIHHHBBffffff",
        "field_names": (
            "ping_number",
            "start_mm",
            "length_mm",
            "timestamp_ms",
            "ping_hz",
            "gain_index",
            "num_results",
            "sos_dmps",
            "channel_number",
            "reserved",
            "pulse_duration_sec",
            "analog_gain",
            "max_pwr_db",
            "min_pwr_db",
            "transducer_heading_deg",
            "vehicle_heading_deg",
            "pwr_results",
        ),
        "payload_length": 52,
    },
}

PINGMESSAGE_UNDEFINED = 0

payload_dict_all = {
    PINGMESSAGE_UNDEFINED: {
        "name": "undefined",
        "format": "",
        "field_names": (),
        "payload_length": 0,
    },
}
payload_dict_all.update(payload_dict_common)
payload_dict_all.update(payload_dict_omniscan450)
