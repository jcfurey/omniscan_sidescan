# -*- coding: utf-8 -*-
"""Encoder/decoder for the Blue Robotics 'Ping' binary message protocol.

Vendored and trimmed from ping-python (Blue Robotics, MIT) -- see LICENSE in
this directory.  Behaviour is unchanged; only the set of known message ids is
reduced to the ones the Omniscan 450 uses.
"""

import struct

from . import definitions

payload_dict = definitions.payload_dict_all

# Messages whose final field is an ASCII string of run-time length.
asciiMsgs = [
    definitions.COMMON_NACK,
    definitions.COMMON_ASCII_TEXT,
    definitions.OMNISCAN450_JSON_WRAPPER,
]

# Messages whose final field is a byte vector of run-time length.
variable_msgs = [definitions.OMNISCAN450_OS_MONO_PROFILE]


class PingMessage(object):
    """A single ping-protocol packet.

    Construct with a message id to build an outgoing message (populate the
    payload fields as attributes, then call ``pack_msg_data()``), or with
    ``msg_data`` to decode an incoming byte buffer.
    """

    ## header start byte 1
    start_1 = ord("B")

    ## header start byte 2
    start_2 = ord("R")

    ## header struct format
    header_format = "BBHHBB"

    ## checksum struct format
    checksum_format = "H"

    ## data endianness for struct formatting
    endianess = "<"

    ## names of the header fields
    header_field_names = (
        "start_1",
        "start_2",
        "payload_length",
        "message_id",
        "src_device_id",
        "dst_device_id")

    ## number of bytes in a header
    headerLength = 8

    ## number of bytes in a checksum
    checksumLength = 2

    def __init__(self, msg_id=0, msg_data=None):
        ## The message id
        self.message_id = msg_id

        ## The request id for request messages
        self.request_id = None

        ## The message destination
        self.dst_device_id = 0
        ## The message source
        self.src_device_id = 0
        ## The message checksum
        self.checksum = 0

        ## The raw data buffer for this message, updated by pack_msg_data()
        self.msg_data = None

        ## Number of bytes in the message payload
        self.payload_length = 0

        ## Whether an incoming message was structurally valid
        self.valid = False

        if msg_data is not None:
            # Decode an incoming buffer.
            if not self.unpack_msg_data(msg_data):
                # Attempted to create an unknown message.
                return
        else:
            # Build an outgoing message with zeroed payload fields.
            try:
                ## The name of this message
                self.name = payload_dict[self.message_id]["name"]

                ## The field names of this message
                self.payload_field_names = payload_dict[self.message_id]["field_names"]

                for attr in self.payload_field_names:
                    setattr(self, attr, 0)

                if (self.message_id in variable_msgs
                        or self.message_id in asciiMsgs):
                    setattr(self, self.payload_field_names[-1], bytearray())

                self.update_payload_length()

                ## The struct formatting string for the message payload
                self.payload_format = self.get_payload_format()

                self.valid = True

            except KeyError:
                raise KeyError("message id not recognized: %d" % self.message_id)

    def pack_msg_data(self):
        """Pack the object attributes into ``self.msg_data`` and return it."""
        # Necessary for variable length payloads: use the current contents of
        # the variable length field.
        self.update_payload_length()

        msg_format = (PingMessage.endianess + PingMessage.header_format
                      + self.get_payload_format())

        attrs = (PingMessage.header_field_names
                 + payload_dict[self.message_id]["field_names"])

        values = []
        for attr in attrs:
            # Requests carry the requested id in the message_id header slot.
            if attr == "message_id" and self.request_id is not None:
                values.append(self.request_id)
            else:
                values.append(getattr(self, attr))

        self.msg_data = bytearray(struct.pack(msg_format, *values))

        self.msg_data += bytearray(struct.pack(
            PingMessage.endianess + PingMessage.checksum_format,
            self.update_checksum()))

        return self.msg_data

    def unpack_msg_data(self, msg_data):
        """Unpack a byte buffer into object attributes.

        Returns True on success, False if the message id is unknown.
        """
        self.msg_data = msg_data

        if len(self.msg_data) < self.headerLength + self.checksumLength:
            return False

        try:
            header = struct.unpack(
                PingMessage.endianess + PingMessage.header_format,
                self.msg_data[0:PingMessage.headerLength])
        except struct.error:
            return False

        for i, attr in enumerate(PingMessage.header_field_names):
            setattr(self, attr, header[i])

        if self.start_1 != ord('B') or self.start_2 != ord('R'):
            return False

        try:
            ## The name of this message
            self.name = payload_dict[self.message_id]["name"]
        except KeyError:
            return False

        expected_size = (
            self.headerLength + self.payload_length + self.checksumLength)
        if len(self.msg_data) != expected_size:
            return False

        static_length = payload_dict[self.message_id]["payload_length"]
        if self.message_id in variable_msgs or self.message_id in asciiMsgs:
            if self.payload_length < static_length:
                return False
        elif self.payload_length != static_length:
            return False

        ## The field names of this message
        self.payload_field_names = payload_dict[self.message_id]["field_names"]

        if self.payload_length > 0:
            ## The struct formatting string for the message payload
            self.payload_format = self.get_payload_format()

            try:
                payload = struct.unpack(
                    PingMessage.endianess + self.payload_format,
                    self.msg_data[PingMessage.headerLength:
                                  PingMessage.headerLength
                                  + self.payload_length])
            except struct.error:
                return False

            for i, attr in enumerate(self.payload_field_names):
                try:
                    setattr(self, attr, payload[i])
                except IndexError:
                    # Empty trailing variable length field.
                    if (self.message_id in variable_msgs
                            or self.message_id in asciiMsgs):
                        setattr(self, attr, bytearray())
        elif self.message_id in variable_msgs or self.message_id in asciiMsgs:
            setattr(self, self.payload_field_names[-1], bytearray())

        try:
            self.checksum = struct.unpack(
                PingMessage.endianess + PingMessage.checksum_format,
                self.msg_data[PingMessage.headerLength + self.payload_length:
                              PingMessage.headerLength + self.payload_length
                              + PingMessage.checksumLength])[0]
        except struct.error:
            return False
        self.valid = True
        return True

    def calculate_checksum(self):
        """Calculate the checksum over the header and payload bytes."""
        return sum(
            self.msg_data[0:PingMessage.headerLength + self.payload_length]
        ) & 0xffff

    def update_checksum(self):
        """Recalculate and store the checksum, and return it."""
        self.checksum = self.calculate_checksum()
        return self.checksum

    def verify_checksum(self):
        """Return True if the stored checksum matches the calculated one."""
        return self.checksum == self.calculate_checksum()

    def update_payload_length(self):
        """Set payload_length from the current contents of the message."""
        if self.message_id in variable_msgs or self.message_id in asciiMsgs:
            # The last field is always the single dynamic length field.
            self.payload_length = (
                payload_dict[self.message_id]["payload_length"]
                + len(getattr(self, self.payload_field_names[-1])))
        else:
            self.payload_length = payload_dict[self.message_id]["payload_length"]

    def get_payload_format(self):
        """Return the struct format string for this message's payload."""
        if self.message_id in variable_msgs or self.message_id in asciiMsgs:
            # Subtract the static portion from the total payload length.
            var_length = (self.payload_length
                          - payload_dict[self.message_id]["payload_length"])
            if var_length <= 0:
                return payload_dict[self.message_id]["format"]

            return payload_dict[self.message_id]["format"] + str(var_length) + "s"

        return payload_dict[self.message_id]["format"]

    def __repr__(self):
        header_string = "Header:"
        for attr in PingMessage.header_field_names:
            header_string += " " + attr + ": " + str(getattr(self, attr))

        if self.payload_length == 0:
            payload_string = ""
        else:
            payload_string = "Payload:"

            if self.message_id in variable_msgs:
                for attr in payload_dict[self.message_id]["field_names"][:-1]:
                    payload_string += "\n  - " + attr + ": " + str(getattr(self, attr))

                # The variable length field is always the last field; render it
                # as hex rather than as a string.
                attr = payload_dict[self.message_id]["field_names"][-1:][0]
                payload_string += "\n  - " + attr + ": " + str(
                    [hex(item) for item in getattr(self, attr)])
            else:
                for attr in payload_dict[self.message_id]["field_names"]:
                    payload_string += "\n  - " + attr + ": " + str(getattr(self, attr))

        return (
            "\n\n--------------------------------------------------\n"
            "ID: " + str(self.message_id) + " - " + self.name + "\n"
            + header_string + "\n"
            + payload_string + "\n"
            + "Checksum: " + str(self.checksum)
            + " check: " + str(self.calculate_checksum())
            + " pass: " + str(self.verify_checksum())
        )


class PingParser(object):
    """Digests a byte stream and decodes PingMessages."""

    __slots__ = (
        "buf",
        "state",
        "payload_length",
        "message_id",
        "errors",
        "parsed",
        "rx_msg",
    )

    NEW_MESSAGE = 0       # Just got a complete checksum-verified message
    WAIT_START = 1        # Waiting for the first character of a message 'B'
    WAIT_HEADER = 2       # Waiting for the second character of 'BR'
    WAIT_LENGTH_L = 3     # Waiting for the low byte of the payload length
    WAIT_LENGTH_H = 4     # Waiting for the high byte of the payload length
    WAIT_MSG_ID_L = 5     # Waiting for the low byte of the message id
    WAIT_MSG_ID_H = 6     # Waiting for the high byte of the message id
    WAIT_SRC_ID = 7       # Waiting for the source device id
    WAIT_DST_ID = 8       # Waiting for the destination device id
    WAIT_PAYLOAD = 9      # Waiting for the last byte of the payload
    WAIT_CHECKSUM_L = 10  # Waiting for the checksum low byte
    WAIT_CHECKSUM_H = 11  # Waiting for the checksum high byte
    ERROR = 12            # Checksum didn't check out

    def __init__(self):
        self.buf = bytearray()
        self.state = self.WAIT_START
        self.payload_length = 0  # remaining for the message being parsed
        self.message_id = 0      # of the message being parsed
        self.errors = 0
        self.parsed = 0
        self.rx_msg = None       # most recently parsed message

    def wait_start(self, msg_byte):
        self.buf = bytearray()
        if msg_byte == ord('B'):
            self.buf.append(msg_byte)
            self.state += 1

    def wait_header(self, msg_byte):
        if msg_byte == ord('R'):
            self.buf.append(msg_byte)
            self.state += 1
        elif msg_byte == ord('B'):
            # The second B may be the beginning of a valid ``BR`` sequence.
            self.buf = bytearray([msg_byte])
            self.state = self.WAIT_HEADER
        else:
            self.state = self.WAIT_START

    def wait_length_l(self, msg_byte):
        self.payload_length = msg_byte
        self.buf.append(msg_byte)
        self.state += 1

    def wait_length_h(self, msg_byte):
        self.payload_length |= (msg_byte << 8)
        self.buf.append(msg_byte)
        self.state += 1

    def wait_msg_id_l(self, msg_byte):
        self.message_id = msg_byte
        self.buf.append(msg_byte)
        self.state += 1

    def wait_msg_id_h(self, msg_byte):
        self.message_id |= (msg_byte << 8)
        self.buf.append(msg_byte)
        self.state += 1

    def wait_src_id(self, msg_byte):
        self.buf.append(msg_byte)
        self.state += 1

    def wait_dst_id(self, msg_byte):
        self.buf.append(msg_byte)
        self.state += 1
        if self.payload_length == 0:  # no payload bytes -> skip waiting
            self.state += 1

    def wait_payload(self, msg_byte):
        self.buf.append(msg_byte)
        self.payload_length -= 1
        if self.payload_length == 0:  # no payload bytes remaining
            self.state += 1

    def wait_checksum_l(self, msg_byte):
        self.buf.append(msg_byte)
        self.state += 1

    def wait_checksum_h(self, msg_byte):
        self.state = self.WAIT_START
        self.payload_length = 0
        self.message_id = 0

        self.buf.append(msg_byte)
        self.rx_msg = PingMessage(msg_data=self.buf)

        if self.rx_msg.valid and self.rx_msg.verify_checksum():
            self.parsed += 1
            return self.NEW_MESSAGE

        self.errors += 1
        return self.ERROR

    def parse_byte(self, msg_byte):
        """Feed the parser a single byte and return the resulting parse state.

        Returns ``PingParser.NEW_MESSAGE`` when the byte completes a valid
        message; the decoded PingMessage is then available as ``self.rx_msg``
        until the next message is decoded.
        """
        # Offset by 1 because NEW_MESSAGE isn't processed -- start at WAIT_START.
        result = self._PARSE_BYTE[self.state - 1](self, msg_byte)

        return self.state if result is None else result

    # Parsing methods in parser-state order.  At the bottom because the methods
    # must already be defined.
    _PARSE_BYTE = (
        wait_start,
        wait_header,
        wait_length_l,
        wait_length_h,
        wait_msg_id_l,
        wait_msg_id_h,
        wait_src_id,
        wait_dst_id,
        wait_payload,
        wait_checksum_l,
        wait_checksum_h,
    )
