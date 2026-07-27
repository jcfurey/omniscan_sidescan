# -*- coding: utf-8 -*-
"""Driver base class for the Omniscan sonars.
"""

import socket # For interacting with the sensor with TCP/IP.
import struct # Encode/decode the packets to be sent/received.

class OmniscanDriver(object):
    """Base class for interfacing with Imagenex sonars.
    """

    def __init__(self, ip_address, port, response_num_bytes):
        """Initialize the driver to talk to the sonar."""
        # TCP/IP connection.
        self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.connection.connect((ip_address, port))
        except (socket.error):
            self.connection = None

        self.response_num_bytes = response_num_bytes

    def get_data(self, msg):
        """Common method to get sonar data and return it."""
        raise NotImplementedError

    def set_parameters(self):
        """Set parameters."""
        raise NotImplementedError

    def _send_request(self):
        """Common method to send TCP request."""
        raise NotImplementedError

    def _read_data(self):
        """Receive request of data through TCP/IP. TODO(aql) more complete doc.
        """
        # Number of bytes received and data format.
        data_format = str(self.response_num_bytes) + "B"

        data = None
        if self.connection != None:
            # Receiving data.
            raw_data = self.connection.recv(self.response_num_bytes)
            data = struct.unpack(data_format,
                raw_data)

        return data

    def _interpret_data(self, data, message=None):
        """Common method to interpret raw data received as response."""
        raise NotImplementedError

    def close_connection(self):
        """Close TCP/IP connection.
        """
        if self.connection != None:
            self.connection.close()