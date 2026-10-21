"""Deterministic TCP end-of-stream tests against the native connection API."""

import os
import socket
import struct
import subprocess
import time

from omniscan_sidescan.brping import PingMessage
from omniscan_sidescan.omniscan450_simulator import Omniscan450Simulator
import pytest


class FinalProfileSimulator(Omniscan450Simulator):
    """Expose the TCP peer so each test controls exactly when EOF is visible."""

    def _handle_connection(self, connection):
        self.client = connection
        super()._handle_connection(connection)


@pytest.mark.parametrize('ending', ['valid', 'newest', 'invalid', 'geometry', 'nack', 'partial'])
def test_final_profile_is_not_lost_or_replayed_at_eof(ending):
    """Return the last valid measurement once and never mask validation or NACKs."""
    binary = os.environ.get('OMNISCAN_CONNECTION_PROBE_EXECUTABLE')
    assert binary, 'Run through colcon test or set OMNISCAN_CONNECTION_PROBE_EXECUTABLE'
    with FinalProfileSimulator(port=0, ping_interval=10) as peer:
        process = subprocess.Popen([binary, str(peer.address[1])], stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            assert process.stdout.readline().strip() == 'READY'
            peer.enabled = False
            final_ping = peer.ping_number
            packet = bytearray(peer._build_profile())
            if ending == 'newest':
                packet.extend(peer._build_profile())
                final_ping += 1
            elif ending in ('invalid', 'geometry'):
                if ending == 'invalid':
                    struct.pack_into('<f', packet, 8 + 32, float('nan'))
                else:
                    struct.pack_into('<I', packet, 8 + 8, 7000)
                struct.pack_into('<H', packet, len(packet) - 2, sum(packet[:-2]) & 0xffff)
            elif ending == 'nack':
                nack = PingMessage(2)
                nack.nacked_id, nack.nack_message = 2197, b'rejected'
                packet.extend(nack.pack_msg_data())
            elif ending == 'partial':
                packet = packet[:100]
            peer.client.sendall(packet)
            peer.client.shutdown(socket.SHUT_WR)
            time.sleep(0.02)
            stdout, stderr = process.communicate(input='\n', timeout=2)
            assert process.returncode == 0, stdout + stderr
            expected = {'invalid': 'DATA_ERROR', 'geometry': 'CONFIGURATION_ERROR',
                        'nack': 'CONFIGURATION_ERROR', 'partial': 'CONNECTION_ERROR'}
            assert stdout.splitlines() == [expected.get(ending, f'PROFILE {final_ping}'), 'CLOSED']
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()
