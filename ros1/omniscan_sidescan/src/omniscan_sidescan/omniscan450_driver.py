#!/usr/bin/env python

"""
  Author: Monika Roznere
  Affiliation: Binghamton University
  Date: 07/19/2025

  Description:
  Class for interacting with Omniscan 450 Side Scan Sonar.
"""

import struct # Preparing the packets to be sent.
from enum import Enum

import time # For short delay in changing parameters

from builtins import super # For compatibility between python 2 and 3.

# from omniscan_sidescan.sonar_driver import OmniscanDriver

from brping import definitions
from brping import Omniscan450


# Sonar default values.
SONAR_IP_ADDRESS = "192.168.2.92" # port
SONAR_PORT = 51200
SPEED_OF_SOUND_MM = 1482000
START_MM = 0
LENGTH_MM = 5000 # 5 meters
MSEC_PER_PING = 0 # best ping results, set to 0
PULSE_LEN_PERCENT = 0.002
FILTER_DURATION_PERCENT = 0.0015
GAIN_INDEX = -1
NUM_RESULTS = 600

# Strings for topics.
SENSOR_NAME = '450'
TOPIC_SEPARATOR = '/'
SONAR_TOPIC_NAME = 'sonar'

# END MACROS.

class Omniscan450Driver():
    def __init__(self, ip_address=SONAR_IP_ADDRESS, port=SONAR_PORT,
        speed_of_sound_mm=SPEED_OF_SOUND_MM, start_mm=START_MM, length_mm=LENGTH_MM,
        msec_per_ping=MSEC_PER_PING, pulse_len_percent=PULSE_LEN_PERCENT,
        filter_duration_percent=FILTER_DURATION_PERCENT,
        gain_index=GAIN_INDEX, num_results=NUM_RESULTS):

        self.ip_address = ip_address
        # self.port = port

        # Setting initial parameters
        self.speed_of_sound_mm = speed_of_sound_mm
        self.start_mm = start_mm
        self.length_mm = length_mm
        self.msec_per_ping = msec_per_ping
        self.pulse_len_percent = pulse_len_percent
        self.filter_duration_percent = filter_duration_percent
        self.gain_index = gain_index
        self.num_results = num_results

        self.omniscan450 = Omniscan450()

        # Establish connection.
        self.omniscan450.connect_tcp(self.ip_address, port)
        if self.omniscan450.initialize() is False:
            print("Omniscan 450 SS Port %s is closed" % self.ip_address)

        data = self.omniscan450.readDeviceInformation()
        print("Device type: %s" % data.device_type)

        # my_msec_per_ping = Omniscan450.calc_msec_per_ping(1000)
        # my_pulse_length = Omniscan450.calc_pulse_length_pc(0.2)
        # self.omniscan.control_os_ping_params(enable=1)

        # Set initial parameters
        self._set_sidescan_params(change_speed_of_sound=True, change_os_ping_params=True)


    def get_data(self):
        """Get data from the sonar and return raw data and scaled result
        """
        data = self.omniscan450.wait_message([definitions.OMNISCAN450_OS_MONO_PROFILE])
        if data:
            scaled_results = Omniscan450.scale_power(data)
            try:
                check = sum(scaled_results) / len(scaled_results)
            except ZeroDivisionError:
                print("Omniscan 450 SS Port %s : length of scaled result is 0" % self.ip_address)
        elif not data:
            print("Omniscan 450 SS Port %s failed to get message" % self.ip_address)

        return data, scaled_results


    def set_parameters(self, config):
        """Set parameters

        Args:
            config: structure coming from dynamic_reconfigure.
        """
        change_speed_of_sound = False
        if self.speed_of_sound_mm != config.speed_of_sound_mm:
            self.speed_of_sound_mm = config.speed_of_sound_mm
            change_speed_of_sound = True

        change_os_ping_params = False
        if self.start_mm != config.start_mm:
            self.length_mm = config.length_mm
            change_os_ping_params = True
        
        if self.length_mm != config.length_mm:
            self.length_mm = config.length_mm
            change_os_ping_params = True
            
        if self.msec_per_ping != config.msec_per_ping:
            self.msec_per_ping = config.msec_per_ping
            change_os_ping_params = True
        
        if self.pulse_len_percent != config.pulse_len_percent:
            self.pulse_len_percent = config.pulse_len_percent
            change_os_ping_params = True

        if self.filter_duration_percent != config.filter_duration_percent:
            self.filter_duration_percent = config.filter_duration_percent
            change_os_ping_params = True

        if self.gain_index != config.gain_index:
            self.gain_index = config.gain_index
            change_os_ping_params = True

        if self.num_results != config.num_results:
            self.num_results = config.num_results
            change_os_ping_params = True
        
        self._set_sidescan_params(change_speed_of_sound, change_os_ping_params)

        return config


    def _set_sidescan_params(self, change_speed_of_sound=False,
                                    change_os_ping_params=False):
        if change_speed_of_sound:
            self.omniscan450.control_set_speed_of_sound(
                speed_of_sound=self.speed_of_sound_mm
            )

        if change_os_ping_params:
            self.omniscan450.control_os_ping_params(
                start_mm=self.start_mm,                     # Start of ping range, set to 0
                length_mm=self.length_mm,                   # Length of the returned profile, end of range = start_mm + length_mm
                msec_per_ping=self.msec_per_ping,           # Set value to limit ping rate, normally set to 0 for best ping rate 
                pulse_len_percent=self.pulse_len_percent,   # % of total ping time for current range, 0.002 typical
                filter_duration_percent=self.filter_duration_percent,   # 0.0015 typical
                gain_index=self.gain_index,                 # Set to -1 for auto gain, otherwise 0-7 sets gain
                num_results=self.num_results,               # Number of signal data points in returning profile (200-1200), 600 typical
                enable=1,                                   # 1 or 0 to enable or disable pinging
                reserved_1=0,
                reserved_2=0,
                reserved_3=0
                # reserved_4=0,
                # reserved_5=0
            )

        time.sleep(0.01)


    def close_connection(self):
        # self.omniscan450.control_os_ping_params(enable=0)
        self.omniscan450.control_os_ping_params(
                start_mm=self.start_mm,                     # Start of ping range, set to 0
                length_mm=self.length_mm,                   # Length of the returned profile, end of range = start_mm + length_mm
                msec_per_ping=self.msec_per_ping,           # Set value to limit ping rate, normally set to 0 for best ping rate 
                pulse_len_percent=self.pulse_len_percent,   # % of total ping time for current range, 0.002 typical
                filter_duration_percent=self.filter_duration_percent,   # 0.0015 typical
                gain_index=self.gain_index,                 # Set to -1 for auto gain, otherwise 0-7 sets gain
                num_results=self.num_results,               # Number of signal data points in returning profile (200-1200), 600 typical
                enable=0,                                   # 1 or 0 to enable or disable pinging
                reserved_1=0,
                reserved_2=0,
                reserved_3=0
                # reserved_4=0,
                # reserved_5=0
            )

        if self.omniscan450.iodev:
            try:
                self.omniscan450.iodev.close()
            except Exception as e:
                print(f"Failed to close socket: {e}")