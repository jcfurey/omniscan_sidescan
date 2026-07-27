#!/usr/bin/env python3

"""
  Author: Monika Roznere
  Affiliation: Binghamton University
  Date: 07/19/2025

  Description:
  Publishes sensor reading from the omniscan sidescan.
"""

# Python general imports
from string import Template # template string

# ROS related imports
import rospy

from dynamic_reconfigure.server import Server
# Drivers.
from omniscan_sidescan.msg import SideScanSonar
from omniscan_sidescan.msg import SideScanSonarRaw

from omniscan_sidescan.omniscan450_driver import Omniscan450Driver
# from omniscan_sidescan.cfg import Omniscan450Config as OmniscanConfig
from omniscan_sidescan.cfg import Omniscan450Config

from brping import definitions
from brping import Omniscan450

import time

# Strings for topics.
SENSOR_NAME = 'omniscan'
TOPIC_SEPARATOR = '/'
SONAR_TOPIC_NAME = 'range'
SONAR_RAW_TOPIC_NAME = 'range_raw'

# Constants.
OMNISCAN450 = "450"

SENSOR_NUMBER = "450" # Default sensor number

# Parameters for node.
PORT_IP_ADDRESS = "192.168.2.92"
PORT_PORT = 51200
STARBOARD_IP_ADDRESS = "192.168.2.93"
STARBOARD_PORT = 51200
SPEED_OF_SOUND_MM = 1482000
START_MM = 0
LENGTH_MM = 5000
MSEC_PER_PING = 0   # Set value to limit ping rate. Normally set to 0 for best ping rate.
PULSE_LEN_PERCENT = 0.002
FILTER_DURATION_PERCENT = 0.0015
GAIN_INDEX = -1
NUM_RESULTS = 600


RESET_TIMEOUT = 10 # Timeout (s) to reset node, if no message has been produced.
# END MACROS.

class SonarNode(object):
    def __init__(self):
        """Initialization of the node.
        """
        # Start the node.
        rospy.init_node('omniscan')

        # # Get parameters.
        sensor_number = str(rospy.get_param('~sensor_number', SENSOR_NUMBER)) # SENSOR_NUMBER

        port_ip_address = str(rospy.get_param('~port_ip_address', PORT_IP_ADDRESS))
        port_port = rospy.get_param('~port_port', PORT_PORT)
        starboard_ip_address = rospy.get_param('~starboard_ip_address', STARBOARD_IP_ADDRESS)
        starboard_port = rospy.get_param('~starboard_port', STARBOARD_PORT)

        speed_of_sound_mm = rospy.get_param('~speed_of_sound_mm', SPEED_OF_SOUND_MM)

        start_mm = rospy.get_param('~start_mm', START_MM)
        length_mm = rospy.get_param('~length_mm', LENGTH_MM)
        self.msec_per_ping = rospy.get_param('~msec_per_ping', MSEC_PER_PING) # Frequency to read sensor.
        pulse_len_percent = rospy.get_param('~pulse_len_percent', PULSE_LEN_PERCENT)
        filter_duration_percent = rospy.get_param('~filter_duration_percent', FILTER_DURATION_PERCENT)
        gain_index = rospy.get_param('~gain_index', GAIN_INDEX)
        num_results = rospy.get_param('~num_results', NUM_RESULTS)

        # import_templates = "from omniscan_sidescan.omniscan${sensor_number}_driver import Omniscan${sensor_number} as OmniscanDriver\n"\
        #                     "from omniscan_sidescan.cfg import Omniscan${sensor_number}Config as OmniscanConfig"

        # template_object = Template(import_templates)
        # import_string = template_object.substitute(sensor_number=sensor_number)
        # exec(import_string, None, globals())

        self.sensor_name = SENSOR_NAME + sensor_number
        # Instantiating publishers.
        self.range_raw_pub = rospy.Publisher(
            TOPIC_SEPARATOR.join([self.sensor_name, SONAR_RAW_TOPIC_NAME]),
            SideScanSonarRaw, queue_size=10)
        self.range_pub = rospy.Publisher(
            TOPIC_SEPARATOR.join([self.sensor_name, SONAR_TOPIC_NAME]),
            SideScanSonar, queue_size=10)

        # self.sensor = OmniscanDriver()

        self.omniscan450_port = Omniscan450Driver(ip_address=port_ip_address, port=port_port,
                                            speed_of_sound_mm=speed_of_sound_mm, start_mm=start_mm, length_mm=length_mm,
                                            msec_per_ping=self.msec_per_ping, pulse_len_percent=pulse_len_percent,
                                            filter_duration_percent=filter_duration_percent,
                                            gain_index=gain_index, num_results=num_results)

        self.omniscan450_starboard = Omniscan450Driver(ip_address=starboard_ip_address, port=starboard_port,
                                            speed_of_sound_mm=speed_of_sound_mm, start_mm=start_mm, length_mm=length_mm,
                                            msec_per_ping=self.msec_per_ping, pulse_len_percent=pulse_len_percent,
                                            filter_duration_percent=filter_duration_percent,
                                            gain_index=gain_index, num_results=num_results)
        


        # Initialize parameter server
        self.parameter_server = Server(Omniscan450Config, self.parameters_callback)

        self.first_exception_time = None

        time.sleep(1)

    
    def parameters_callback(self, config, level):
        """Set parameters of the node for dynamic_reconfigure.
        """
        self.omniscan450_port.set_parameters(config)
        config = self.omniscan450_starboard.set_parameters(config)

        return config


    def spin(self):
        """Code that publishes the data from the sonar sensor.
        """
        node = rospy.Rate(self.msec_per_ping)
    
        # Loop
        while not rospy.is_shutdown():
            # Creation of the message
            sonar_msg = SideScanSonar()
            sonar_raw_msg = SideScanSonarRaw()

            current_time = rospy.get_rostime()

            try:
                # Read sensor
                port_data, port_scaled_results = self.omniscan450_port.get_data()
                starboard_data, starboard_scaled_results = self.omniscan450_starboard.get_data()

                # Populating the messages with the data.
                sonar_raw_msg.header.stamp = current_time
                sonar_raw_msg.header.frame_id = "sonar"

                sonar_raw_msg.port_ping_number = port_data.ping_number
                sonar_raw_msg.starboard_ping_number = starboard_data.ping_number
                sonar_raw_msg.start_mm = port_data.start_mm
                sonar_raw_msg.length_mm = port_data.length_mm
                sonar_raw_msg.port_timestamp_ms = port_data.timestamp_ms
                sonar_raw_msg.starboard_timestamp_ms = starboard_data.timestamp_ms
                sonar_raw_msg.port_ping_hz = port_data.ping_hz
                sonar_raw_msg.starboard_ping_hz = starboard_data.ping_hz
                sonar_raw_msg.port_gain_index = port_data.gain_index
                sonar_raw_msg.starboard_gain_index = starboard_data.gain_index
                sonar_raw_msg.num_results = port_data.num_results
                sonar_raw_msg.sos_dmps = port_data.sos_dmps
                sonar_raw_msg.channel_number = port_data.channel_number
                sonar_raw_msg.port_pulse_duration_sec = port_data.pulse_duration_sec
                sonar_raw_msg.starboard_pulse_duration_sec = starboard_data.pulse_duration_sec
                sonar_raw_msg.port_analog_gain = port_data.analog_gain
                sonar_raw_msg.starboard_analog_gain = starboard_data.analog_gain
                sonar_raw_msg.port_max_pwr_db = port_data.max_pwr_db
                sonar_raw_msg.starboard_max_pwr_db = starboard_data.max_pwr_db
                sonar_raw_msg.port_min_pwr_db = port_data.min_pwr_db
                sonar_raw_msg.starboard_min_pwr_db = starboard_data.min_pwr_db
                sonar_raw_msg.port_transducer_heading_deg = port_data.transducer_heading_deg
                sonar_raw_msg.starboard_transducer_heading_deg = starboard_data.transducer_heading_deg
                sonar_raw_msg.vehicle_heading_deg = port_data.vehicle_heading_deg

                sonar_raw_msg.port_pwr_data = port_data.pwr_results
                sonar_raw_msg.starboard_pwr_data = starboard_data.pwr_results

                # # For logging purpose.
                # rospy.logdebug(sonar_raw_msg)

                sonar_msg.header.stamp = current_time
                sonar_msg.header.frame_id = "sonar"

                sonar_msg.port_ping_number = port_data.ping_number
                sonar_msg.starboard_ping_number = starboard_data.ping_number
                sonar_msg.start_mm = port_data.start_mm
                sonar_msg.length_mm = port_data.length_mm
                sonar_msg.port_timestamp_ms = port_data.timestamp_ms
                sonar_msg.starboard_timestamp_ms = starboard_data.timestamp_ms
                sonar_msg.port_ping_hz = port_data.ping_hz
                sonar_msg.starboard_ping_hz = starboard_data.ping_hz
                sonar_msg.port_gain_index = port_data.gain_index
                sonar_msg.starboard_gain_index = starboard_data.gain_index
                sonar_msg.num_results = port_data.num_results
                sonar_msg.sos_dmps = port_data.sos_dmps
                sonar_msg.channel_number = port_data.channel_number
                sonar_msg.port_pulse_duration_sec = port_data.pulse_duration_sec
                sonar_msg.starboard_pulse_duration_sec = starboard_data.pulse_duration_sec
                sonar_msg.port_analog_gain = port_data.analog_gain
                sonar_msg.starboard_analog_gain = starboard_data.analog_gain
                sonar_msg.port_max_pwr_db = port_data.max_pwr_db
                sonar_msg.starboard_max_pwr_db = starboard_data.max_pwr_db
                sonar_msg.port_min_pwr_db = port_data.min_pwr_db
                sonar_msg.starboard_min_pwr_db = starboard_data.min_pwr_db
                sonar_msg.port_transducer_heading_deg = port_data.transducer_heading_deg
                sonar_msg.starboard_transducer_heading_deg = starboard_data.transducer_heading_deg
                sonar_msg.vehicle_heading_deg = port_data.vehicle_heading_deg

                sonar_msg.port_scaled_data = port_scaled_results
                sonar_msg.starboard_scaled_data = starboard_scaled_results

                # Publish ROS messages.
                self.range_raw_pub.publish(sonar_raw_msg)
                self.range_pub.publish(sonar_msg)
                if self.first_exception_time:
                    self.first_exception_time = None

            except:
                # Error when reading data.
                if self.first_exception_time is None:
                    rospy.logerr("Exception when reading Omniscan450 sonar data.")
                    self.first_exception_time = current_time
                # else:
                #     if current_time - self.first_exception_time > rospy.Duration(RESET_TIMEOUT):
                #         rospy.signal_shutdown("Omniscan sonar sensor not ready")

            # Keep the frequency.
            node.sleep()

        # Terminate sensor.
        self.omniscan450_port.close_connection()
        self.omniscan450_starboard.close_connection()


if __name__ == '__main__':
    sonar_node = SonarNode()
    sonar_node.spin()