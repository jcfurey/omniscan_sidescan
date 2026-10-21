# One sonar channel. Header time is host receive time, not synchronized device time.
# Device fields retain their explicitly named wire units.
std_msgs/Header header
uint64 configuration_generation
uint32 ping_number
uint32 timestamp_ms              # Milliseconds since this device's startup
uint32 start_mm
uint32 length_mm
uint32 ping_hz                   # Acoustic carrier frequency, not profile rate
uint16 gain_index
uint16 num_results
uint16 sos_dmps                  # Speed of sound in decimeters/second
uint8 channel_number
float32 pulse_duration_sec
float32 analog_gain
float32 max_pwr_db
float32 min_pwr_db
float32 transducer_heading_deg
float32 vehicle_heading_deg
uint16[<=1200] pwr_data
float32[<=1200] scaled_data       # dB, interpolated using this profile's bounds
