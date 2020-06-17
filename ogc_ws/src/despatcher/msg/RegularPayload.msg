# Message content of a regular payload

# Header includes the timestamp and msg prefix type (under frame ID)
std_msgs/Header header

# Prefix
bool is_aircraft
uint8 vehicle_no

# Data
uint8 airspeed
uint16 alt
bool armed
uint16 mode
uint16 groundspeed
uint8 windspeed
bool fuel
bool batt 
float32 lat
float32 lon
float32 throttle
bool vtol
uint8 wp
uint8 vibe