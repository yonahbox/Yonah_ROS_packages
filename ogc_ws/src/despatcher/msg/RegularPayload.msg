# Message content of a regular payload

# Header includes the timestamp and msg prefix type (under frame ID)
std_msgs/Header header

# Prefix
bool is_aircraft
uint8 vehicle_no

# Data
uint8 airspeed
int16 alt
bool armed
uint8 mode
uint16 groundspeed
bool fuel
bool batt 
float32 lat
float32 lon
float32 throttle
bool vtol
uint8 wp
uint16 wp_total
uint8 vibe