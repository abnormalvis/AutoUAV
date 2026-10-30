std_msgs/Header header

# Lifecycle state of the bridge.
uint8 STATE_INIT = 0
uint8 STATE_OK = 1
uint8 STATE_STALE = 2

uint8 state

# True only while the bridge is forwarding fresh, accepted VIO samples.
bool valid

string input_topic
string world_frame
string body_frame

# Measured rates, not configured rates.
float64 input_rate_hz
float64 output_rate_hz

# now() - header.stamp of the most recent accepted sample, in seconds.
float64 input_latency

uint64 received
uint64 published
uint64 rejected_invalid
uint64 rejected_stale
uint64 rejected_jump
uint64 timestamp_resets
uint64 origin_resets
