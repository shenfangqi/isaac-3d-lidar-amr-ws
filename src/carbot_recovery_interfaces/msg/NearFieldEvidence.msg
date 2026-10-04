# Fresh, partial near-field collision evidence from a depth or range sensor.
#
# header.stamp is the original sensor capture time.  header.frame_id and
# observed_space.header.frame_id must name the same fixed navigation frame.
# Receivers must transform the source sensor at header.stamp and reject stale,
# future-dated, reordered, or frame-inconsistent messages.
std_msgs/Header header

# Standard OccupancyGrid semantics with a stricter safety interpretation:
#   0   = directly observed free at collision-relevant height
#   100 = observed occupied
#   -1  = unknown, invalid depth, occluded, or outside the sensor field of view
# Values other than exactly 0 never authorize recovery motion.
nav_msgs/OccupancyGrid observed_space

# Monotonically increasing within one producer process.  A restart may reset
# this value and must also change producer_instance_id.
uint64 update_sequence
string producer_instance_id

# Stable diagnostic identity and the optical/range frame that generated the
# evidence.  Runtime authorization also validates the configured publisher
# topology; these strings are not authority by themselves.
string sensor_id
string sensor_frame

# Actual filtering bounds used by the producer for this message.  They are
# evidence metadata and must be checked against the accepted hardware profile.
float32 minimum_valid_range_m
float32 maximum_valid_range_m
float32 minimum_collision_height_m
float32 maximum_collision_height_m
