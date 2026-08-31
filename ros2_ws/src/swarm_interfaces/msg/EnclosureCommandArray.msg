std_msgs/Header header
# Monotonically increasing source sequence. Consumers can reject stale or
# replayed enclosure plans even when their contents happen to be identical.
uint64 sequence
EnclosureCommand[] commands
uint32 num_drones
