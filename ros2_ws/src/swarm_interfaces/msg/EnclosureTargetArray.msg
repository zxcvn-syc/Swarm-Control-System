# Array of targets for enclosure control
# Published periodically for Voronoi-based enclosure calculation

std_msgs/Header header
uint32 frame_idx
EnclosureTarget[] targets

# Field positions for multi-UAV formation reference
float32[8] drone_x          # Drone X positions (up to 8 drones)
float32[8] drone_y          # Drone Y positions
uint8 num_drones            # Number of active drones

# Enclosure zone parameters
float32 enclosure_radius    # Current enclosure radius
float32 min_enclosure_dist  # Minimum enclosure distance
