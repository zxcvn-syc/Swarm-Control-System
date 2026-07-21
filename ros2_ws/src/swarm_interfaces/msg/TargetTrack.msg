# Target tracking message for swarm coordination
# Optimized for scheduling module integration (Perception -> Scheduling)

uint32 target_id              # Unique track ID
float64 x                     # X coordinate (pixel or world)
float64 y                     # Y coordinate (pixel or world)
float64 vx                    # Velocity X (pixels/sec)
float64 vy                    # Velocity Y (pixels/sec)

# Enhanced fields for scheduling module
float32 confidence             # Detection confidence (0.0-1.0)
uint8 cls                     # Object class (COCO style)
bool is_confirmed             # Track confirmation status
float32 speed                 # Current speed magnitude
uint8 motion_mode             # 0=unknown, 1=stationary, 2=slow, 3=fast

# Optional predicted future positions (for trajectory planning)
# Array indices: [0-4] for steps 1-5 ahead
float32[5] pred_x            # Predicted X positions (steps 1-5 ahead)
float32[5] pred_y            # Predicted Y positions (steps 1-5 ahead)
float32[5] pred_conf          # Confidence for each prediction
