# Encapsulation control data message
# For perception/planning -> encapsulation group integration
# Used by Chen Sirui's Voronoi-based enclosure calculation

# Target information for enclosure calculation
uint32 target_id
float64 x                     # Target X position
float64 y                     # Target Y position
float32 speed                # Target speed magnitude
uint8 motion_mode            # 0=unknown, 1=stationary, 2=slow, 3=fast
float32 confidence           # Target confidence

# Target bounding box for size reference
float32 box_x1
float32 box_y1
float32 box_x2
float32 box_y2

# Future trajectory prediction (for enclosure prediction)
float32[5] pred_x           # Predicted X positions (steps 1-5)
float32[5] pred_y           # Predicted Y positions (steps 1-5)

# Track history for trajectory analysis
float32[10] history_x       # Recent X positions
float32[10] history_y       # Recent Y positions
