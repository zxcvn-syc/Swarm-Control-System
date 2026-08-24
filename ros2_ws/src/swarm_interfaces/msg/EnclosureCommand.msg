uint32 drone_id
float64 target_x
float64 target_y
float64 target_z
float32 enclosure_radius

# Enclosure layer identifiers (three-layer containment)
uint8 LAYER_MONITOR=0   # UAV monitor layer (outer ring)
uint8 LAYER_BLOCK=1     # UGV block layer (inner ring)
uint8 LAYER_COMMAND=2   # human command layer (reserved)
uint8 layer
