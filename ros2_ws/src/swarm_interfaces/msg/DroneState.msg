# Per-drone state published by the simulator / state estimator.
uint32 drone_id
float64 x
float64 y
float64 z
float64 vx
float64 vy
float64 vz
bool available

# Platform type identifiers
uint8 PLATFORM_DRONE=0
uint8 PLATFORM_CAR=1
uint8 platform_type

