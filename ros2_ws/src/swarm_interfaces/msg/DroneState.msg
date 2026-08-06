# Per-drone state published by the simulator / state estimator.
uint32 drone_id
float64 x
float64 y
float64 z
float64 vx
float64 vy
float64 vz
bool available
# Platform classification: 0 = aerial UAV, 1 = ground UGV.
uint8 platform_type
