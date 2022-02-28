uint64 timestamp             # time since system start (microseconds)

float32[3] pos_est_filtered  # Filtered target position used for vehicle's setpoint generation
float32[3] vel_est_filtered  # Filtered Target velocity used for vehicle's setpoint generation
bool emergency_ascent
float32 gimbal_pitch
