# Epoch 1970-01-01 UTC time, ignoring leap seconds.
uint32 utc_seconds
uint32 utc_nanoseconds

# One bit per status info, 1= active, 0= inactive
uint32 status

float64 latitude
float64 longitude
float32 ellipsoid_height

# Using NED coordinate system
float32 roll
float32 pitch
float32 heading
float32 heave

float32 roll_rate
float32 pitch_rate
float32 yaw_rate

float32 north_velocity
float32 east_velocity
float32 down_velocity

float32 latitude_error
float32 longitude_error
float32 height_error
float32 roll_error
float32 pitch_error
float32 heading_error
float32 heave_error

float32 north_acceleration
float32 east_acceleration
float32 down_acceleration

uint32 delayed_heave_utc_seconds
uint32 delayed_heave_utc_nanoseconds
float32 delayed_heave
