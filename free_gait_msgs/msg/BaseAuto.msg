# Command to let controller add auto generated base motion
# for stability and reachability of footsteps.

# Base height in control frame.
# If 0, current height of the robot is used.
float64 height

# If the duration of the base motion should
# NOT be adapted to the duration of the step's leg.
# If no leg motion is defined in the step,
# the average velocities are used.
# Default is false.
bool ignore_timing_of_leg_motion

# Average linear velocity [m/s] and angular
# velocity [rad/s] of the base motion.
# If 0, default is used.
float64 average_linear_velocity
float64 average_angular_velocity

# Margin for the support polygon [m].
# If 0, default is used.
float64 support_margin

# Center of Mass Tolerance
# If 0, default is used.
float64 center_of_mass_tolerance

# Leg length tolerance
# If 0, default is used.
float64 leg_length_tolerance

# Scaling factor to compute minimum limb length
# If 0, default is used.
float64 min_limb_length_scale

# Scaling factor to compute maximum limb length at closing of contact
# If 0, default is used.
float64 max_limb_length_at_closing_contact_scale

# Scaling factor to compute maximum limb length at opening of contact
# If 0, default is used.
float64 max_limb_length_at_opening_contact_scale