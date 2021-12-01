# This message describes a contact schedule
#
# A contact schedule is defined by the start and end contact
# configuration for all limbs.

std_msgs/Header header

# Array of ContactState information (placement) for each limb
# at the start and the end of the contact schedule

# Length of the following three vectors needs to match. The limb
# order follows the order of `contact_name`
string[] contact_name
geometry_msgs/Pose[] contact_configuration_start
geometry_msgs/Pose[] contact_configuration_end

uint8 STAND = 0
uint8 WALK  = 1
uint8 JUMP  = 2
uint8 TROT  = 3
uint8 PACE  = 4
uint8 BOUND = 5

# Specifies the type of gait. Note, not all gait types might be
# available for a given platform
uint8 gait_type

# t_ds = Duration of double support in seconds. Needs to be a
# multiple of the MPC's dt
float64 t_ds

# t_ds = Duration of single support in seconds. Needs to be a
# multiple of the MPC's dt
float64 t_ss

# t_ds = Duration of ??? in seconds. Needs to be a
# multiple of the MPC's dt
float64 t_uds

# t_ds = Duration of ??? in seconds. Needs to be a
# multiple of the MPC's dt
float64 t_uss

# Step height at the apex of the swing trajectory in m
float64 step_height
