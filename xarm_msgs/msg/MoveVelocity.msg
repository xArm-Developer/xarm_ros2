# This format is suitable for the following topic
#   - vc_set_joint_velocity
#   - vc_set_cartesian_velocity

# vc_set_joint_velocity/vc_set_cartesian_velocity
float32[] speeds

# vc_set_joint_velocity
bool is_sync        true

# vc_set_cartesian_velocity
bool is_tool_coord  false

# the maximum duration of the spedd, over this time will automatically set the speed to 0
#   duration > 0: seconds, indicates the maximum number of seconds that this speed can be maintained
#   duration == 0: always effective, will not stop automativally
#   duration < 0: default value, only used to be compatible with the old protocol, equivalent to 0
# avaiable for firmware_version >= 1.8.0
float32 duration -1.0
