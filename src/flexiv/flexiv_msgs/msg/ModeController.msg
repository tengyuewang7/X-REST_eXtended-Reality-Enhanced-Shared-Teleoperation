int8 MODE_UNKNOWN                          = -1
int8 MODE_IDLE                             = 0
int8 MODE_RT_JOINT_TORQUE                  = 1
int8 MODE_RT_JOINT_POSITION                = 2
int8 MODE_NRT_JOINT_POSITION               = 3
int8 MODE_NRT_PLAN_EXECUTION               = 4
int8 MODE_NRT_PRIMITIVE_EXECUTION          = 5
int8 MODE_RT_CARTESIAN_MOTION_FORCE_BASE   = 6
int8 MODE_RT_CARTESIAN_MOTION_FORCE_TCP    = 7
int8 MODE_NRT_CARTESIAN_MOTION_FORCE_BASE  = 8
int8 MODE_NRT_CARTESIAN_MOTION_FORCE_TCP   = 9

int8 mode
float64[7] positions
float64[7] velocities
float64[7] accelerations
float64[7] torques 
float64[7] max_vel 
float64[7] max_acc  
int8 plan_index
bool gravity_comp
bool soft_limits
std_msgs/String primitive_name 
geometry_msgs/Pose tcp_pose
