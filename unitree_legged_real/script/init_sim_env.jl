include("julia_robot_interface.jl")
include("utils.jl")

using Rotations
using Printf
using OSQP
using SparseArrays

using RobotOS
@rosimport sensor_msgs.msg: Imu, Joy
@rosimport nav_msgs.msg: Odometry
@rosimport geometry_msgs.msg: Pose, Twist, QuaternionStamped, WrenchStamped
@rosimport unitree_legged_msgs.msg: MotorCmd, MotorState, LowState
rostypegen()
using .sensor_msgs.msg
using .nav_msgs.msg
using .geometry_msgs.msg
using .unitree_legged_msgs.msg