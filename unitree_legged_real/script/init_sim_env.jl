include("julia_robot_interface.jl")
include("utils.jl")

using Rotations
using Printf
using OSQP
using SparseArrays

using RobotOS
# @rosimport std_msgs.msg: Float64
@rosimport sensor_msgs.msg: Imu, Joy
@rosimport nav_msgs.msg: Odometry
@rosimport geometry_msgs.msg: Point, Quaternion, Pose, Twist, QuaternionStamped, WrenchStamped
@rosimport unitree_legged_msgs.msg: MotorCmd, MotorState, LowState
rostypegen()
# using .std_msgs.msg
using .sensor_msgs.msg
using .nav_msgs.msg
using .geometry_msgs.msg
using .unitree_legged_msgs.msg

#just call functions in A1Robot, do not instantiate robot interface

using Dates # this is for get time to control gait, but may need to use ros time later