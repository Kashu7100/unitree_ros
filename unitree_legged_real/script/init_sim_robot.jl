"""
This script initialize the Julia controller for A1 simulation

In this script we start a robot interface to send/receive data from simulation
Then we convert the current Julia REPL into a ros node 

Notice! must use_sim_time for roslauch gazebo environment
"""
# start A1Robot nterface
include("julia_robot_interface.jl")

""" init the robot """
# robot = A1Robot.RobotInterface()  # do not connect to real robot in sim
# A1Robot.fbk_state = A1Robot.LowState()
A1Robot.init_fbk()
# A1Robot.InitSend(robot)

""" ROS related """
# convert the current Julia REPL as ros node
using RobotOS
using StaticArrays

@rosimport sensor_msgs.msg: Imu, Joy
@rosimport nav_msgs.msg: Odometry
@rosimport geometry_msgs.msg: Pose, Twist, QuaternionStamped, WrenchStamped
@rosimport unitree_legged_msgs.msg: MotorCmd, MotorState
rostypegen()
using .sensor_msgs.msg
using .nav_msgs.msg
using .geometry_msgs.msg
using .unitree_legged_msgs.msg

init_node("rosjl_sim",disable_signals="True")
# initialize necessary publisher, subscriber and callbacks
function joy_stick_callback(msg::sensor_msgs.msg.Joy, out_axes::Array{Float32,1}, out_button::Array{Int16,1})
    # pt_msg = Point(msg.point.x, msg.point.y, 0.0)
    for i=1:8
        out_axes[i] = msg.axes[i]
    end
    for i=1:11
        out_button[i] = msg.buttons[i]
    end
    # println(msg.axes)
end
const joy_stick_sub = Subscriber{sensor_msgs.msg.Joy}("joy", joy_stick_callback, (A1Robot.joy_data_axes,A1Robot.joy_data_buttons,), queue_size=1)

""" receive sim state """
# function xsens_filter_callback(msg::sensor_msgs.msg.QuaternionStamped, out_data::geometry_msgs.msg.Pose)
#     # pt_msg = Point(msg.point.x, msg.point.y, 0.0)
#     out_data.orientation = msg.quaternion
# end
# xsens_filter_sub = Subscriber{sensor_msgs.msg.QuaternionStamped}("filter/quaternion", xsens_filter_callback, (body_pose,), queue_size=1)
function sim_pose_callback(msg::nav_msgs.msg.Odometry, position::Array{Float64,1}, velocity::Array{Float64,1}, orientation::Array{Float64,1})
    # pt_msg = Point(msg.point.x, msg.point.y, 0.0)
    position[1] = msg.pose.pose.position.x
    position[2] = msg.pose.pose.position.y
    position[3] = msg.pose.pose.position.z
    orientation[2] = msg.pose.pose.orientation.x
    orientation[3] = msg.pose.pose.orientation.y
    orientation[4] = msg.pose.pose.orientation.z
    orientation[1] = msg.pose.pose.orientation.w
    velocity[1] = msg.twist.twist.linear.x
    velocity[2] = msg.twist.twist.linear.y
    velocity[3] = msg.twist.twist.linear.z
end
const sim_pose_sub = Subscriber{nav_msgs.msg.Odometry}("body_pose_ground_truth", sim_pose_callback, (A1Robot.body_position,A1Robot.body_velocity,A1Robot.body_orientation,), queue_size=1)


# function xsens_imu_callback(msg::sensor_msgs.msg.Imu, out_data::sensor_msgs.msg.Imu)
#     # pt_msg = Point(msg.point.x, msg.point.y, 0.0)
#     out_data.angular_velocity = msg.angular_velocity
# end
# xsens_imu_sub = Subscriber{sensor_msgs.msg.Imu}("imu/data", xsens_imu_callback, (imu_data,), queue_size=1)
function sim_imu_callback(msg::sensor_msgs.msg.Imu, state::A1Robot.LowState)
    # pt_msg = Point(msg.point.x, msg.point.y, 0.0)
    v = SVector{3}([msg.angular_velocity.x,
                  msg.angular_velocity.y,
                  msg.angular_velocity.z])
    state.imu.gyroscope = v
    # out_data2.orientation = msg.orientation
end
const sim_imu_sub = Subscriber{sensor_msgs.msg.Imu}("trunk_imu", sim_imu_callback, (A1Robot.fbk_state,), queue_size=1)

# simulated foot contact
function sim_foot_force_FR_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    # println("got")
    state.footForce[1] = convert(Int16, floor(msg.wrench.force.z*10))
end
const sim_footforceFR_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/FR_foot_contact/the_force", 
    sim_foot_force_FR_callback, (A1Robot.fbk_state,), queue_size=1)
function sim_foot_force_FL_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    state.footForce[2] = convert(Int16, floor(msg.wrench.force.z*10))
end
const sim_footforceFL_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/FL_foot_contact/the_force", 
    sim_foot_force_FL_callback, (A1Robot.fbk_state,), queue_size=1)
function sim_foot_force_RR_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    state.footForce[3] = convert(Int16, floor(msg.wrench.force.z*10))
end
const sim_footforceRR_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/RR_foot_contact/the_force", 
    sim_foot_force_RR_callback, (A1Robot.fbk_state,), queue_size=1)
function sim_foot_force_RL_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    state.footForce[4] = convert(Int16, floor(msg.wrench.force.z*10))
end
const sim_footforceRL_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/RL_foot_contact/the_force", 
    sim_foot_force_RL_callback, (A1Robot.fbk_state,), queue_size=1)

# simulated robot joint state 
function sim_joint_FR_hip_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.MotorState)
    state.mode = UInt8(msg.mode)
    state.q = msg.q
    # println(state.motorState[1].q)
    state.dq = Float32(msg.dq)
    state.ddq = Float32(msg.ddq)
    state.tauEst = Float32(msg.tauEst)
end
const sim_joint_FR_hip_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FR_hip_controller/state", 
                                sim_joint_FR_hip_callback, (A1Robot.fbk_state.motorState[1],), queue_size=1)

function sim_joint_FR_thigh_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[2].mode = UInt8(msg.mode)
    state.motorState[2].q = Float32(msg.q)
    # println(state.motorState[2].q)
    state.motorState[2].dq = Float32(msg.dq)
    state.motorState[2].ddq = Float32(msg.ddq)
    state.motorState[2].tauEst = Float32(msg.tauEst)
end
const sim_joint_FR_thigh_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FR_thigh_controller/state", 
                                sim_joint_FR_thigh_callback, (A1Robot.fbk_state,), queue_size=1)

function sim_joint_FR_calf_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[3].mode = UInt8(msg.mode)
    state.motorState[3].q = Float32(msg.q)
    # println(state.motorState[3].q)
    state.motorState[3].dq = Float32(msg.dq)
    state.motorState[3].ddq = Float32(msg.ddq)
    state.motorState[3].tauEst = Float32(msg.tauEst)
end
const sim_joint_FR_calf_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FR_calf_controller/state", 
                                sim_joint_FR_calf_callback, (A1Robot.fbk_state,), queue_size=1)         


function sim_joint_FL_hip_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[4].mode = UInt8(msg.mode)
    state.motorState[4].q = Float32(msg.q)
    state.motorState[4].dq = Float32(msg.dq)
    state.motorState[4].ddq = Float32(msg.ddq)
    state.motorState[4].tauEst = Float32(msg.tauEst)
end
const sim_joint_FL_hip_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FL_hip_controller/state", 
                                sim_joint_FL_hip_callback, (A1Robot.fbk_state,), queue_size=1)

function sim_joint_FL_thigh_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[5].mode = UInt8(msg.mode)
    state.motorState[5].q = Float32(msg.q)
    state.motorState[5].dq = Float32(msg.dq)
    state.motorState[5].ddq = Float32(msg.ddq)
    state.motorState[5].tauEst = Float32(msg.tauEst)
end
const sim_joint_FL_thigh_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FL_thigh_controller/state", 
                                sim_joint_FL_thigh_callback, (A1Robot.fbk_state,), queue_size=1)

function sim_joint_FL_calf_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[6].mode = UInt8(msg.mode)
    state.motorState[6].q = Float32(msg.q)
    state.motorState[6].dq = Float32(msg.dq)
    state.motorState[6].ddq = Float32(msg.ddq)
    state.motorState[6].tauEst = Float32(msg.tauEst)
end
const sim_joint_FL_calf_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FL_calf_controller/state", 
                                sim_joint_FL_calf_callback, (A1Robot.fbk_state,), queue_size=1)                                  


function sim_joint_RR_hip_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[7].mode = UInt8(msg.mode)
    state.motorState[7].q = Float32(msg.q)
    state.motorState[7].dq = Float32(msg.dq)
    state.motorState[7].ddq = Float32(msg.ddq)
    state.motorState[7].tauEst = Float32(msg.tauEst)
end
const sim_joint_RR_hip_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RR_hip_controller/state", 
                                sim_joint_RR_hip_callback, (A1Robot.fbk_state,), queue_size=1)

function sim_joint_RR_thigh_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[8].mode = UInt8(msg.mode)
    state.motorState[8].q = Float32(msg.q)
    state.motorState[8].dq = Float32(msg.dq)
    state.motorState[8].ddq = Float32(msg.ddq)
    state.motorState[8].tauEst = Float32(msg.tauEst)
end
const sim_joint_RR_thigh_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RR_thigh_controller/state", 
                                sim_joint_RR_thigh_callback, (A1Robot.fbk_state,), queue_size=1)

function sim_joint_RR_calf_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[9].mode = UInt8(msg.mode)
    state.motorState[9].q = Float32(msg.q)
    state.motorState[9].dq = Float32(msg.dq)
    state.motorState[9].ddq = Float32(msg.ddq)
    state.motorState[9].tauEst = Float32(msg.tauEst)
end
const sim_joint_RR_calf_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RR_calf_controller/state", 
                                sim_joint_RR_calf_callback, (A1Robot.fbk_state,), queue_size=1)  


function sim_joint_RL_hip_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[10].mode = UInt8(msg.mode)
    state.motorState[10].q = Float32(msg.q)
    state.motorState[10].dq = Float32(msg.dq)
    state.motorState[10].ddq = Float32(msg.ddq)
    state.motorState[10].tauEst = Float32(msg.tauEst)
end
const sim_joint_RL_hip_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RL_hip_controller/state", 
                                sim_joint_RL_hip_callback, (A1Robot.fbk_state,), queue_size=1)

function sim_joint_RL_thigh_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[11].mode = UInt8(msg.mode)
    state.motorState[11].q = Float32(msg.q)
    state.motorState[11].dq = Float32(msg.dq)
    state.motorState[11].ddq = Float32(msg.ddq)
    state.motorState[11].tauEst = Float32(msg.tauEst)
end
const sim_joint_RL_thigh_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RL_thigh_controller/state", 
                                sim_joint_RL_thigh_callback, (A1Robot.fbk_state,), queue_size=1)

function sim_joint_RL_calf_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[12].mode = UInt8(msg.mode)
    state.motorState[12].q = Float32(msg.q)
    state.motorState[12].dq = Float32(msg.dq)
    state.motorState[12].ddq = Float32(msg.ddq)
    state.motorState[12].tauEst = Float32(msg.tauEst)
end
const sim_joint_RL_calf_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RL_calf_controller/state", 
                                sim_joint_RL_calf_callback, (A1Robot.fbk_state,), queue_size=1)  

""" send sim data """
simcmd_list = [unitree_legged_msgs.msg.MotorCmd() for i=1:12]
const sim_pub = []
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FR_hip_controller/command", queue_size=1)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FR_thigh_controller/command", queue_size=1)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FR_calf_controller/command", queue_size=1)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FL_hip_controller/command", queue_size=1)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FL_thigh_controller/command", queue_size=1)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FL_calf_controller/command", queue_size=1)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RR_hip_controller/command", queue_size=1)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RR_thigh_controller/command", queue_size=1)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RR_calf_controller/command", queue_size=1)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RL_hip_controller/command", queue_size=1)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RL_thigh_controller/command", queue_size=1)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RL_calf_controller/command", queue_size=1)
push!(sim_pub,pub)

function sim_setCmdMotorTau(id::Int, tau::Float32)
    simcmd_list[id+1].mode = 0x0A
    simcmd_list[id+1].tau = tau
end

function sim_SendCommand()
    #ros send
    for i=1:12
        publish(sim_pub[i], simcmd_list[i])
    end
end


""" the rate of the control loop, very important parameter  """
ctrl_hz = 200.0
ctrl_dt = 1.0/ctrl_hz
loop_rate = Rate(ctrl_hz)

