"""
This script initialize the Julia controller for A1 simulation

In this script we start a robot interface to send/receive data from simulation
Then we convert the current Julia REPL into a ros node 
"""
# start A1Robot nterface
include("julia_robot_interface.jl")

""" init the robot """
# robot = A1Robot.RobotInterface()  # do not connect to real robot in sim
fbk_state = A1Robot.LowState()
for i=1:4
    fbk_state.footForce[i] = 0
end
for i=1:12
    fbk_state.motorState[i].q = 0.01
    fbk_state.motorState[i].dq = 0.0
end
# A1Robot.InitSend(robot)

""" ROS related """
# convert the current Julia REPL as ros node
using RobotOS
@rosimport sensor_msgs.msg: Imu, Joy
@rosimport geometry_msgs.msg: Pose, QuaternionStamped, WrenchStamped
@rosimport unitree_legged_msgs.msg: MotorCmd, MotorState
rostypegen()
using .sensor_msgs.msg
using .geometry_msgs.msg
using .unitree_legged_msgs.msg
init_node("rosjl_sim",disable_signals="True")
# initialize necessary publisher, subscriber and callbacks
joy_data = sensor_msgs.msg.Joy()
joy_data.buttons = zeros(11)
joy_data.axes = zeros(Float32,8)
# function joy_stick_callback(msg::sensor_msgs.msg.Joy, out_data::sensor_msgs.msg.Joy)
#     # pt_msg = Point(msg.point.x, msg.point.y, 0.0)
#     out_data.axes = msg.axes
#     out_data.buttons = msg.buttons
#     # println(msg.axes)
# end
# joy_stick_sub = Subscriber{sensor_msgs.msg.Joy}("joy", joy_stick_callback, (joy_data,), queue_size=1)

""" receive sim state """
body_pose = geometry_msgs.msg.Pose()
body_pose.orientation.w = 1
# function xsens_filter_callback(msg::sensor_msgs.msg.QuaternionStamped, out_data::geometry_msgs.msg.Pose)
#     # pt_msg = Point(msg.point.x, msg.point.y, 0.0)
#     out_data.orientation = msg.quaternion
# end
# xsens_filter_sub = Subscriber{sensor_msgs.msg.QuaternionStamped}("filter/quaternion", xsens_filter_callback, (body_pose,), queue_size=1)

imu_data = sensor_msgs.msg.Imu()
# function xsens_imu_callback(msg::sensor_msgs.msg.Imu, out_data::sensor_msgs.msg.Imu)
#     # pt_msg = Point(msg.point.x, msg.point.y, 0.0)
#     out_data.angular_velocity = msg.angular_velocity
# end
# xsens_imu_sub = Subscriber{sensor_msgs.msg.Imu}("imu/data", xsens_imu_callback, (imu_data,), queue_size=1)
function sim_imu_callback(msg::sensor_msgs.msg.Imu, out_data::sensor_msgs.msg.Imu,out_data2::geometry_msgs.msg.Pose)
    # pt_msg = Point(msg.point.x, msg.point.y, 0.0)
    out_data.angular_velocity = msg.angular_velocity
    out_data2.orientation = msg.orientation
end
sim_imu_sub = Subscriber{sensor_msgs.msg.Imu}("trunk_imu", sim_imu_callback, (imu_data,body_pose), queue_size=1)

# simulated foot contact
function sim_foot_force_FR_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    # println("got")
    state.footForce[1] = convert(Int16, floor(msg.wrench.force.z*10))
end
sim_footforceFR_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/FR_foot_contact/the_force", 
    sim_foot_force_FR_callback, (fbk_state,), queue_size=10)
function sim_foot_force_FL_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    state.footForce[2] = convert(Int16, floor(msg.wrench.force.z*10))
end
sim_footforceFL_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/FL_foot_contact/the_force", 
    sim_foot_force_FL_callback, (fbk_state,), queue_size=10)
function sim_foot_force_RR_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    state.footForce[3] = convert(Int16, floor(msg.wrench.force.z*10))
end
sim_footforceRR_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/RR_foot_contact/the_force", 
    sim_foot_force_RR_callback, (fbk_state,), queue_size=10)
function sim_foot_force_RL_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    state.footForce[4] = convert(Int16, floor(msg.wrench.force.z*10))
end
sim_footforceRL_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/RL_foot_contact/the_force", 
    sim_foot_force_RL_callback, (fbk_state,), queue_size=10)

# simulated robot joint state 
function sim_joint_FR_hip_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[1].mode = UInt8(msg.mode)
    state.motorState[1].q = Float32(msg.q)
    state.motorState[1].dq = Float32(msg.dq)
    state.motorState[1].ddq = Float32(msg.ddq)
end
sim_joint_FR_hip_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FR_hip_controller/state", 
                                sim_joint_FR_hip_callback, (fbk_state,), queue_size=10)

function sim_joint_FR_thigh_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[2].mode = UInt8(msg.mode)
    state.motorState[2].q = Float32(msg.q)
    state.motorState[2].dq = Float32(msg.dq)
    state.motorState[2].ddq = Float32(msg.ddq)
end
sim_joint_FR_thigh_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FR_thigh_controller/state", 
                                sim_joint_FR_thigh_callback, (fbk_state,), queue_size=10)

function sim_joint_FR_calf_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[3].mode = UInt8(msg.mode)
    state.motorState[3].q = Float32(msg.q)
    state.motorState[3].dq = Float32(msg.dq)
    state.motorState[3].ddq = Float32(msg.ddq)
end
sim_joint_FR_calf_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FR_calf_controller/state", 
                                sim_joint_FR_calf_callback, (fbk_state,), queue_size=10)         


function sim_joint_FL_hip_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[4].mode = UInt8(msg.mode)
    state.motorState[4].q = Float32(msg.q)
    state.motorState[4].dq = Float32(msg.dq)
    state.motorState[4].ddq = Float32(msg.ddq)
end
sim_joint_FL_hip_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FL_hip_controller/state", 
                                sim_joint_FL_hip_callback, (fbk_state,), queue_size=10)

function sim_joint_FL_thigh_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[5].mode = UInt8(msg.mode)
    state.motorState[5].q = Float32(msg.q)
    state.motorState[5].dq = Float32(msg.dq)
    state.motorState[5].ddq = Float32(msg.ddq)
end
sim_joint_FL_thigh_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FL_thigh_controller/state", 
                                sim_joint_FL_thigh_callback, (fbk_state,), queue_size=10)

function sim_joint_FL_calf_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[6].mode = UInt8(msg.mode)
    state.motorState[6].q = Float32(msg.q)
    state.motorState[6].dq = Float32(msg.dq)
    state.motorState[6].ddq = Float32(msg.ddq)
end
sim_joint_FL_calf_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/FL_calf_controller/state", 
                                sim_joint_FL_calf_callback, (fbk_state,), queue_size=10)                                  


function sim_joint_RR_hip_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[7].mode = UInt8(msg.mode)
    state.motorState[7].q = Float32(msg.q)
    state.motorState[7].dq = Float32(msg.dq)
    state.motorState[7].ddq = Float32(msg.ddq)
end
sim_joint_RR_hip_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RR_hip_controller/state", 
                                sim_joint_RR_hip_callback, (fbk_state,), queue_size=10)

function sim_joint_RR_thigh_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[8].mode = UInt8(msg.mode)
    state.motorState[8].q = Float32(msg.q)
    state.motorState[8].dq = Float32(msg.dq)
    state.motorState[8].ddq = Float32(msg.ddq)
end
sim_joint_RR_thigh_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RR_thigh_controller/state", 
                                sim_joint_RR_thigh_callback, (fbk_state,), queue_size=10)

function sim_joint_RR_calf_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[9].mode = UInt8(msg.mode)
    state.motorState[9].q = Float32(msg.q)
    state.motorState[9].dq = Float32(msg.dq)
    state.motorState[9].ddq = Float32(msg.ddq)
end
sim_joint_RR_calf_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RR_calf_controller/state", 
                                sim_joint_RR_calf_callback, (fbk_state,), queue_size=10)  


function sim_joint_RL_hip_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[10].mode = UInt8(msg.mode)
    state.motorState[10].q = Float32(msg.q)
    state.motorState[10].dq = Float32(msg.dq)
    state.motorState[10].ddq = Float32(msg.ddq)
end
sim_joint_RL_hip_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RL_hip_controller/state", 
                                sim_joint_RL_hip_callback, (fbk_state,), queue_size=10)

function sim_joint_RL_thigh_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[11].mode = UInt8(msg.mode)
    state.motorState[11].q = Float32(msg.q)
    state.motorState[11].dq = Float32(msg.dq)
    state.motorState[11].ddq = Float32(msg.ddq)
end
sim_joint_RL_thigh_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RL_thigh_controller/state", 
                                sim_joint_RL_thigh_callback, (fbk_state,), queue_size=10)

function sim_joint_RL_calf_callback(msg::unitree_legged_msgs.msg.MotorState, state::A1Robot.LowState)
    state.motorState[12].mode = UInt8(msg.mode)
    state.motorState[12].q = Float32(msg.q)
    state.motorState[12].dq = Float32(msg.dq)
    state.motorState[12].ddq = Float32(msg.ddq)
end
sim_joint_RL_calf_sub = Subscriber{unitree_legged_msgs.msg.MotorState}("a1_gazebo/RL_calf_controller/state", 
                                sim_joint_RL_calf_callback, (fbk_state,), queue_size=10)  

""" send sim data """
simcmd_list = [unitree_legged_msgs.msg.MotorCmd() for i=1:12]
sim_pub = []
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FR_hip_controller/command", queue_size = 10)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FR_thigh_controller/command", queue_size = 10)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FR_calf_controller/command", queue_size = 10)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FL_hip_controller/command", queue_size = 10)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FL_thigh_controller/command", queue_size = 10)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FL_calf_controller/command", queue_size = 10)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RR_hip_controller/command", queue_size = 10)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RR_thigh_controller/command", queue_size = 10)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RR_calf_controller/command", queue_size = 10)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RL_hip_controller/command", queue_size = 10)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RL_thigh_controller/command", queue_size = 10)
push!(sim_pub,pub)
pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RL_calf_controller/command", queue_size = 10)
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
ctrl_hz = 100.0
ctrl_dt = 1.0/ctrl_hz
loop_rate = Rate(ctrl_hz)

""" functions to control the robot """
# Cartesian space torque control 
# input reference position, velocity, acc (probably from a trajectory)
# input feedback joint angle, joint angular velocity
# input desired foot force (in body frame)
# output desired leg joint torque
function torque_ctrl(leg_ID::Int, 
    ref_p::Vector{Float64}, ref_v::Vector{Float64}, ref_a::Vector{Float64},
    q::Vector{Float64}, dq::Vector{Float64}, F::Vector{Float64})::Vector{Float64}
    
    J = A1Robot.J(leg_ID, q)
    dJ = A1Robot.dJ(leg_ID, q, dq)
    p = A1Robot.fk(leg_ID,q)
    v = J*dq
    M = A1Robot.getMassMtx(leg_ID,q)
    c = A1Robot.getVelQuadraticForces(leg_ID,q,dq)
    grav = A1Robot.getGravityForces(leg_ID,q)

    Kp = diagm([90;90;90])
    Kd = diagm([15;15;15])

    tau = J'*(Kp*(ref_p-p)+Kd*(ref_v-v)) + J'*inv(J)'*M*inv(J)*(ref_a-dJ*dq) + c + grav;
    # add the foot force 
    tau = tau + J'*F
    return tau
end
