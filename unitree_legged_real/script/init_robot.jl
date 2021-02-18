"""
This script initialize the Julia controller for A1
In this script we start a robot interface to send/receive data from A1
Then we convert the current Julia REPL into a ros node 
"""
# start A1Robot nterface
include("julia_robot_interface.jl")

""" ROS related """
# convert the current Julia REPL as ros node
using RobotOS
@rosimport sensor_msgs.msg: Imu, Joy
rostypegen()
using .sensor_msgs.msg
init_node("rosjl",disable_signals="True")
# initialize necessary publisher, subscriber and callbacks
joy_data = sensor_msgs.msg.Joy()
joy_data.axes = zeros(Float32,8)
function joy_stick_callback(msg::sensor_msgs.msg.Joy, out_data::sensor_msgs.msg.Joy)
    # pt_msg = Point(msg.point.x, msg.point.y, 0.0)
    out_data.axes = msg.axes
    out_data.buttons = msg.buttons
    # println(msg.axes)
end
joy_stick_sub = Subscriber{sensor_msgs.msg.Joy}("joy", joy_stick_callback, (joy_data,), queue_size=1)

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
function torque_ctrl_simple(leg_ID::Int, 
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

    tau = J'*(Kp*(ref_p-p)+Kd*(ref_v-v)) + c + grav;
    # add the foot force 
    tau = tau + J'*F
    return tau
end