""" just an empty ros loop to trigger callback processing """
# using RobotOS
# @rosimport sensor_msgs.msg: Imu, Joy
# rostypegen()
# using .sensor_msgs.msg

# assume now we already have following steps:
# roscore
# roslaunch xsens_mti_driver xsens_mti_node.launch
# julia 
# julia> using RobotOS
# julia> init_node("rosjl_example",disable_signals="True")
# now julia REPL is a rosnode
# finally include this file 

# include("init_sim_robot.jl")
# standard wrapper to stop the program
try
    q_list = zeros(3,4)  # FR, FL, RR, RL
    dq_list = zeros(3,4)  # FR, FL, RR, RL
    foot_force_filter = zeros(4)
    foot_filter_const = 0.1
    for i=1:4
        leg_ID = i-1 # for leg ID we all use C style 
        q = [A1Robot.fbk_state.motorState[leg_ID*3+1].q;
             A1Robot.fbk_state.motorState[leg_ID*3+2].q;
             A1Robot.fbk_state.motorState[leg_ID*3+3].q]
        q_list[:,i] = convert(Array{Float64,1}, q)

        foot_force_filter[i] = (1-foot_filter_const)*foot_force_filter[i] + foot_filter_const*A1Robot.fbk_state.footForce[i]
    end    
    # function callback(msg::sensor_msgs.msg.Joy, pub_obj::Publisher{sensor_msgs.msg.Imu})
    #     # pt_msg = Point(msg.point.x, msg.point.y, 0.0)
    #     println(msg.linear_acceleration.z)
    # end
    # pub = Publisher{sensor_msgs.msg.Imu}("pts", queue_size=10)
    # sub = Subscriber{sensor_msgs.msg.Joy}("joy", callback, (pub,), queue_size=10)
    # loop_rate = Rate(5.0)
    while true
        """ this control loop runs 10Hz """
        for i=1:4
            leg_ID = i-1 # for leg ID we all use C style 
            q = [A1Robot.fbk_state.motorState[leg_ID*3+1].q;
                 A1Robot.fbk_state.motorState[leg_ID*3+2].q;
                 A1Robot.fbk_state.motorState[leg_ID*3+3].q]
            q_list[:,i] = convert(Array{Float64,1}, q)
    
            foot_force_filter[i] = (1-foot_filter_const)*foot_force_filter[i] + foot_filter_const*A1Robot.fbk_state.footForce[i]
        end 
        show(stdout, "text/plain", q_list)
        # println(joy_data.axes)
        # println(body_pose.orientation)
        # println(body_pose.position)
        # println(fbk_state.footForce[1])
        println(A1Robot.fbk_state.motorState[1].tauEst)

        """ read imu from ros  """
        
        rossleep(loop_rate)
    end
catch e
    if e isa InterruptException
       # cleanup
       println("control loop terminated by the user")
    #    rethrow(e)
    else
        println(e)
        rethrow(e)
        # println("some other error")
    end
end
