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

# standard wrapper to stop the program
try
    # function callback(msg::sensor_msgs.msg.Joy, pub_obj::Publisher{sensor_msgs.msg.Imu})
    #     # pt_msg = Point(msg.point.x, msg.point.y, 0.0)
    #     println(msg.linear_acceleration.z)
    # end
    # pub = Publisher{sensor_msgs.msg.Imu}("pts", queue_size=10)
    # sub = Subscriber{sensor_msgs.msg.Joy}("joy", callback, (pub,), queue_size=10)
    # loop_rate = Rate(5.0)
    while true
        """ this control loop runs 10Hz """
        
        println(joy_data.axes)

        """ read imu from ros  """
        
        # rossleep(loop_rate)
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