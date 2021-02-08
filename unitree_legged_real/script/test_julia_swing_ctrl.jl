""" 
    swing control demo
In this program we start a control loop to move the FR leg. We want to examine the possibility of 
using Julia to control the torque of the robot leg
""" 


# first start the robot, enter sudo su mode, start julia
# second include("test_julia_interface.jl"), which starts the RobotInterfae "robot"
# then, include this file in REPL to start the control loop
# terminate the control loop by ctrl-c
try
    while true
        """ this control loop runs 100Hz """
        sleep(0.01)

        """ first read feedback through the RobotInterface, state is stored in fbk_state """
        A1Robot.getFbkState!(robot, fbk_state) # LowState is defined in test_julia_interface.jl
        # debug print to show we get 
        # k = zeros(Float32,3)
        # for i=1:3
        #     k[i] = fbk_state.motorState[i].tauEst
        # end
        # @show k

        """ get initial position of the FR leg """    

        """ generate a minimum jerk trajectory for the FR leg """
        
        
        """ generate control torque using the kinematics and dynamics of the robot """


        """ send control torque to the robot """

    end
catch e
    if e isa InterruptException
       # cleanup
       println("control loop terminated by the user")
    #    rethrow(e)
    else
        println("some other error")
    end
end