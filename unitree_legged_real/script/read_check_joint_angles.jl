try
    while true
        """ this control loop runs 10Hz """
        sleep(0.1)

        """ first read feedback through the RobotInterface, state is stored in fbk_state """
        A1Robot.getFbkState!(robot, fbk_state) # LowState is defined in test_julia_interface.jl
        q_FR = [fbk_state.motorState[1].q;fbk_state.motorState[2].q;fbk_state.motorState[3].q]
        q_FR = convert(Array{Float64,1}, q_FR)
        p_FR = A1Robot.fk(A1Robot.C_FR_,q_FR)
        q_FL = [fbk_state.motorState[4].q;fbk_state.motorState[5].q;fbk_state.motorState[6].q]
        q_FL = convert(Array{Float64,1}, q_FL)
        p_FL = A1Robot.fk(A1Robot.C_FL_,q_FL)
        q_RR = [fbk_state.motorState[7].q;fbk_state.motorState[8].q;fbk_state.motorState[9].q]
        q_RR = convert(Array{Float64,1}, q_RR)
        p_RR = A1Robot.fk(A1Robot.C_RR_,q_RR)
        q_RL = [fbk_state.motorState[10].q;fbk_state.motorState[11].q;fbk_state.motorState[12].q]
        q_RL = convert(Array{Float64,1}, q_RL)
        p_RL = A1Robot.fk(A1Robot.C_RL_,q_RL)
        q_list = hcat(q_FL,q_FR,q_RL,q_RR)
        p_list = hcat(p_FL,p_FR,p_RL,p_RR)
        show(stdout, "text/plain", q_list)
        # show(stdout, "text/plain", p_list)
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