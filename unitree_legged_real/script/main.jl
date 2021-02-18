""" 
    main control loop
In this program we let the robot stand up and move around
We first use our own loop time,
but I suspect we will need to use ROS time to better sync with other things


inlcude("init_robot.jl") before this 
""" 

using LinearAlgebra
using Dates
using RobotOS

# include("init_robot.jl")

try
    A1Robot.InitSend(robot)
    """ global variables """
    control_state = 0      # 0 is standing 


    # get inital leg position, generate a trajectory 
    """ get initial position of the leg """    
    A1Robot.getFbkState!(robot, fbk_state)
    q_list = zeros(3,4)  # FR, FL, RR, RL
    dq_list = zeros(3,4)  # FR, FL, RR, RL
    foot_force_filter = zeros(4)
    foot_filter_const = 0.1
    for i=1:4
        leg_ID = i-1 # for leg ID we all use C style 
        q = [fbk_state.motorState[leg_ID*3+1].q;
             fbk_state.motorState[leg_ID*3+2].q;
             fbk_state.motorState[leg_ID*3+3].q]
        q_list[:,i] = convert(Array{Float64,1}, q)

        foot_force_filter[i] = (1-foot_filter_const)*foot_force_filter[i] + foot_filter_const*fbk_state.footForce[i]
    end

    # the target foot position
    ref_p_list = zeros(3,4) 
    ref_v_list = zeros(3,4) 
    for i=1:4
        leg_ID = i-1 # for leg ID we all use C style 
        ref_p_list[:,i] = A1Robot.fk(leg_ID, q_list[:,i]) 

        if leg_ID == A1Robot.C_FR_ || leg_ID == A1Robot.C_RR_
            ref_p_list[:,i]  += [0;-0.02;0]
        elseif leg_ID == A1Robot.C_FL_ || leg_ID == A1Robot.C_RL_
            ref_p_list[:,i]  += [0; 0.02;0]
        end
    end
    max_foot_height = maximum(ref_p_list[3,:])
    ref_p_list[3,:] .= max_foot_height
    # init_p = A1Robot.fk(leg_ID, q) 
    # if leg_ID == A1Robot.C_FR_ || leg_ID == A1Robot.C_RR_
    #     init_p += [0;-0.02;0]
    #     tgt_p = init_p + [0.13;0;0]
    #     mid_p = (tgt_p+init_p)/2+[0;-0.08;0.08]
    # elseif leg_ID == A1Robot.C_FL_ || leg_ID == A1Robot.C_RL_
    #     init_p += [0; 0.02;0]
    #     tgt_p = init_p + [0.13;0;0]
    #     mid_p = (tgt_p+init_p)/2+[0;0.08;0.08]
    # end
    # ref_p_list = [init_p mid_p tgt_p init_p]
    # ref_t_list = [0.0,1.0/3.0,2.0/3.0,1.0]

    # start_t = Dates.DateTime(now())
    # cur_period_t = 0;
    standup_down_z_vel = 0;

    while true
        """ this control loop runs at the loop_rate (defined in init_robot.jl) """

        """ get feedback """
        A1Robot.getFbkState!(robot, fbk_state) # LowState is defined in test_julia_interface.jl
        for i=1:4
            leg_ID = i-1 # for leg ID we all use C style 
            q = [fbk_state.motorState[leg_ID*3+1].q;fbk_state.motorState[leg_ID*3+2].q;fbk_state.motorState[leg_ID*3+3].q]
            dq = [fbk_state.motorState[leg_ID*3+1].dq;fbk_state.motorState[leg_ID*3+2].dq;fbk_state.motorState[leg_ID*3+3].dq]
            q_list[:,i] = convert(Array{Float64,1}, q)
            dq_list[:,i] = convert(Array{Float64,1}, dq)
            foot_force_filter[i] = (1-foot_filter_const)*foot_force_filter[i] + foot_filter_const*fbk_state.footForce[i]
        end

        """ different control state """
        if control_state == 0
            # control state 0, the standup state 
            # use joy stick axes 5 to control
            joy_input = convert(Float64, joy_data.axes[5])
            if (joy_input>0.2 || joy_input<-0.2)
                standup_down_z_vel = 0.1*joy_input
            else
                standup_down_z_vel = 0
            end
            @show joy_data.axes

            # modify reference p and reference v
            for i=1:4
                ref_p_list[3,i] += ctrl_dt*standup_down_z_vel
                if ref_p_list[3,i] > -0.01
                    ref_p_list[3,i] = -0.01
                elseif ref_p_list[3,i] < -0.26
                    ref_p_list[3,i] = -0.26
                end
                ref_v_list[3,i] = standup_down_z_vel
            end
            # @show ref_p_list
            @show fbk_state.footForce
            show(stdout, "text/plain", ref_p_list)
            println("--")        
            for i=1:4
                leg_ID = i-1
                foot_F = zeros(3)
                # if (foot_force_filter[i] > 30)
                    foot_F = [0;0; -17.7*9.81/4.0]
                # end
                current_p = A1Robot.fk(leg_ID, q_list[:,i]) 
                # traj_p_list = [current_p ref_p_list[:,i]]
                # traj_t_list = [0.0,0.1]
                # ref_p, ref_v, ref_a = hermite_cubic_traj(traj_p_list, traj_t_list, 0.1)
                ref_p = ref_p_list[:,i]
                ref_v = ref_v_list[:,i]
                ref_a = [0.0,0.0,0.0]
        


                # J = A1Robot.J(leg_ID, q_list[:,i])
                # dJ = A1Robot.dJ(leg_ID, q_list[:,i], dq_list[:,i])
                # p = A1Robot.fk(leg_ID,q_list[:,i])
                # v = J*dq_list[:,i]
                # M = A1Robot.getMassMtx(leg_ID,q_list[:,i])
                # c = A1Robot.getVelQuadraticForces(leg_ID,q_list[:,i],dq_list[:,i])
                # grav = A1Robot.getGravityForces(leg_ID,q_list[:,i])
            
                # Kp = diagm([600;600;300])
                # Kd = diagm([15;15;15])
            
                # tau = J'*(Kp*(ref_p-p)+Kd*(ref_v-v)) + c + grav;
               

                tau = torque_ctrl(leg_ID, ref_p, ref_v, ref_a, q_list[:,i], dq_list[:,i], foot_F)
                # @show tau
                # if leg_ID == 0
                    @show q_list[:,i]
                    @show current_p
                    # @show ref_p_list[:,i]
                    # @show tau
                    A1Robot.setCmdMotorTau(robot, leg_ID*3, Float32(tau[1]))
                    A1Robot.setCmdMotorTau(robot, leg_ID*3+1, Float32(tau[2]))
                    A1Robot.setCmdMotorTau(robot, leg_ID*3+2, Float32(tau[3]))
                # end
            end
        end

        # tau = zeros(Float32, 3)
        # A1Robot.setCmdMotorTau(robot, leg_ID*3, Float32(tau[1]))
        # A1Robot.setCmdMotorTau(robot, leg_ID*3+1, Float32(tau[2]))
        # A1Robot.setCmdMotorTau(robot, leg_ID*3+2, Float32(tau[3]))

        """ send control torque to the robot """
        A1Robot.SendCommand(robot)
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