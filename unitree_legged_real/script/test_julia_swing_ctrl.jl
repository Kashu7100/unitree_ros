""" 
    swing control demo
In this program we start a control loop to move the FR leg. We want to examine the possibility of 
using Julia to control the torque of the robot leg
""" 

# helper function: generate hermite cubic trajectory
function hermite_cubic_knot(cur_t::Float64, t0::Float64, x0::Array{Float64,1},dx0::Array{Float64,1},
                                        t1::Float64, x1::Array{Float64,1},dx1::Array{Float64,1})
    p = x1*((3*(cur_t - t0)^2)/(t0 - t1)^2 + (2*(cur_t - t0)^3)/(t0 - t1)^3) - x0*((3*(cur_t - t0)^2)/(t0 - t1)^2 + (2*(cur_t - t0)^3)/(t0 - t1)^3 - 1) + dx0*(t0 - t1)*((2*(cur_t - t0)^2)/(t0 - t1)^2 + (cur_t - t0)^3/(t0 - t1)^3 + (cur_t - t0)/(t0 - t1)) + dx1*((cur_t - t0)^2/(t0 - t1)^2 + (cur_t - t0)^3/(t0 - t1)^3)*(t0 - t1);
    dp = x1*((3*(2*cur_t - 2*t0))/(t0 - t1)^2 + (6*(cur_t - t0)^2)/(t0 - t1)^3) - x0*((3*(2*cur_t - 2*t0))/(t0 - t1)^2 + (6*(cur_t - t0)^2)/(t0 - t1)^3) + dx1*((2*cur_t - 2*t0)/(t0 - t1)^2 + (3*(cur_t - t0)^2)/(t0 - t1)^3)*(t0 - t1) + dx0*(t0 - t1)*(1/(t0 - t1) + (2*(2*cur_t - 2*t0))/(t0 - t1)^2 + (3*(cur_t - t0)^2)/(t0 - t1)^3);
    ddp = x1*(6/(t0 - t1)^2 + (6*(2*cur_t - 2*t0))/(t0 - t1)^3) - x0*(6/(t0 - t1)^2 + (6*(2*cur_t - 2*t0))/(t0 - t1)^3) + dx1*(2/(t0 - t1)^2 + (3*(2*cur_t - 2*t0))/(t0 - t1)^3)*(t0 - t1) + dx0*(4/(t0 - t1)^2 + (3*(2*cur_t - 2*t0))/(t0 - t1)^3)*(t0 - t1);
    return p,dp,ddp
end

function hermite_cubic_traj(ref_p_list::Array{Float64,2}, ref_t_list::Array{Float64,1}, t::Float64)
    knots_idx = 1
    while t>=ref_t_list[knots_idx]
        knots_idx = knots_idx + 1;
        if knots_idx == size(ref_t_list,1)
            break
        end
    end
    knot1_idx = knots_idx-1
    knot2_idx = knots_idx
    return hermite_cubic_knot(
        t, 
        ref_t_list[knots_idx-1], ref_p_list[:,knots_idx-1], zeros(size(ref_p_list[:,1])), 
        ref_t_list[knots_idx],   ref_p_list[:,knots_idx],   zeros(size(ref_p_list[:,1])));    
end


# first start the robot, enter sudo su mode, start julia
# second include("test_julia_interface.jl"), which starts the RobotInterfae "robot"
# then, include this file in REPL to start the control loop
# terminate the control loop by ctrl-c
using Dates
using LinearAlgebra
const leg_ID = A1Robot.C_RR_;
# const leg_ID = A1Robot.C_FR_;
try
    A1Robot.InitSend(robot)
    # get inital leg position, generate a trajectory 
    """ get initial position of the leg """    
    A1Robot.getFbkState!(robot, fbk_state)
    q = [fbk_state.motorState[leg_ID*3+1].q;fbk_state.motorState[leg_ID*3+2].q;fbk_state.motorState[leg_ID*3+3].q]
    q = convert(Array{Float64,1}, q)
    init_p = A1Robot.fk(leg_ID, q) 
    if leg_ID == A1Robot.C_FR_ || leg_ID == A1Robot.C_RR_
        init_p += [0;-0.02;0]
        tgt_p = init_p + [0.13;0;0]
        mid_p = (tgt_p+init_p)/2+[0;-0.08;0.08]
    elseif leg_ID == A1Robot.C_FL_ || leg_ID == A1Robot.C_RL_
        init_p += [0; 0.02;0]
        tgt_p = init_p + [0.13;0;0]
        mid_p = (tgt_p+init_p)/2+[0;0.08;0.08]
    end
    ref_p_list = [init_p mid_p tgt_p init_p]
    ref_t_list = [0.0,1.0/3.0,2.0/3.0,1.0]

    start_t = Dates.DateTime(now())
    cur_period_t = 0;
    while true
        """ this control loop runs 500Hz """
        sleep(0.002)
        current_t = Dates.DateTime(now())
        total_run_time = Dates.value.((current_t-start_t))/1000.0
        cur_period_t = mod(total_run_time, 1)

        """ first read feedback through the RobotInterface, state is stored in fbk_state """
        A1Robot.getFbkState!(robot, fbk_state) # LowState is defined in test_julia_interface.jl
        # debug print to show we get 
        # k = zeros(Float32,3)
        # for i=1:3
        #     k[i] = fbk_state.motorState[i].tauEst
        # end
        # @show k

        q = [fbk_state.motorState[leg_ID*3+1].q;fbk_state.motorState[leg_ID*3+2].q;fbk_state.motorState[leg_ID*3+3].q]
        dq = [fbk_state.motorState[leg_ID*3+1].dq;fbk_state.motorState[leg_ID*3+2].dq;fbk_state.motorState[leg_ID*3+3].dq]
        q = convert(Array{Float64,1}, q)
        dq = convert(Array{Float64,1}, dq)
        

        """ generate a minimum jerk trajectory for the FR leg """
        ref_p, ref_v, ref_a = hermite_cubic_traj(ref_p_list, ref_t_list, cur_period_t)
        
        """ generate control torque using the kinematics and dynamics of the robot """
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
        @show tau

        # tau = zeros(Float32, 3)
        """ send control torque to the robot """
        A1Robot.setCmdMotorTau(robot, leg_ID*3, Float32(tau[1]))
        A1Robot.setCmdMotorTau(robot, leg_ID*3+1, Float32(tau[2]))
        A1Robot.setCmdMotorTau(robot, leg_ID*3+2, Float32(tau[3]))

        A1Robot.SendCommand(robot)
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