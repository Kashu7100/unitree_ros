""" understand the rotation of Julia"""

using Rotations
using Printf
using OSQP
using SparseArrays
include("utils.jl")
# include("init_sim_robot.jl")
try
    """ global variables """
    """ get initial position of the leg """    
    # A1Robot.getFbkState!(robot, A1Robot.fbk_state)
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
    # the target foot position
    # ref_p_list = zeros(3,4) 
    # ref_v_list = zeros(3,4) 
    # for i=1:4
    #     leg_ID = i-1 # for leg ID we all use C style 
    #     ref_p_list[:,i] = A1Robot.fk(leg_ID, q_list[:,i]) 

    #     if leg_ID == A1Robot.C_FR_ || leg_ID == A1Robot.C_RR_
    #         ref_p_list[:,i]  += [0;-0.02;0]
    #     elseif leg_ID == A1Robot.C_FL_ || leg_ID == A1Robot.C_RL_
    #         ref_p_list[:,i]  += [0; 0.02;0]
    #     end
    # end
    # max_foot_height = maximum(ref_p_list[3,:])
    # ref_p_list[3,:] .= max_foot_height
    ref_body_position = [A1Robot.body_position[1];
                         A1Robot.body_position[2];
                         A1Robot.body_position[3]];
    standup_down_z_vel = 0;

    control_state = 0      # 0 is standing 
    # foot contact state  
    #TODO: add a scheduling
    foot_contact = [1,1,1,1]
    mass = 10.2
    mg = @SVector[0,0,mass*9.81]
    normal_load = mg/4.0
    F_prev = vcat(normal_load,normal_load,normal_load,normal_load)
    # constants for QP
    miu = 0.1
    Cb = @SMatrix [ 0    0    -1;
                    1    0  -miu;
                   -1    0  -miu;
                    0    1  -miu;
                    0   -1  -miu]
    # TODO: make this sparse?
    z3 = @SMatrix zeros(5,3)
    z33 = @SMatrix zeros(3,3)
    I3 = SMatrix{3,3}(1I)
    C = [Cb z3 z3 z3;
         z3 Cb z3 z3;
         z3 z3 Cb z3;
         z3 z3 z3 Cb]
    C = SMatrix{20,12}(C)  
    # the three parameters of the QP, different F components have different value    
    alpha_x = alpha_y = alpha_z = 2     
    beta_x = beta_y = beta_z = 0.3          
    s_vec = [1,1,10,55,55,55]  
    S = diagm(s_vec)
    S = SMatrix{6,6}(S)  
    alpha_vec = [alpha_x,alpha_y,alpha_z]  
    alpha_mtx = diagm(vcat(alpha_vec,alpha_vec,alpha_vec,alpha_vec))
    alpha_mtx = SMatrix{12,12}(alpha_mtx)
    beta_vec = [beta_x,beta_y,beta_z]  
    beta_mtx = diagm(vcat(beta_vec,beta_vec,beta_vec,beta_vec))   
    beta_mtx = SMatrix{12,12}(beta_mtx)

    while true
        """ this control loop runs 100Hz """

        """ get feedback """
        # A1Robot.getFbkState!(robot, A1Robot.fbk_state) # LowState is defined in test_julia_interface.jl
        for i=1:4
            leg_ID = i-1 # for leg ID we all use C style 
            q = [A1Robot.fbk_state.motorState[leg_ID*3+1].q;A1Robot.fbk_state.motorState[leg_ID*3+2].q;A1Robot.fbk_state.motorState[leg_ID*3+3].q]
            dq = [A1Robot.fbk_state.motorState[leg_ID*3+1].dq;A1Robot.fbk_state.motorState[leg_ID*3+2].dq;A1Robot.fbk_state.motorState[leg_ID*3+3].dq]
            q_list[:,i] = convert(Array{Float64,1}, q)
            dq_list[:,i] = convert(Array{Float64,1}, dq)
            foot_force_filter[i] = (1-foot_filter_const)*foot_force_filter[i] + foot_filter_const*A1Robot.fbk_state.footForce[i]
        end
        println(A1Robot.joy_data_buttons)
        # println(A1Robot.body_pose.position)
        curr_body_position = [A1Robot.body_position[1];
                              A1Robot.body_position[2];
                              A1Robot.body_position[3]];
        curr_body_vel = [A1Robot.body_velocity[1];
                            A1Robot.body_velocity[2];
                            A1Robot.body_velocity[3]];
        # println(A1Robot.body_twist.linear)
        rotation_q = UnitQuaternion(A1Robot.body_orientation[1],
                                    A1Robot.body_orientation[2],
                                    A1Robot.body_orientation[3],
                                    A1Robot.body_orientation[4])
        # println(q)
        yaw, roll, pitch = quat_to_euler(rotation_q)
        println("rotation_q")
        println([yaw/pi*180.0, roll/pi*180.0, pitch/pi*180.0])

        q_tilt, q_torsion = quat_decompose_tilt_torsion(rotation_q)
        yaw, roll, pitch = quat_to_euler(q_tilt)
        # println([yaw/pi*180.0, roll/pi*180.0, pitch/pi*180.0])

        """ safety """
        if A1Robot.joy_data_buttons[5] == 1
            A1Robot.joy_data_buttons[5] = 0
            break
        end

        """ different control state """
        if control_state == 0
            # control state 0, the standup state 
            # use joy stick axes 5 to control
            joy_input = convert(Float64, A1Robot.joy_data_axes[5])
            if (joy_input>0.2 || joy_input<-0.2)
                standup_down_z_vel = 0.02*joy_input
            else
                standup_down_z_vel = 0
            end            
            # modify reference p and reference v
            # for i=1:4
            #     ref_p_list[3,i] += ctrl_dt*standup_down_z_vel
            #     if ref_p_list[3,i] > -0.01
            #         ref_p_list[3,i] = -0.01
            #     elseif ref_p_list[3,i] < -0.26
            #         ref_p_list[3,i] = -0.26
            #     end
            #     ref_v_list[3,i] = standup_down_z_vel
            # end
            
            ref_body_position[3] += ctrl_dt*standup_down_z_vel
            if ref_body_position[3] < 0.05
                ref_body_position[3] = 0.05
            elseif ref_body_position[3] > 0.3
                ref_body_position[3] = 0.3
            end

            q_tgt = UnitQuaternion(1.0,0.0,0.0,0.0)
            q_err = q_tgt*conj(q_tilt)

            current_w = A1Robot.fbk_state.imu.gyroscope

            # println("current_w")
            # println(current_w)

            Kp_w_val = 0.0
            Kd_w_val = 0.0
            Kp = diagm([Kp_w_val;Kp_w_val;Kp_w_val])
            Kd = diagm([Kd_w_val;Kd_w_val;Kd_w_val])
            Inertia = diagm([0.17;0.13;0.17])
            q_err_log = log(q_err)
            q_err_vec = @SVector[q_err_log.x,q_err_log.y,q_err_log.z]
            e_w = Kp*q_err_vec - Kd*current_w
            obj = Inertia*e_w
            # @printf("%6.4f \t %6.4f \t %6.4f \n", obj[1],obj[2],obj[3])

            # println(ref_body_position)
            # println(curr_body_position)

            # construct QP
            Kp_a_val = 0.0
            Kd_a_val = 0.0
            Kp_a = diagm([0.0;0.0;Kp_a_val])
            Kd_a = diagm([0.0;0.0;Kd_a_val])
            ref_body_vel = [0;0;standup_down_z_vel]
            tgt_a = Kp_a*(ref_body_position-curr_body_position)+Kd_a*(-curr_body_vel)
            # println(tgt_a)
            ma = mass*tgt_a
            b = vcat(mg+ma,obj)
            pR_list = []
            # println(A1Robot.fbk_state.motorState)
            show(stdout, "text/plain", q_list)
            for i=1:4
                leg_ID = i - 1
                
                p = A1Robot.fk(leg_ID, q_list[:,i])
                # println("p")
                # println(p)
                pR = skew(convert(SVector{3},p))*conj(rotation_q) # this q convert force to body frame
                push!(pR_list, pR)
            end

            pRmtx = []
            Imtx = []
            for i=1:4
                # if foot_contact[i] == 1
                    if isempty(pRmtx)
                        pRmtx = pR_list[i]
                        Imtx = I3
                    else
                        pRmtx = hcat(pRmtx, pR_list[i])
                        Imtx = hcat(Imtx,I3)
                    end
                # end
            end
            A = vcat(Imtx,pRmtx)  # always 6x12

            # D confines the foot force
            D = sparse(I(12))
            ld = fill(-Inf,12)
            ud = fill(Inf,12)
            for i=1:4
                if foot_contact[i] == 0
                    ld[(i-1)*3:i*3] .= 0
                    ud[(i-1)*3:i*3] .= 0
                end
            end

            # QP 
            # F'(A'*S*A+alpha+beta)F - (A'Sb+2 beta F_rev)'F
            # Cx<=0
            # Dx = 0

            P = A'*S*A + alpha_mtx + beta_mtx
            q = -(A'*S*b + 2*beta_mtx*F_prev)
            model = OSQP.Model()
            OSQP.setup!(model, P=sparse(P), q=Vector(q), A=sparse([D; C]), l=[ld; fill(-Inf,20)], u=[ud; zeros(20)],
                eps_abs=1e-6, eps_rel=1e-6, verbose=false)
            res = OSQP.solve!(model)

            F = reshape(res.x,3,4)
            show(stdout, "text/plain", F)
            println("---")


            # show(stdout, "text/plain", ref_p_list)
            F_prev = res.x

            for i=1:4
                leg_ID = i-1
                foot_F = conj(rotation_q)*(-normal_load) # foot push into ground
                p = A1Robot.fk(leg_ID, q_list[:,i])
                foot_tau = p × foot_F
                # foot_tau = zeros(3)
                # println(vcat(foot_tau,foot_F))
                # show(stdout, "text/plain", F)
                tau = stance_torque_ctrl(leg_ID, q_list[:,i], Array(foot_F))
                println(tau)
                println([A1Robot.fbk_state.motorState[leg_ID*3+1].tauEst;
                         A1Robot.fbk_state.motorState[leg_ID*3+2].tauEst;
                         A1Robot.fbk_state.motorState[leg_ID*3+3].tauEst;])
                # tau = torque_ctrl(leg_ID, ref_p, ref_v, ref_a, q_list[:,i], dq_list[:,i], foot_F)
                # A1Robot.setCmdMotorTau(robot, leg_ID*3, Float32(tau[1]))
                # A1Robot.setCmdMotorTau(robot, leg_ID*3+1, Float32(tau[2]))
                # A1Robot.setCmdMotorTau(robot, leg_ID*3+2, Float32(tau[3]))
                sim_setCmdMotorTau(leg_ID*3, Float32(tau[1]))
                sim_setCmdMotorTau(leg_ID*3+1, Float32(tau[2]))
                sim_setCmdMotorTau(leg_ID*3+2, Float32(tau[3]))
            end
        end

        # """ send control torque to the robot """
        # A1Robot.SendCommand(robot)
        # sim_SendCommand()
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