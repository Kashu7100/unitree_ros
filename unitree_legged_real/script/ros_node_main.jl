#include("init_sim_env.jl")
if ! isinteractive()
    include("init_sim_env.jl")
end

# joy callback
function joy_stick_callback(msg::sensor_msgs.msg.Joy, out_data::A1Robot.Joy)
    for i=1:8
        out_data.axes[i] = msg.axes[i]
    end
    for i=1:11
        out_data.buttons[i] = msg.buttons[i]
    end
end

# base state callback
function sim_pose_callback(msg::nav_msgs.msg.Odometry, base_state::A1Robot.BaseState)
    # pt_msg = Point(msg.point.x, msg.point.y, 0.0)
    base_state.position[1] = msg.pose.pose.position.x
    base_state.position[2] = msg.pose.pose.position.y
    base_state.position[3] = msg.pose.pose.position.z
    base_state.orientation[2] = msg.pose.pose.orientation.x
    base_state.orientation[3] = msg.pose.pose.orientation.y
    base_state.orientation[4] = msg.pose.pose.orientation.z
    base_state.orientation[1] = msg.pose.pose.orientation.w
    base_state.velocity[1] = msg.twist.twist.linear.x
    base_state.velocity[2] = msg.twist.twist.linear.y
    base_state.velocity[3] = msg.twist.twist.linear.z
end

# motor state callbacks
function sim_low_state_callback(msg::unitree_legged_msgs.msg.LowState, state::A1Robot.LowState)
    for i=1:12
        state.motorState[i].q = msg.motorState[i].q
        state.motorState[i].dq = msg.motorState[i].dq
        state.motorState[i].tauEst = msg.motorState[i].tauEst
    end
end

# foot force callback 
function sim_foot_force_FR_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    state.footForce[1] = convert(Int16, floor(msg.wrench.force.z*10))
end
function sim_foot_force_FL_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    state.footForce[2] = convert(Int16, floor(msg.wrench.force.z*10))
end
function sim_foot_force_RR_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    state.footForce[3] = convert(Int16, floor(msg.wrench.force.z*10))
end
function sim_foot_force_RL_callback(msg::geometry_msgs.msg.WrenchStamped, state::A1Robot.LowState)
    state.footForce[4] = convert(Int16, floor(msg.wrench.force.z*10))
end

# IMU callback
function sim_imu_callback(msg::sensor_msgs.msg.Imu, state::A1Robot.LowState)
    state.imu.gyroscope[1] = msg.angular_velocity.x
    state.imu.gyroscope[2] = msg.angular_velocity.y
    state.imu.gyroscope[3] = msg.angular_velocity.z
end

""" main entrance """
function main(ctrl_mode::Integer)
    init_node("rosjl_main_node",disable_signals="True")

    """ all robot feedback subscriber"""
    # joy data and sub
    joy_data = A1Robot.Joy()
    joy_stick_sub = Subscriber{sensor_msgs.msg.Joy}("joy", joy_stick_callback,(joy_data,), queue_size=1)

    # body pose state estimation sub
    base_state = A1Robot.BaseState()
    base_state.orientation[1] = 1.0
    sim_pose_sub = Subscriber{nav_msgs.msg.Odometry}("body_pose_ground_truth", sim_pose_callback, (base_state,), queue_size=1)

    # motor states
    fbk_state = A1Robot.LowState()
    for i=1:12
        fbk_state.motorState[i].q = 0.01
        fbk_state.motorState[i].dq = 0.0
    end
    sim_low_state_sub = Subscriber{unitree_legged_msgs.msg.LowState}("a1_gazebo/lowState/state", 
        sim_low_state_callback, (fbk_state,), queue_size=1)

    # foot force sub
    sim_footforceFR_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/FR_foot_contact/the_force", 
        sim_foot_force_FR_callback, (fbk_state,), queue_size=1)
    sim_footforceFL_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/FL_foot_contact/the_force", 
        sim_foot_force_FL_callback, (fbk_state,), queue_size=1)
    sim_footforceRR_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/RR_foot_contact/the_force", 
        sim_foot_force_RR_callback, (fbk_state,), queue_size=1)
    sim_footforceRL_sub = Subscriber{geometry_msgs.msg.WrenchStamped}("visual/RL_foot_contact/the_force", 
        sim_foot_force_RL_callback, (fbk_state,), queue_size=1)

    # IMU sub
    sim_imu_sub = Subscriber{sensor_msgs.msg.Imu}("trunk_imu", sim_imu_callback, (fbk_state,), queue_size=1)   

    """ send sim data data and pub"""
    simcmd_list = [unitree_legged_msgs.msg.MotorCmd() for i=1:12]
    sim_pub_list  = []
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FR_hip_controller/command", queue_size=10)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FR_thigh_controller/command", queue_size=10)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FR_calf_controller/command", queue_size=10)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FL_hip_controller/command", queue_size=10)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FL_thigh_controller/command", queue_size=10)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/FL_calf_controller/command", queue_size=10)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RR_hip_controller/command", queue_size=10)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RR_thigh_controller/command", queue_size=10)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RR_calf_controller/command", queue_size=10)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RL_hip_controller/command", queue_size=10)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RL_thigh_controller/command", queue_size=10)
    push!(sim_pub_list,pub)
    pub = Publisher{unitree_legged_msgs.msg.MotorCmd}("a1_gazebo/RL_calf_controller/command", queue_size=10)
    push!(sim_pub_list,pub)
    
    """ start control loop """
    try
        loop(fbk_state, joy_data, base_state, simcmd_list, sim_pub_list, ctrl_mode)
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
end

function sim_setCmdMotorTau(id::Int, tau::Float32, simcmd_list)
    simcmd_list[id+1].mode = 0x0A
    simcmd_list[id+1].q = 2.146e+9;        # 禁止位置环
    simcmd_list[id+1].Kp = 0;
    simcmd_list[id+1].dq = 16000.0;        # 禁止速度环
    simcmd_list[id+1].Kd = 0;
    simcmd_list[id+1].tau = tau
end

function sim_SendCommand(simcmd_list, sim_pub_list)
    #ros send
    for i=1:12
        publish(sim_pub_list[i], simcmd_list[i])
    end
end


function loop(fbk_state, joy_data, base_state, simcmd_list, sim_pub_list, ctrl_mode)
    ctrl_hz = 250.0
    ctrl_dt = 1.0/ctrl_hz
    loop_rate = Rate(ctrl_hz)   
    
    q_list = zeros(3,4)  # FR, FL, RR, RL
    dq_list = zeros(3,4)  # FR, FL, RR, RL

    p_eb = zeros(3)
    v_eb = zeros(3)
    q_eb = UnitQuaternion(1.0,0.0,0.0,0.0)
    leg_ID = 0.0
    leg_tau = zeros(3)

    rossleep(Rate(Duration(4)))
    """ get init robot position """
    ref_base_position = [base_state.position[1];
                         base_state.position[2];
                         base_state.position[3]];
    ref_base_position0 = [base_state.position[1];
                        base_state.position[2];
                        base_state.position[3]];    
    q_eb = UnitQuaternion(base_state.orientation[1],
                            base_state.orientation[2],
                            base_state.orientation[3],
                            base_state.orientation[4])    
    q_eb0 = UnitQuaternion(base_state.orientation[1],
                            base_state.orientation[2],
                            base_state.orientation[3],
                            base_state.orientation[4])  
    # get initial joint angles
    for i=1:4
        leg_ID = i-1 # for leg ID we all use C style 
        for j=1:3
            q_list[j,i] = convert(Float64, fbk_state.motorState[leg_ID*3+j].q)
            dq_list[j,i] = convert(Float64, fbk_state.motorState[leg_ID*3+j].dq)
        end
    end  
    init_foot_pos_b = zeros(3,4)
    nominal_foot_pos_b = zeros(3,4)
    nominal_foot_pos_e = zeros(3,4)
    ref_foot_pos_e = zeros(3,4)
    for i=1:4
        leg_ID = i-1
        init_foot_pos_b[:,i] = A1Robot.fk(leg_ID, q_list[:,i])
        nominal_foot_pos_b[:,i] = A1Robot.fk(leg_ID, [0.0;0.6;-1.3])
        nominal_foot_pos_e[:,i] = ref_base_position + q_eb*nominal_foot_pos_b[:,i]
    end
    show(stdout, "text/plain", q_list)
    println(" ")
    show(stdout, "text/plain", init_foot_pos_b)
    show(stdout, "text/plain", nominal_foot_pos_b)
    ref_foot_pos_e = nominal_foot_pos_e 
    # create a test trajectory for control_state = 2

    # init_p = nominal_foot_pos_b[:,1]
    # tgt_p = init_p + [0.13;0;0]
    # mid_p = (tgt_p+init_p)/2+[0;-0.08;0.11]
    # ref_p_list = [init_p mid_p tgt_p init_p]
    # gait_circle= 1
    # ref_t_list = [0.0,gait_circle*0.3,gait_circle*0.6,gait_circle]
            
    standup_down_z_vel = 0;
    control_state = ctrl_mode      # 0 is standing, joy control orientation
                                   # 1 is moving 
                           # 1 is moving 
                                   # 1 is moving 
                           # 1 is moving 
                                   # 1 is moving 
    # foot contact state  
    #TODO: add a scheduling
    foot_contact = [1,1,1,1] 
    foot_contact_prev = [1,1,1,1] 
    foot_contact_schedule = [1,1,1,1] # this is plan
    foot_contact_fbk = [1,1,1,1] # this is plan
    gait_time = 0.0
    gait_total_time = 0.66 # this is a parameter controls the length of each gait
    start_t = get_rostime()
    current_t = get_rostime()
    FR_gait_phase = [0.05,0.45]      # lift leg time: first number*gait_total_time, touch down leg time: second number*gait_total_time
    FL_gait_phase = [0.55,0.95]
    RR_gait_phase = [0.55,0.95]
    RL_gait_phase = [0.05,0.45]
    leg_gait_phase = [FR_gait_phase,FL_gait_phase,RR_gait_phase,RL_gait_phase]
    gait_is_running = 0          # this flag shows whether the gait scheduler is running or not
    
    # variables for swing trajectory generation
    swing_cur_foot_pos = zeros(3,4)
    swing_mid_foot_pos = zeros(3,4)
    swing_tgt_foot_pos = zeros(3,4)
    swing_tgt_foot_pos_e = nominal_foot_pos_e
    swing_traj_time = zeros(3,4)

    #  task space
    # Kp_swing = diagm([3400.0;3200.0;2600.0])
    # Ki_swing = diagm([88.0;88.0;88.0])
    # Kd_swing = diagm([550.0;200.0;500.0])
    #joint space
    Kp_swing = diagm([90.0;90.0;90.0])
    Ki_swing = diagm([80.0;80.0;80.0])
    Kd_swing = diagm([15.0;15.0;15.0])

    # variables for swing and stance leg torque
    tau_swing = zeros(3,4)
    tau_stance = zeros(3,4)
    tau = zeros(3,4)
    error_pos_foot = zeros(3,4)

    mass = 12.5
    mg = @SVector[0,0,mass*9.81]
    normal_load = mg/4.0
    F_prev = vcat(normal_load,normal_load,normal_load,normal_load)
    # constants for QP
    miu = 0.3
    Cb = @SMatrix [ 0    0    -1;
                    1    0  -miu;
                   -1    0  -miu;
                    0    1  -miu;
                    0   -1  -miu]
    # russ tedrake, friction cone 

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
    alpha_x = alpha_y = alpha_z = 0.7
    beta_x = beta_y = beta_z = 0.05         
    s_vec = [1,1,1,60,60,60]  
    S = diagm(s_vec)
    S = SMatrix{6,6}(S)  
    alpha_vec = [alpha_x,alpha_y,alpha_z]  
    alpha_mtx = diagm(vcat(alpha_vec,alpha_vec,alpha_vec,alpha_vec))
    alpha_mtx = SMatrix{12,12}(alpha_mtx)
    beta_vec = [beta_x,beta_y,beta_z]  
    beta_mtx = diagm(vcat(beta_vec,beta_vec,beta_vec,beta_vec))   
    beta_mtx = SMatrix{12,12}(beta_mtx)

    """ variables used in the loop """
    # frame definition 
    # e - world frame
    # b - body frame 
    
    # R_eb, q_eb
    # pe = q_eb*pb
    # a mamathecial introduction to robot manipulation
    # modern robotics 

    # debug pub
    debug_pub = Publisher{geometry_msgs.msg.Quaternion}("rosjl/debug/point1", queue_size=1)
    debug_point = geometry_msgs.msg.Quaternion()
    debug_pub2 = Publisher{geometry_msgs.msg.Quaternion}("rosjl/debug/point2", queue_size=1)
    debug_point2 = geometry_msgs.msg.Quaternion()
    debug_pub3 = Publisher{geometry_msgs.msg.Quaternion}("rosjl/debug/point3", queue_size=1)
    debug_point3 = geometry_msgs.msg.Quaternion()

    while ! is_shutdown()

        """ get feedback """
        # get joint angles
        for i=1:4
            leg_ID = i-1 # for leg ID we all use C style 
            for j=1:3
                q_list[j,i] = convert(Float64, fbk_state.motorState[leg_ID*3+j].q)
                dq_list[j,i] = convert(Float64, fbk_state.motorState[leg_ID*3+j].dq)
            end
        end  
        # get current base states
        for i=1:3
            p_eb[i] = base_state.position[i]
            v_eb[i] = base_state.velocity[i]
        end
        q_eb = UnitQuaternion(base_state.orientation[1],
                              base_state.orientation[2],
                              base_state.orientation[3],
                              base_state.orientation[4])

        """ safety """
        if joy_data.buttons[5] == 1
            joy_data.buttons[5] = 0
            break
        end

        """ switch control state """

        """ different control state """
        """ in control state 0 we move orientation using joy stick """
        """ in control state 1 we """
        if control_state == 0
            # control state 0, the standup state 
            # use joy stick axes 5 to control
            joy_input = convert(Float64, joy_data.axes[2])
            if (joy_input>0.2 || joy_input<-0.2)
                standup_down_z_vel = 0.2*joy_input
            else
                standup_down_z_vel = 0
            end            
            x_vel = 0
            y_vel = 0
            joy_input = convert(Float64, joy_data.axes[5])
            if (joy_input>0.2 || joy_input<-0.2)
                x_vel = 0.05*joy_input
            else
                x_vel = 0
            end      
            joy_input = convert(Float64, joy_data.axes[4])
            if (joy_input>0.2 || joy_input<-0.2)
                y_vel = 0.05*joy_input
            else
                y_vel = 0
            end      
            
            ref_base_position[3] += ctrl_dt*standup_down_z_vel
            if ref_base_position[3] < 0.05
                ref_base_position[3] = 0.05
            elseif ref_base_position[3] > 0.3
                ref_base_position[3] = 0.3
            end
            ref_base_position[1] += ctrl_dt*x_vel
            if ref_base_position[1] < ref_base_position0[1]-0.05
                ref_base_position[1] = ref_base_position0[1]-0.05
            elseif ref_base_position[1] > ref_base_position0[1]+0.05
                ref_base_position[1] = ref_base_position0[1]+0.05
            end
            ref_base_position[2] += ctrl_dt*y_vel
            if ref_base_position[2] < ref_base_position0[2]-0.05
                ref_base_position[2] = ref_base_position0[2]-0.05
            elseif ref_base_position[2] > ref_base_position0[2]+0.05
                ref_base_position[2] = ref_base_position0[2]+0.05
            end

            # tgt_yaw = joy_data.axes[1]*30/180*pi
            # tgt_roll = joy_data.axes[4]*30/180*pi
            # tgt_pitch = joy_data.axes[5]*30/180*pi
            tgt_yaw = joy_data.axes[1]*30/180*pi
            tgt_roll = joy_data.axes[7]*20/180*pi
            tgt_pitch = joy_data.axes[8]*20/180*pi

            q_tilt, q_torsion = quat_decompose_tilt_torsion(q_eb)
            q_tgt = q_torsion # TODO: get current yaw 
            # q_tgt = UnitQuaternion(1.0,0.0,0.0,0.0) # TODO: get current yaw 
            q_tgt = UnitQuaternion(RotZYX(tgt_yaw, tgt_roll, tgt_pitch)) # TODO: get current yaw 
            q_err = q_tgt*conj(q_eb)

            w_b = fbk_state.imu.gyroscope
            w_e = q_eb*w_b
            # println("current_w")
            # println(current_w)

            Kp = diagm([120;220;60])
            Kd = diagm([5;25;10])
            Inertia = diagm([0.1;0.1;0.1])
            Ie = q_eb*Inertia*conj(q_eb)
            q_err_log = log(q_err)
            q_err_vec = @SVector[q_err_log.x,q_err_log.y,q_err_log.z]
            e_w = Kp*q_err_vec + Kd*(-w_e)
            tgt_wa = Ie*e_w
            # @printf("%6.4f \t %6.4f \t %6.4f \n", tgt_wa[1],tgt_wa[2],tgt_wa[3])
            # debug_point2.x = tgt_wa[1]
            # debug_point2.y = tgt_wa[2]
            # debug_point2.z = tgt_wa[3]
            # publish(debug_pub2, debug_point2)

            # println(ref_base_position)
            # println(p_eb)

            # construct QP
            Kp_a = diagm([50;50;20])
            Kd_a = diagm([25;25;10])
            ref_body_vel = [x_vel;y_vel;standup_down_z_vel]
            tgt_a = Kp_a*(ref_base_position-p_eb)+Kd_a*(ref_body_vel-v_eb)
            # println(tgt_a)
            ma = mass*tgt_a
            b = vcat(ma+mg,tgt_wa)

            pR_list = []
            # println(A1Robot.fbk_state.motorState)
            # show(stdout, "text/plain", q_list)
            for i=1:4
                leg_ID = i - 1
                
                p = A1Robot.fk(leg_ID, q_list[:,i])
                p = convert(SVector{3},p)
                # println("p")
                # println(p)
                pR = skew(q_eb*p) # this q convert distance to world franme
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
            # println(A)

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

            # print(b)
            P = A'*S*A + alpha_mtx + beta_mtx
            # print(P)
            q = -(A'*S*b + 2*beta_mtx*F_prev)
            # print(q)
            model = OSQP.Model()
            OSQP.setup!(model, P=sparse(P), q=Vector(q), A=sparse([D; C]), l=[ld; fill(-Inf,20)], u=[ud; zeros(20)],
                eps_abs=1e-6, eps_rel=1e-6, verbose=false)
            res = OSQP.solve!(model)

            F = reshape(res.x,3,4)
            
            # show(stdout, "text/plain", F)
            # println("---")

            # debug_point.x = F[3,1]
            # debug_point.y = F[3,2]
            # debug_point.z = F[3,3]
            # debug_point.w = F[3,4]

            # publish(debug_pub, debug_point)
            # debug_point2.x =0.0
            # debug_point2.y = F[3,4]
            # debug_point2.z = 0.0
            # publish(debug_pub2, debug_point2)

            # show(stdout, "text/plain", ref_p_list)
            F_prev = res.x

            for i=1:4
                leg_ID = i-1
                foot_F = conj(q_eb)*(-F[:,i]) # foot push into ground
                # foot_tau = zeros(3)
                # println(vcat(foot_tau,foot_F))
                # show(stdout, "text/plain", F)
                tau = A1Robot.stance_torque_ctrl(leg_ID, q_list[:,i], Array(foot_F))
                # println(tau)
                # println([fbk_state.motorState[leg_ID*3+1].tauEst;
                #          fbk_state.motorState[leg_ID*3+2].tauEst;
                        #  fbk_state.motorState[leg_ID*3+3].tauEst;])
                # tau = torque_ctrl(leg_ID, ref_p, ref_v, ref_a, q_list[:,i], dq_list[:,i], foot_F)
                # A1Robot.setCmdMotorTau(robot, leg_ID*3, Float32(tau[1]))
                # A1Robot.setCmdMotorTau(robot, leg_ID*3+1, Float32(tau[2]))
                # A1Robot.setCmdMotorTau(robot, leg_ID*3+2, Float32(tau[3]))
                sim_setCmdMotorTau(leg_ID*3, Float32(tau[1]),simcmd_list)
                sim_setCmdMotorTau(leg_ID*3+1, Float32(tau[2]),simcmd_list)
                sim_setCmdMotorTau(leg_ID*3+2, Float32(tau[3]),simcmd_list)
            end
            sim_SendCommand(simcmd_list, sim_pub_list)

        elseif control_state == 1
            """ use this control state to test gait generator """
            tgt_vy = joy_data.axes[4]*0.06
            tgt_vx = joy_data.axes[5]*0.1

            tgt_vel = SVector{3}(tgt_vx, tgt_vy,0)
            horiz_vel = SVector{3}(v_eb[1], v_eb[2],0)

            if joy_data.buttons[3] > 0.05
                if gait_is_running == 0
                    gait_is_running = 1
                    start_t = get_rostime()
                    gait_time = 0.0
                    foot_contact = [1,1,1,1]
                end
            else
                if gait_time >= gait_total_time
                    gait_is_running = 0
                    gait_time = 0.0
                    # record a new ref_base_position, state estimation may drift
                    # ref_base_position = [base_state.position[1];
                    #      base_state.position[2];
                    #      ref_base_position0[3]];
                end
                foot_contact = [1,1,1,1]
                foot_contact_schedule = [1,1,1,1]
            end

            if gait_is_running == 1
                # reset time
                if gait_time >= gait_total_time
                    gait_time = 0.0
                    start_t = get_rostime()
                end
                # # always reset body position
                # ref_base_position = [base_state.position[1];
                #      base_state.position[2];
                #      ref_base_position0[3]];

                # current_t = Dates.DateTime(now())
                # gait_time = Dates.value.((current_t-start_t))/1000.0

                current_t = get_rostime()
                dt = current_t - start_t
                gait_time = to_sec(dt)


                for i=1:4
                    if gait_time > leg_gait_phase[i][1]*gait_total_time && gait_time < leg_gait_phase[i][2]*gait_total_time
                        foot_contact_schedule[i] = 0
                    else
                        foot_contact_schedule[i] = 1
                    end
                end
                foot_contact = foot_contact_schedule
            end


            for i=1:4
                if fbk_state.footForce[i] > 0.0
                    foot_contact_fbk[i] = 1
                else
                    foot_contact_fbk[i] = 0
                end
                # early contact
                if gait_time > (leg_gait_phase[i][1]+leg_gait_phase[i][2])/2*gait_total_time && gait_time <= leg_gait_phase[i][2]*gait_total_time && foot_contact_fbk[i] == 1
                    foot_contact[i] =  foot_contact_fbk[i]  
                end 
            end
            # debug_point.x = foot_contact[1]
            # debug_point.y = foot_contact_schedule[1]
            # debug_point.w = foot_contact_fbk[1]

            # publish(debug_pub, debug_point)

            # get target body movement
            ref_body_vel = Array(tgt_vel)
            ref_base_position += ctrl_dt*ref_body_vel
            # trigger swing trajectory generation
            for i=1:4
                leg_ID = i-1
                if foot_contact_prev[i] == 1 && foot_contact[i] == 0
                    # foot pos are in body frame 
                    pb = A1Robot.fk(leg_ID,q_list[:,i])
                    swing_cur_foot_pos[:,i] = pb
                    # convert nominal pose to world frame  
                    p_tgt_e = p_eb + q_eb*nominal_foot_pos_b[:,i]
                    
                    contact_time = abs(leg_gait_phase[i][1]*gait_total_time) +  (1- leg_gait_phase[i][2])*gait_total_time
                    # p_tgt_e = SA[pe[1],pe[2],nominal_foot_pos_e[3,i]] # use initial foot height
                    p_tgt_e += contact_time/2*ref_body_vel # raibert heuristic
                    p_tgt_e += sqrt(0.33/9.81)*(v_eb-ref_body_vel) # capture point
                    swing_tgt_foot_pos_e[:,i] = p_tgt_e
                    p_tgt_b = conj(q_eb)*(p_tgt_e - p_eb)
                    # swing_tgt_foot_pos[:,i] = nominal_foot_pos_b[:,i]
                    swing_tgt_foot_pos[:,i] = p_tgt_b

                    swing_mid_foot_pos[:,i] = (swing_cur_foot_pos[:,i]+swing_tgt_foot_pos[:,i])/2 + [0.0;0.0;0.15]
                    swing_traj_time[1,i] = leg_gait_phase[i][1]*gait_total_time 
                    swing_traj_time[3,i] = leg_gait_phase[i][2]*gait_total_time 
                    swing_traj_time[2,i] = (swing_traj_time[1,i]+ swing_traj_time[3,i])/2
                    # show(stdout, "text/plain", swing_cur_foot_pos)
                    # show(stdout, "text/plain", swing_mid_foot_pos)
                    # show(stdout, "text/plain", swing_tgt_foot_pos)
                    # show(stdout, "text/plain", swing_traj_time)
                    # if i == 1
                    #     debug_point.z = 1
                    # end
                else
                    # if i == 1
                    #     debug_point.z = 0
                    # end
                end
                foot_contact_prev[i] = foot_contact[i]
            end
            # the reference base position is adjusted according to current stance foot and predicted swing foot location
            
            # averge_pos = zeros(3)
            # for i=1:4
            #     leg_ID = i-1
            #     if foot_contact[i] == 0
            #         averge_pos[1] += swing_tgt_foot_pos_e[1,i]
            #         averge_pos[2] += swing_tgt_foot_pos_e[2,i]
            #         averge_pos[3] += swing_tgt_foot_pos_e[3,i]
            #     else
            #         pb = A1Robot.fk(leg_ID,q_list[:,i])
            #         # convert foot pose to world frame  
            #         pe = p_eb + q_eb*pb
            #         averge_pos[1] += pe[1]
            #         averge_pos[2] += pe[2]
            #         averge_pos[3] += pe[3]
            #     end
            # end

            # ref_base_position[1] = 0.9*ref_base_position[1] + 0.1*averge_pos[1]/4
            # ref_base_position[2] = 0.9*ref_base_position[2] + 0.1*averge_pos[2]/4

            # 1. calculate QP foot force 
            # control foot according to gait schedule
            # q_tgt = UnitQuaternion(1.0,0.0,0.0,0.0) # TODO: unit first
            q_tgt = q_eb0
            q_err = q_tgt*conj(q_eb)  # Rd*R^T

            w_b = fbk_state.imu.gyroscope
            w_e = q_eb*w_b

            Kp = diagm([210;130;100])
            Kd = diagm([26;15;30])
            Inertia = diagm([0.1;0.1;0.1])
            Ie = q_eb*Inertia*conj(q_eb)
            q_err_log = log(q_err)
            q_err_vec = @SVector[q_err_log.x,q_err_log.y,q_err_log.z]
            e_w = Kp*q_err_vec + Kd*(-w_e)
            tgt_wa = Ie*e_w
            # tgt_wa = max.( min.(tgt_wa,pi), -pi)

            # construct QP
            Kp_a = diagm([60.0;60.0;30.0])
            Kd_a = diagm([10.0;10.0;10.0])
            tgt_a = Kp_a*(ref_base_position-p_eb)+Kd_a*(ref_body_vel-v_eb)
            ma = mass*tgt_a
            # ma = max.( min.(ma,4.9), -4.9)
            b = vcat(ma+mg,tgt_wa)

            # debug_point.x = q_err_vec[1]
            # debug_point.y = p_eb[1]
            # debug_point.z = q_err_vec[2]
            # debug_point.w = p_eb[2]


            pR_list = []
            for i=1:4
                leg_ID = i - 1
                
                p = A1Robot.fk(leg_ID, q_list[:,i])
                p = convert(SVector{3},p)
                pR = skew(q_eb*p) # this q convert distance to world franme
                push!(pR_list, pR)
            end

            pRmtx = []
            Imtx = []
            for i=1:4
                    if isempty(pRmtx)
                        pRmtx = pR_list[i]
                        Imtx = I3
                    else
                        pRmtx = hcat(pRmtx, pR_list[i])
                        Imtx = hcat(Imtx,I3)
                    end
            end
            A = vcat(Imtx,pRmtx)  # always 6x12

            # D confines the foot force
            D = sparse(I(12))
            ld = fill(-Inf,12)
            ud = fill(Inf,12)
            if sum(foot_contact) >= 2  # assume <2 never happens
                for i=1:4
                    if foot_contact[i] == 0  # use real contact
                        ld[(i-1)*3+1:i*3] .= 0
                        ud[(i-1)*3+1:i*3] .= 0
                    end
                end
            else
                break # robot must fall
            end
            
            P = A'*S*A + alpha_mtx + beta_mtx
            q = -(A'*S*b + 2*beta_mtx*F_prev)
            model = OSQP.Model()
            OSQP.setup!(model, P=sparse(P), q=Vector(q), A=sparse([D; C]), l=[ld; fill(-Inf,20)], u=[ud; zeros(20)],
                eps_abs=1e-6, eps_rel=1e-6, verbose=false)
            res = OSQP.solve!(model)

            F = reshape(res.x,3,4)

            debug_point2.x = F[3,1]
            debug_point2.y = F[3,2]
            debug_point2.z = F[3,3]
            debug_point2.w = F[3,4]

            # publish(debug_pub, debug_point)

            # show(stdout, "text/plain", ref_p_list)
            F_prev = res.x

            # 2. control swing trajectory       
            tau .= 0.0

            for i=1:4
                leg_ID = i-1
                if foot_contact[i] == 0 
                    # we assume if foot contact is 0 there is a generated trajectory
                    ref_p_list = [swing_cur_foot_pos[:,i] swing_mid_foot_pos[:,i] swing_tgt_foot_pos[:,i]]
                    ref_t_list = swing_traj_time[:,i]
                    ref_p, ref_v, ref_a = hermite_cubic_traj(ref_p_list, ref_t_list, gait_time)

                    icap = 700
                    # pb = A1Robot.fk(leg_ID,q_list[:,i])
                    ref_q = A1Robot.ik(leg_ID, ref_p, q_list[:,i])
                    # ref_q = [0.0;0.9;-2.0]
                    # ref_v = zeros(3)
                    # ref_a = zeros(3)

                    error_pos_foot[:,i] += (ref_q-q_list[:,i])*ctrl_dt 
                    error_pos_foot = max.(min.(error_pos_foot,icap),-icap)
                    error_pos_foot .*= 0.8
                    tau_swing[:,i] = A1Robot.swing_torque_gazebo_joint(leg_ID,ref_q,ref_v,ref_a,q_list[:,i],dq_list[:,i],error_pos_foot[:,i], Kp_swing,Ki_swing,Kd_swing)

                    # error_pos_foot[:,i] += (ref_p-pb)*ctrl_dt 
                    # error_pos_foot = max.(min.(error_pos_foot,icap),-icap)
                    # error_pos_foot .*= 0.8
                    # tau_swing[:,i] = A1Robot.swing_torque_gazebo(leg_ID,ref_p,ref_v,ref_a,q_list[:,i],dq_list[:,i],error_pos_foot[:,i], Kp_swing,Ki_swing,Kd_swing)
                    tau[:,i] = tau_swing[:,i]
                    # debug tracking performance
                    # Jb = A1Robot.J(leg_ID, q_list[:,i])
                    # v = Jb*dq_list[:,i]
                    if i == 1
                        debug_point.x = ref_q[1]
                        debug_point.y = q_list[1,i]
                        debug_point.z = ref_q[2]
                        debug_point.w = q_list[2,i]


                        # debug_point3.x = tau_swing[1,i]
                        # debug_point3.y = tau_swing[2,i]
                        # debug_point3.z = tau_swing[3,i]
                    end
                else
                    foot_F = conj(q_eb)*(-F[:,i]) # foot push into ground
                    tau_stance[:,i] = A1Robot.stance_torque_ctrl(leg_ID, q_list[:,i], Array(foot_F))
                    tau[:,i] = tau_stance[:,i]
                    # foot_F = zeros(3) # foot push into ground
                    # ref_p = nominal_foot_pos[:,i]
                    # ref_v = zeros(3) 
                    # ref_a = zeros(3)
                    # tau_stance[:,i] = A1Robot.swing_torque_ctrl(leg_ID,ref_p,ref_v,ref_a,q_list[:,i],dq_list[:,i],Array(foot_F),Kp,Kd)
                    # tau = tau_stance[:,i]
                    # # debug tracking performance
                    # pb = A1Robot.fk(leg_ID,q_list[:,i])
                    # if i == 1
                    #     debug_point.x = norm(pb-ref_p)
                    # end
                    error_pos_foot[:,i]  .= 0.0
                end
                
                # if i==1
                sim_setCmdMotorTau(leg_ID*3, Float32(tau[1,i]),simcmd_list)
                sim_setCmdMotorTau(leg_ID*3+1, Float32(tau[2,i]),simcmd_list)
                sim_setCmdMotorTau(leg_ID*3+2, Float32(tau[3,i]),simcmd_list)
                # end
            end
            debug_point3.w = foot_contact[1]


            debug_point3.x = v_eb[1]
            debug_point3.y = v_eb[2]
            debug_point3.z = v_eb[3]
            sim_SendCommand(simcmd_list, sim_pub_list)


            # debug the gait scheduler
            # debug_point.x = foot_contact_schedule[1]
            # debug_point.y = foot_contact_schedule[2]
            # debug_point.z = foot_contact_schedule[3]
            # debug_point.w = foot_contact_schedule[4]

            # publish(debug_pub, debug_point)

            # debug_point2.x = gait_time
            # debug_point2.y = gait_is_running
            # debug_point2.z = norm(tgt_vel)

            publish(debug_pub, debug_point)
            publish(debug_pub2, debug_point2)
            publish(debug_pub3, debug_point3)
            
        
        elseif control_state == 2
            """ just test swing leg
                robot need to be hang up using "unitree_move_kinetic" 
            """ 
            #track ref_foot_pos_e
            tgt_y = joy_data.axes[4]*0.1
            tgt_x = joy_data.axes[5]*0.1
            tgt_z = joy_data.axes[2]*0.1
            ref_foot_pos_e = nominal_foot_pos_e + repeat([tgt_x;tgt_y;tgt_z],1,4)

            i = 1
            leg_ID = 0  # just control one leg
            tau = zeros(3)


            current_t = get_rostime()
            dt = current_t - start_t
            gait_time = to_sec(dt)
            if gait_time >= gait_circle
                start_t = get_rostime()
            end

            # ref_p = Array(conj(q_eb)*(ref_foot_pos_e[:,i] - p_eb))
            # ref_v = zeros(3)
            # ref_a = zeros(3)
            # println(gait_time)
            ref_p, ref_v, ref_a = hermite_cubic_traj(ref_p_list, ref_t_list, gait_time)
        

            pb = A1Robot.fk(leg_ID,q_list[:,i])

            debug_point.x = ref_p[1]
            debug_point.y = ref_p[2]
            debug_point.z = ref_p[3]
            debug_point.w = 0.0

            debug_point2.x = pb[1]
            debug_point2.y = pb[2]
            debug_point2.z = pb[3]
            debug_point2.w = 0.0

            icap = 60
            error_pos_foot[:,i] += (ref_p-pb)*ctrl_dt # TODO:anti-windup

            error_pos_foot = max.(min.(error_pos_foot,icap),-icap)
            tau = A1Robot.swing_torque_gazebo(leg_ID,ref_p,ref_v,ref_a,q_list[:,i],dq_list[:,i],error_pos_foot[:,i])
            # Kp = diagm([2200.0;2200.0;2200.0])
            # Ki = diagm([88.0;88.0;88.0])
            # icap = 60
            # Kd = diagm([500.0;500.0;500.0])
            # q = q_list[:,i]
            # dq = dq_list[:,i]
            # # tau = A1Robot.swing_torque_ctrl(leg_ID,ref_p,ref_v,ref_a,q_list[:,i],dq_list[:,i],zeros(3),Kp,Kd)
            # # task space 
            # Jb = A1Robot.J(leg_ID, q)
            # Jinv = inv(Jb)
            # dJb = A1Robot.dJ(leg_ID, q, dq)
            # v = Jb*dq
            # M = A1Robot.getMassMtx(leg_ID,q)
            # h = A1Robot.getVelQuadraticForces(leg_ID,q,dq) + A1Robot.getGravityForces(leg_ID,q)
            
            # # modern robotics 8.90, 8.91
            # Lambda = Jinv'*M*Jinv
            # eta = Jinv'*h - Lambda*dJb*dq
            # # modern robotics 11.47
            # error_pos_foot[:,i] += (ref_p-pb)*ctrl_dt # TODO:anti-windup

            # error_pos_foot = max.(min.(error_pos_foot,icap),-icap)
            
            # tau = Jb'*(Lambda*(ref_a + Kp*(ref_p-pb)+ Ki*error_pos_foot[:,i] +Kd *(ref_v-v) ) + eta ) 

            
            sim_setCmdMotorTau(leg_ID*3, Float32(tau[1]),simcmd_list)
            sim_setCmdMotorTau(leg_ID*3+1, Float32(tau[2]),simcmd_list)
            sim_setCmdMotorTau(leg_ID*3+2, Float32(tau[3]),simcmd_list)

            sim_SendCommand(simcmd_list, sim_pub_list)

            publish(debug_pub, debug_point)
            publish(debug_pub2, debug_point2)
            publish(debug_pub3, debug_point3)
             
        end

        
        rossleep(loop_rate)
        
    end
end


if ! isinteractive()
    main()
end
