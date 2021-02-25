
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

# quternion to euler angle, notice how to get quaternion value 
using Rotations
using StaticArrays
using LinearAlgebra
function quat_to_euler(q::UnitQuaternion{Float64})
    cos_pitch_cos_yaw  = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    cos_pitch_sin_yaw  =     + 2.0 * (q.x*q.y + q.w*q.z)
	sin_pitch	       =     - 2.0 * (q.x*q.z - q.w*q.y) 
	cos_pitch		   = 0.0
	sin_roll_cos_pitch =     + 2.0 * (q.y*q.z + q.w*q.x)   
	cos_roll_cos_pitch = 1.0 - 2.0 * (q.x*q.x + q.y*q.y)   


	cos_pitch = sqrt(cos_pitch_cos_yaw*cos_pitch_cos_yaw + cos_pitch_sin_yaw*cos_pitch_sin_yaw);

	yaw   = atan(cos_pitch_sin_yaw, cos_pitch_cos_yaw)
	pitch = atan(sin_pitch, cos_pitch)
    roll  = atan(sin_roll_cos_pitch, cos_roll_cos_pitch)
    return yaw, roll, pitch
end

function get_yaw_angle_from_quat(q::UnitQuaternion{Float64})
    cos_pitch_cos_yaw  = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    cos_pitch_sin_yaw  =     + 2.0 * (q.x*q.y + q.w*q.z)
	yaw   = atan(cos_pitch_sin_yaw, cos_pitch_cos_yaw)
	return yaw
end

# Simultaneous Orthogonal Rotations Angle (SORA) angle to quaternion
function SORA_to_quat(theta_x::Float64, theta_y::Float64, theta_z::Float64)
	theta = sqrt(theta_x * theta_x + theta_y * theta_y + theta_z * theta_z)
    q = [0.0,0.0,0.0,0.0]
	if theta == 0.0
		q[1] = 1.0
		q[2] = 0.0
		q[3] = 0.0
		q[4] = 0.0
	else
		half_theta = theta * 0.5
		cos_half_theta = cos(half_theta);
		sin_half_theta = sin(half_theta);

		inv_norm = sin_half_theta / theta;

		q[1] = cos_half_theta;
		q[2] = theta_x * inv_norm;
		q[3] = theta_y * inv_norm;
        q[4] = theta_z * inv_norm;
    end
    return UnitQuaternion(q[1],q[2],q[3],q[4])
end

function axis_angle_to_quat(angle::Float64, axis::SVector{3})
    sin_half_ang = sin(angle*0.5)
    cos_half_ang = cos(angle*0.5)
    return UnitQuaternion(cos_half_ang,sin_half_ang*axis[1],sin_half_ang*axis[2],sin_half_ang*axis[3])
end

function sine_between_vector(v1::SVector{3}, v2::SVector{3})
    return norm(cross(v1,v2))/norm(v1)/norm(v2) 
end

function cosine_between_vector(v1::SVector{3}, v2::SVector{3})
    return dot(v1,v2)/norm(v1)/norm(v2) 
end

function angle_between_vector(v1::SVector{3}, v2::SVector{3})
    sine = sine_between_vector(v1,v2)
    cosine = cosine_between_vector(v1,v2)
    return atan(sine, cosine)
end

function get_tilt_quat_n_angles(q::UnitQuaternion{Float64})
    ref_vec = @SVector [0.0,0.0,1.0]
    body_vec = q[:,3]
    rotate_axis = cross(ref_vec, body_vec)
    rotate_ang = angle_between_vector(ref_vec, body_vec)

    theta_x = theta*rotate_axis[1]
    theta_y = theta*rotate_axis[2]
    q_tilt = axis_angle_to_quat(rotate_ang, rotate_axis)
    return q_tilt, theta_x, theta_y
end

function get_tilt_quat(q::UnitQuaternion{Float64})
    ref_vec = @SVector [0.0,0.0,1.0]
    body_vec = q[:,3]
    rotate_axis = normalize(cross(ref_vec, body_vec))
    rotate_ang = angle_between_vector(ref_vec, body_vec)

    q_tilt = axis_angle_to_quat(rotate_ang, rotate_axis)
    return q_tilt
end

function get_error_quat(q_cur::UnitQuaternion{Float64}, q_tgt::UnitQuaternion{Float64})
    q_err = q_tgt*conj(q_cur)
end

function quat_decompose_tilt_torsion(q::UnitQuaternion{Float64})
    q_tilt = get_tilt_quat(q)
    q_torsion = get_error_quat(q_tilt, q)
    return q_tilt, q_torsion
end

function skew(v::SVector{3})
    return @SMatrix[0 -v[3] v[2];v[3] 0 -v[1];-v[2] v[1] 0]
end