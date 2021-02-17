
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