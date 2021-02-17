using RobotDynamics
using TrajectoryOptimization
using Altro
using StaticArrays, LinearAlgebra

const TO = TrajectoryOptimization
const RD = RobotDynamics

# Define model discretization
N = 101
tf = 5.
dt = tf/(N-1)

# Define the model struct with parameters
struct SimplifiedQuad <: AbstractModel
    # mass and inertia 
    m::Float64 
    inertia::Array{Float64,2}
      
    g::Float64
  
    
    function SimplifiedQuad(m::Float64, Inertia::Array{Float64,2}) 
      new(m, Inertia, 9.81)
    end 
end

Inertia = diagm([0.13, 0.17, 0.083])
SimplifiedQuad() = SimplifiedQuad(10.2, Inertia)

"""
State and control definition 
         1:3       4:6       7:10        11:13     14:16     17:19    20:22   23:25
State = [p           v         q          w_b        p1       p2       p3       p4]
    world position   |  orientation(R_eb)  |           position of foot in world
              world velocity        body angular velocity
"""

# four forces, each has 3 dim
RobotDynamics.control_dim(::SimplifiedQuad) = 12

# Define the 3D forces at the center of mass, in the world frame
function RobotDynamics.forces(model::SimplifiedQuad, x::StaticVector, u::StaticVector)
    # order of forces follow unitree definition FR, FL, RR, RL
    F_FR = @SVector [u[1], u[2], u[3]]
    F_FL = @SVector [u[4], u[5], u[6]]
    F_RR = @SVector [u[7], u[8], u[9]]
    F_RL = @SVector [u[10], u[11], u[12]]
    return F_FR + F_FL + F_RR + F_RL
end

# Define the 3D moments at the center of mass, in the body frame
function RobotDynamics.moments(model::SimplifiedQuad, x::StaticVector, u::StaticVector)
    
    # get orientation from state
    q = UnitQuaternion(x[7],x[8],x[9],x[10])

    # first assume each foot is at the nomimal stance
    r_FR_b =  @SVector [0.2, -0.15, -0.25]
    r_FL_b =  @SVector [0.2, 0.15, -0.25]
    r_RR_b =  @SVector [-0.2, -0.15, -0.25]
    r_RL_b =  @SVector [-0.2, 0.15, -0.25]

    # get forces in world frame 
    F_FR_w = @SVector [u[1], u[2], u[3]]
    F_FL_w = @SVector [u[4], u[5], u[6]]
    F_RR_w = @SVector [u[7], u[8], u[9]]
    F_RL_w = @SVector [u[10], u[11], u[12]]

    # r x f   in body frame 
    total_moment = cross(r_FR_b, q*F_FR_w) + cross(r_FL_b, q*F_FL_w) 
                 + cross(r_RR_b, q*F_RR_w) + cross(r_RL_b, q*F_RL_w) 

    return SVector{3,Float64}(total_moment)  # body frame
end

# Define the continuous dynamics
function RobotDynamics.dynamics(model::SimplifiedQuad, x::StaticVector, u::StaticVector)
    
end