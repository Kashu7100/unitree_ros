include("julia_robot_interface.jl")
using ForwardDiff

q = [0.0;0.1;1.3]
Jb=  A1Robot.J(1,q)

g(x) = A1Robot.fk(1,x)

G = ForwardDiff.jacobian(g, q)


xd = [0.0;0.0;-0.3]

q1 = A1Robot.ik(1, xd, q)