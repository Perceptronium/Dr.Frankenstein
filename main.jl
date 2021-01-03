include("functions.jl")
include("defines.jl")
import Pkg
Pkg.add("Plots")
using Plots
using LinearAlgebra

# This will be our world reference
R_0 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

# Let's declare transformation matrixes
theta = declare_theta(0, 0, 0, 0, 0, 0);
T_0_1 = rot_x(alpha[1])*trans_x(d[1])*rot_z(theta[1])*trans_z(r[1]);
T_1_2 = rot_x(alpha[2])*trans_x(d[2])*rot_z(theta[2])*trans_z(r[2]);
T_2_3 = rot_x(alpha[3])*trans_x(d[3])*rot_z(theta[3])*trans_z(r[3]);
T_3_4 = rot_x(alpha[4])*trans_x(d[4])*rot_z(theta[4])*trans_z(r[4]);
T_4_5 = rot_x(alpha[5])*trans_x(d[5])*rot_z(theta[5])*trans_z(r[5]);
T_5_6 = rot_x(alpha[6])*trans_x(d[6])*rot_z(theta[6])*trans_z(r[6]);

# Let's now create our frames

R_1 = R_0*T_0_1;
R_2 = R_1*T_1_2;
R_3 = R_2*T_2_3;
R_4 = R_3*T_3_4;
R_5 = R_4*T_4_5;
MGD = R_5*T_5_6;

gr()

plt = plot([-10 700], [-10 700], [-10 700],
           xlabel = "x",
           ylabel = "y",
           zlabel = "z",
           )

draw_frame(R_0)
draw_frame(R_1)
draw_frame(R_2)
draw_frame(R_3)
draw_frame(R_4)
draw_frame(R_5)
draw_frame(MGD)
