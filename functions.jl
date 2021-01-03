#######################################################
# Rotation matrixes
#######################################################
function rot_z(psi)
    return [cos(psi) -sin(psi) 0 0;
            sin(psi) cos(psi) 0 0;
            0 0 1 0;
            0 0 0 1];
end

function rot_x(fi)
    return [1 0 0 0;
            0 cos(fi) -sin(fi) 0;
            0 sin(fi) cos(fi) 0;
            0 0 0 1];
end

function rot_y(theta)
    return [cos(theta) 0 sin(theta) 0;
            0 1 0 0;
            -sin(theta) 0 cos(theta) 0;
            0 0 0 1];
end

#######################################################
# Translation matrixes
#######################################################
function trans_x(a)
        return [1 0 0 a;
                0 1 0 0;
                0 0 1 0;
                0 0 0 1];
        end
function trans_y(b)
        return [1 0 0 0;
                0 1 0 b;
                0 0 1 0;
                0 0 0 1];
        end

function trans_z(c)
        return [1 0 0 0;
                0 1 0 0;
                0 0 1 c;
                0 0 0 1];
        end
#######################################################
# Draw a frame
#######################################################

function draw_frame(R)
                x1 = R[1,4];
                x2 = x1 + R[1,1]*100;
                y1 = R[2,4];
                y2 = y1 + R[2,1]*100;
                z1 = R[3,4];
                z2 = z1 + R[3,1]*100;
                plot!([x1;x2], [y1;y2], [z1;z2], linecolor =:red)

                x3 = x1 + R[1,2]*100;
                y3 = y1 + R[2,2]*100;
                z3 = z1 + R[3,2]*100;
                plot!([x1;x3], [y1;y3], [z1;z3], linecolor =:green)

                x4 = x1 + R[1,3]*100;
                y4 = y1 + R[2,3]*100;
                z4 = z1 + R[3,3]*100;
                plot!([x1;x4], [y1;y4], [z1;z4], linecolor =:blue)
end

#######################################################
# Trapezoid law
#######################################################

# Valeurs limites des articulations du robot
         #q     #q    #qdot   #qsec  #Cmax
robot = [-pi    pi     3.3    30    44.2;
         -pi/2  pi/2   3.3    30    44.2;
         -pi/2  3*pi/4 3.3    30    21.142;
         -pi    pi     3.3    30    21.142;
         -pi/2  pi/2   3.2    30    6.3;
         -pi    pi     3.2    30    6.3];

# #Calcul de la loi trapeze
# function CalculeTrapeze(robot, qi, qf, duree)
#
#         # Init vecteurs
#         t1 = zeros(6,1);
#         t2 = zeros(6,1);
#         tf = zeros(6,1);
#
#         # On calcule les t1 de chaque articulation
#         for i = 1:6
#         t1[i] = robot[i,3]/robot[i,4]; # t1 pour chaque articulation
#         end
#         t1[:] .= maximum(t1); # On garde que le plus lent, format vecteur
#
#         # tf pour chaque articulation
#         delta_q = qf-qi;
#         for i = 1:6
#         tf[i] = (delta_q[i]/robot[i,3]) + t1[i];
#         end
#         unit_tf = maximum(tf);
#         if duree > unit_tf
#                 unit_tf = duree; # We can force a longer time on plateau
#         end
#         tf[:] .= unit_tf; # Formatting to put it in the Param matrix
#
#         # t2 pour chaque articulation
#         t2 = tf - t1; # Soit imposé par une duree, soit on garde le plus lent
#
#         return Param = [qi, qf, t1, t2, tf];
# end

# S'aligner sur l'articulation la plus contraignante
# Prendre potentiellement des valeurs inferieures aux q_dot

#Calcul de la loi trapeze
function CalculeTrapeze(robot, qi, qf, duree)

        # Init vecteurs
        t1 = zeros(6,1);
        t2 = zeros(6,1);
        tf = zeros(6,1);

        # Etape 1 : Calculer t1 de chaque articulation
        for i = 1:6
        t1[i] = robot[i,3]/robot[i,4]; # t1 pour chaque articulation
        end

        # Etape 2 : Calculer tf de chaque articulation
        delta_q = qf-qi;
        for i = 1:6
        tf[i] = (delta_q[i]/robot[i,3]) + t1[i];
        end

        # Etape 3 : Synchronisation des axes, on garde le tf maximum
        max_tf = maximum(tf);
        if duree > max_tf
                max_tf = duree; # On peut forcer un temps encore plus long sur le plateau
        end
        tf[:] .= max_tf; # Formatting to put it in the Param matrix

        # Etape 4 : Ajuster le profil de vitesse des articulations plus rapides
        for i = 1:6
        robot[i,3] = (robot[i,4]*tf[i] - sqrt(robot[i,4]*(robot[i,4]*tf[i]*tf[i] - 4*delta_q[i])))/2
        end

        # t2 pour chaque articulation
        t2 = tf - t1;

        return Param = [qi, qf, t1, t2, tf, robot];
end

# Testing
qi = zeros(6,1);
qf = [pi;pi/2;pi/4;pi;0;pi/2];
duree = 10;
Param = CalculeTrapeze(robot, qi, qf, duree);

function calculeQ(Param, t)

        # Init les vecteurs
        q = zeros(6,1);
        qd = zeros(6,1);
        qdd = zeros(6,1);
        robot = Param[6];

        if t < Param[3][1] && t >= 0; # Si 0 < t < t1
                for i = 1:6
                        qdd[i] = robot[i,4];
                        qd[i] = qdd[i]*t;
                        q[i] = 0.5*qdd[i]*t*t + Param[1][i];
                end
        elseif t >= Param[3][1] && t<= Param[4][1] # Si t1 < t < t2
                for i = 1:6
                        qdd[i] = 0;
                        qd[i] = robot[i,3];
                        q[i] = qd[i]*(t-Param[3][1]) + 0.5*robot[i,4]*Param[3][1]*Param[3][1] + Param[1][i];
                end
        elseif t >= Param[4][1] && t <= Param[5][1] # Si t2 < t < tf
                for i = 1:6
                        qdd[i] = -robot[i,4];
                        qd[i] = -(-qdd[i])*(t-Param[4][1])+robot[i,3];
                        q[i] = -0.5*qdd[i]*(t-Param[4][1])*(t-Param[4][1]) + robot[i,3]*(t-Param[4][1]) + robot[i,3]*(Param[4][1]-Param[3][1]) +
                        0.5*(-qdd[i])*Param[3][1]*Param[3][1] + Param[1][i];
                end
        end

        return q, qd, qdd;
end

t = 0:5;
q1_v = zeros(10);
q2_v = zeros(10);
q3_v = zeros(10);
q4_v = zeros(10);
q5_v = zeros(10);
q6_v = zeros(10);
for i in range(1, length = 5)
        q,qd,qdd = calculeQ(Param, t[i])
        q1_v[i] = qd[1];
        q2_v[i] = qd[2];
        q3_v[i] = qd[3];
        q4_v[i] = qd[4];
        q5_v[i] = qd[5];
        q6_v[i] = qd[6];
end

plot()
plot!(xlabel = "Calcul des vitesses", xlims = (0,12))
plot!(t[1:5],q1_v[1:5])
plot!(t[1:5],q2_v[1:5])
plot!(t[1:5],q3_v[1:5])
plot!(t[1:5],q4_v[1:5])
plot!(t[1:5],q5_v[1:5])
plot!(t[1:5],q6_v[1:5])


function MGD(theta)

        # Matrices de transformation
        T_0_1 = rot_x(alpha[1])*trans_x(d[1])*rot_z(theta[1])*trans_z(r[1]);
        T_1_2 = rot_x(alpha[2])*trans_x(d[2])*rot_z(theta[2])*trans_z(r[2]);
        T_2_3 = rot_x(alpha[3])*trans_x(d[3])*rot_z(theta[3])*trans_z(r[3]);
        T_3_4 = rot_x(alpha[4])*trans_x(d[4])*rot_z(theta[4])*trans_z(r[4]);
        T_4_5 = rot_x(alpha[5])*trans_x(d[5])*rot_z(theta[5])*trans_z(r[5]);
        T_5_6 = rot_x(alpha[6])*trans_x(d[6])*rot_z(theta[6])*trans_z(r[6]);

        # Calcul en chaine des reperes de chaque articulation
        R_1 = R_0*T_0_1;
        R_2 = R_1*T_1_2;
        R_3 = R_2*T_2_3;
        R_4 = R_3*T_3_4;
        R_5 = R_4*T_4_5;
        MGD = R_5*T_5_6;

        return MGD      # dim 4x4
end

function geoJacobian(q_des)

        P = MGD(q_des)[1:3,4];
        T_0_1 = rot_x(alpha[1])*trans_x(d[1])*rot_z(q_des[1])*trans_z(r[1]);
        T_1_2 = rot_x(alpha[2])*trans_x(d[2])*rot_z(q_des[2])*trans_z(r[2]);
        T_2_3 = rot_x(alpha[3])*trans_x(d[3])*rot_z(q_des[3])*trans_z(r[3]);
        T_3_4 = rot_x(alpha[4])*trans_x(d[4])*rot_z(q_des[4])*trans_z(r[4]);
        T_4_5 = rot_x(alpha[5])*trans_x(d[5])*rot_z(q_des[5])*trans_z(r[5]);
        T_5_6 = rot_x(alpha[6])*trans_x(d[6])*rot_z(q_des[6])*trans_z(r[6]);

        T_0_2 = T_0_1*T_1_2;
        T_0_3 = T_0_2*T_2_3;
        T_0_4 = T_0_3*T_3_4;
        T_0_5 = T_0_4*T_4_5;
        T_0_6 = T_0_5*T_5_6;

        J1 = [cross(T_0_1[1:3,3], P - T_0_1[1:3,4]);
              T_0_1[1:3,3]]
        J2 = [cross(T_0_2[1:3,3], P - T_0_2[1:3,4]);
              T_0_2[1:3,3]]
        J3 = [cross(T_0_3[1:3,3], P - T_0_3[1:3,4]);
              T_0_3[1:3,3]]
        J4 = [cross(T_0_4[1:3,3], P - T_0_4[1:3,4]);
              T_0_4[1:3,3]]
        J5 = [cross(T_0_5[1:3,3], P - T_0_5[1:3,4]);
              T_0_5[1:3,3]]
        J6 = [cross(T_0_6[1:3,3], P - T_0_6[1:3,4]);
              T_0_6[1:3,3]]
        J = [J1 J2 J3 J4 J5 J6];

        return J
end

# Transforme un vecteur en son pre-produit matriciel
function toSkew(a)

        a = [0 -a[3] a[2];
             a[3] 0 -a[1];
             -a[2] a[1] 0];
        return a;

end

function calculeCinematique(robot, qi, qf, duree)

        # Generation de trajectoires
        Param = CalculeTrapeze(robot, qi, qf, duree);
        q_des,dq_des,d2q_des = calculeQ(Param, t);

        # Erreur de suivi en position
        Pe = MGD(theta');
        Pe = Pe[1:3,4];
        Pd = MGD(q_des);
        Pd = Pd[1:3,4];
        eps_p = Pd - Pe; # dim 3x1

        # Erreur de suivi en orientation
        Ae = MGD(theta);
        Ae = Ae[1:3,1:3];
        Ad = MGD(q_des);
        Ad = Ad[1:3,1:3];
        A = Ad*Ae';
        eps_o = 0.5*[A[3,2]-A[2,3];
                     A[1,3]-A[3,1];
                     A[2,1]-A[1,2]] # dim 3x1

        # Calcul de la jacobienne geometrique
        J = geoJacobian(q_des);
        dX = J*dq_des;

        # Calcul de dPd
        dPd = dX[1:3];

        # Calcul de wd
        wd = dX[4:6];

        # Declaration de Kp et Ko
        Kp = ones(3,3);
        Ko = ones(3,3);

        # Calcul de la vitesse lineaire a donner
        dPe = dPd + Kp*eps_p;

        # Calcul de L
        L = -0.5*(toSkew(Ad[1:3,1])*toSkew(Ae[1:3,1])+toSkew(Ad[1:3,2])*toSkew(Ae[1:3,2])+toSkew(Ad[1:3,3])*toSkew(Ae[1:3,3]))

        # Calcul de la vitesse angulaire a donner
        we = pinv(L)*(L'*wd + Ko*eps_o);

        # Loi de commande articulaire en vitesse
        dqC = pinv(J)*[dPe;we];

        return dqC;
end
