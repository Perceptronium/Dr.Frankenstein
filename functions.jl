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

#Calcul de la loi trapeze
function CalculeTrapeze(robot, qi, qf, duree)

        # Init vecteurs
        t1 = zeros(6,1);
        t2 = zeros(6,1);
        tf = zeros(6,1);

        # On calcule les t1 de chaque articulation
        for i = 1:6
        t1[i] = robot[i,3]/robot[i,4]; # t1 pour chaque articulation
        end
        t1[:] .= maximum(t1); # On garde que le plus lent, format vecteur

        # tf pour chaque articulation
        delta_q = qf-qi;
        for i = 1:6
        tf[i] = (delta_q[i]/robot[i,3]) + t1[i];
        end
        unit_tf = maximum(tf);
        if duree > unit_tf
                unit_tf = duree; # We can force a longer time on plateau
        end
        tf[:] .= unit_tf; # Formatting to put it in the Param matrix

        # t2 pour chaque articulation
        t2 = tf - t1; # Soit imposé par une duree, soit on garde le plus lent

        return Param = [qi, qf, t1, t2, tf];
end

# S'aligner sur l'articulation la plus contraignante
# Prendre potentiellement des valeurs inferieures aux q_dot

# Testing
qi = zeros(6,1);
qf = [pi;pi/2;pi/4;pi;0;pi/2];
duree = 10;
Param = CalculeTrapeze(robot, qi, qf, duree);

function calculeQ(robot, Param, t)

        # Init les vecteurs
        q = zeros(6,1);
        qd = zeros(6,1);
        qdd = zeros(6,1);

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

t = 0:10;
q1_v = zeros(10);
q2_v = zeros(10);
q3_v = zeros(10);
for i in range(1, length = 10)
        q,qd,qdd = calculeQ(robot, Param, t[i])
        q1_v[i] = qd[1];
        q2_v[i] = qd[2];
        q3_v[i] = qd[3];
end

plot()
plot!(t[1:10],q1_v[1:10])
plot!(t[1:10],q2_v[1:10])
plot!(t[1:10],q3_v[1:10])

function CommandeCinematique(robot, qi, qf, duree)

        # GENERATION DE TRAJECTOIRES : ESPACE ART
        Param = CalculeTrapeze(robot, qi, qf, duree);
        q_des,dq_des,d2q_des = calculeQ(robot, Param, t[i]);

        # ON DOIT PASSER A L'ESPACE OP
        X_des = MGD(q_des);
        dX_des = Jacobian()*dq_des;

        # ON RECUPERE POSITION ET ORIENTATION



end
