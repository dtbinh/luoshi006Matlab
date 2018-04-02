function quads = quad_dynamic(params,quads,i,F,time_step)
% quadrotor model from :
% Control of a Quadrotor Helicopter Using Visual Feedback
%  3      1
%    \  /
%    /  \
%  2      4
C = 0.01;


f1 = F(1);f2 = F(2);
f3 = F(3);f4 = F(4);

% arm length angle theta;
thetaL = 45/180*pi();
cT = cos(thetaL);
sT = sin(thetaL);

u1 = ( f1 + f2 + f3 + f4 )/params.mass;
u2 = (-f1*params.arm_length1*cT + f2*params.arm_length2*cT - f3*params.arm_length3*cT + f4*params.arm_length4*cT )/params.I(1);
u3 = (-f1*params.arm_length1*sT + f2*params.arm_length2*sT + f3*params.arm_length3*sT - f4*params.arm_length4*sT )/params.I(2);
u4 = C* ( f1 + f2 - f3 - f4 )/params.I(3);

An = quads.R(:,:,i) * [0;0;u1] - [ params.dragX * quads.vx(i);
                                   params.dragY * quads.vy(i);
                                   params.dragZ * quads.vz(i); ];
Vn = An * time_step + [quads.vx(i);
                       quads.vy(i);
                       quads.vz(i); ];
Pn = Vn * time_step + 0.5 * An * time_step^2 + [quads.px(i);
                                                quads.py(i);
                                                quads.pz(i); ];
quads.ax(i+1) = An(1);
quads.ay(i+1) = An(2);
quads.az(i+1) = An(3) - params.grav;

quads.vx(i+1) = Vn(1);
quads.vy(i+1) = Vn(2);
quads.vz(i+1) = Vn(3);

quads.px(i+1) = Pn(1);
quads.py(i+1) = Pn(2);
quads.pz(i+1) = Pn(3);

% debug TODO
% quads.R(:,:,i+1) = quads.R(:,:,i);

% angle update
Atheta = [u2; u3; u4];
quads.omega(:,i+1) = quads.omega(:,i) + Atheta * time_step;

S = [1 -quads.omega(3,i+1) quads.omega(2,i+1);
    quads.omega(3,i+1) 1 -quads.omega(1,i+1);
    -quads.omega(2,i+1) quads.omega(1,i+1) 1];

quads.R(:,:,i+1) =S * quads.R(:,:,i);
[ez,ey,ex] = dcm2angle(quads.R(:,:,i+1),'ZYX');
% [ez,ey,ex]*180/pi()
% disp(quads.ax(i+1))
% disp(quads.ay(i+1))
% disp(quads.az(i+1))
% disp(quads.vx(i+1))
% disp(quads.vy(i+1))
% disp(quads.vz(i+1))
% disp(quads.px(i+1))
% disp(quads.py(i+1))
% disp(quads.pz(i+1))
end











