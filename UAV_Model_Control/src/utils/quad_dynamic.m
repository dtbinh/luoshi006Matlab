function quads = quad_dynamic(params,quads,i,F,time_step)
% quadrotor model from :
% Control of a Quadrotor Helicopter Using Visual Feedback
%  3      1
%    \  /
%    /  \
%  2      4
C = 0.1;


f1 = F(1);f2 = F(2);
f3 = F(3);f4 = F(4);

u1 = ( f1 + f2 + f3 + f4 )/params.mass;
u2 = (-f1 + f2 - f3 + f4 )/params.I(1,1);
u3 = (-f1 + f2 + f3 - f4 )/params.I(2,2);
u4 = C* ( f1 + f2 - f3 - f4 )/params.I(3,3);

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
quads.az(i+1) = An(3);

quads.vx(i+1) = Vn(1);
quads.vy(i+1) = Vn(2);
quads.vz(i+1) = Vn(3);

quads.px(i+1) = Pn(1);
quads.py(i+1) = Pn(2);
quads.pz(i+1) = Pn(3);

% debug TODO
quads.R(:,:,i+1) = quads.R(:,:,i);

end