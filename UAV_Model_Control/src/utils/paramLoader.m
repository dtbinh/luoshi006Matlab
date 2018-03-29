function params = paramLoader()
% load parameters of quadrotor.
% copy from https://github.com/stormmax/quadrotor/blob/master/control/crazyflie.m

m = 0.030;  % weight (in kg) with 5 vicon markers (each is about 0.25g)
g = 9.81;   % gravitational constant
I = [1.43e-5,   0,          0; % inertial tensor in m^2 kg
     0,         1.43e-5,    0;
     0,         0,          2.89e-5];
L1 = 0.046; % arm length in m
L2 = 0.046; % arm length in m
L3 = 0.046; % arm length in m
L4 = 0.046; % arm length in m

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.grav = g;
params.arm_length1 = L1;
params.arm_length2 = L2;
params.arm_length3 = L3;
params.arm_length4 = L4;

params.maxangle = 40*pi/180; % you can specify the maximum commanded angle here
params.maxF     = 2.5*m*g;   % left these untouched from the nano plus
params.minF     = 0.05*m*g;  % left these untouched from the nano plus

params.dragX = 0; % drag coefficients.
params.dragY = 0;
params.dragZ = 0;

end