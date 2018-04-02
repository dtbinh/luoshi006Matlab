% script for quadrotor simulation.
% by luoshi006

close all
clear all
clc
addpath('utils')

time_sim = 0.2;
time_step = 0.01;
t = 0:time_step:time_sim;

% parameters of uav;
params = paramLoader();
fprintf('Initializing params...\n')
quads  = stateInit();

% calc
F = [0.0751, 0.075, 0.075, 0.075];
for i = 1:size(t,2)
    quads = quad_dynamic(params,quads,i,F,time_step);
    
end

% plot
subplot(3,1,1)
plot(quads.px)
subplot(3,1,2)
plot(quads.py)
subplot(3,1,3)
plot(quads.pz)