%% Clean Up
clear all
close all
clc

%% Define Parameter struct
param.rm = 0.01; %Motor radius
param.Km = 0.06; %Motor constant
param.Jm = 7e-6; %Motor Inertia
param.Lm = 0.005; %Motor induction
param.Rm = 3; %Motor resistor
param.mu_r = 0.02; %Rolling friction coefficient
param.rb = 0.005; %bearing radius
param.mu_l_x = 1; %linear bearing x damping
param.mu_l_y = 1; %linear bearing y damping
param.m = 0.5; %mass of print head

param.mu_t = rm^2*(((mu_l_x+mu_l_y)/8) + ((1.5*mu_r)/(rb^2)));
param.mu_del = rm^2*(((mu_l_x-mu_l_y)/4) + (mu_r/(rb^2)));
param.J = 0.5*m*rm^2 + Jm;

param.P = (2*mu_t + mu_del)/J;
param.Q = (2*mu_t - mu_del)/J;

param.Rn = 10; % nozzle elec resistance
param.Cn = 10e-3; % nozzle elec capacity
param.Ln = 25e-3; % nozzle inductivity

param.Rtn = 5; % nozzle temp resistance
param.Ctn = 0.25; % nozzle temp capacity
param.T_e = 30; % combination of environmental and filament temp

%% Simulation
% initial state
x0 = [0 0 0 0 0 0 0 0 0]';
t_span = [0 50];
t_start = t_span(1);
t_current = t_start;
t_end = t_span(2);

while t_current < t_end
    ode_func = @(t,x) sys(x,input_func(t,desired_x,desired_y,mode,param),param);
    %event_func = @(t,x) guards();
end