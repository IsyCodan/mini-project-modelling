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

param.mu_t = param.rm^2*(((param.mu_l_x+param.mu_l_y)/8) + ((1.5*param.mu_r)/(param.rb^2)));
param.mu_del = param.rm^2*(((param.mu_l_x-param.mu_l_y)/4) + (param.mu_r/(param.rb^2)));
param.J = 0.5*param.m*param.rm^2 + param.Jm;

param.P = (2*param.mu_t + param.mu_del)/param.J;
param.Q = (2*param.mu_t - param.mu_del)/param.J;

param.Rn = 10; % nozzle elec resistance
param.Cn = 10e-3; % nozzle elec capacity
param.Ln = 25e-3; % nozzle inductivity

param.Rtn = 5; % nozzle temp resistance
param.Ctn = 0.25; % nozzle temp capacity
param.T_e = 20; % combination of environmental and filament temp

param.T_low = 25; % temperature when nozzle off
param.T_high = 230; % temperature when nozzle on
param.L_filament = 2; % length of filament on spool
param.L_wipe = 1e-2; % length of filament dispensed for wiping
param.dot_d_standard = 5e-2; % standard extrusion velocity

param.x_saved = 0; % reserved x-parameter for saving locations
param.y_saved = 0; % reserved y-parameter for saving locations
param.t_saved = 0; % reserved time parameter to return at stopped printing location
param.d_saved = 0; % reserved parameter for saving dispensed filament length
param.park_velocity = 5e-2; % parking velocity

%% Simulation
% build desired trajectory for x and y
T_ellipse = 5;
w_ellipse = 2*pi/T_ellipse;
x_center = 2e-1;
y_center = 1.5e-1;
r_a_ellipse = 1e-1;
r_b_ellipse = 7.5e-2;

desired_x = @(t) [x_center + r_a_ellipse*cos(w_ellipse*t);
                 -r_a_ellipse*w_ellipse*sin(w_ellipse*t);
                 -r_a_ellipse*w_ellipse^2*cos(w_ellipse*t);
                 r_a_ellipse*w_ellipse^3*sin(w_ellipse*t)];
desired_y = @(t) [y_center + r_b_ellipse*sin(w_ellipse*t);
             r_b_ellipse*w_ellipse*cos(w_ellipse*t);
             -r_b_ellipse*w_ellipse^2*sin(w_ellipse*t);
             -r_b_ellipse*w_ellipse^3*cos(w_ellipse*t)];

desired_x_0 = desired_x(0);
desired_y_0 = desired_y(0);

% initial state
x0 = [desired_x_0(1) desired_x_0(2) 0 desired_y_0(1) desired_y_0(2) 0 param.T_high sqrt(Rn/Rtn*(T_high-T_e)) 0 0]';
t_start = 0;
t_end = 70;

mode = 'print';
t_log = []; x_log = []; mode_log = [];

while t_start < t_end
    ode_func = @(t,x) sys(x,input_func(t,desired_x,desired_y,mode,param),param);
    event_func = @(t,x) guards(t,x,mode,param);
    
    opts = odeset('Events',event_func, 'MaxStep', 1e-2);
    [t_seq, x_seq, t_of_evt, x_of_event, idx_of_event]= ode45(ode_func, [t_start, t_end], x0, opts);
    
    t_log = [t_log; t_seq];
    x_log = [x_log; x_seq];
    mode_log = [mode_log; repmat(string(mode), length(t_seq), 1)];

    t_start = t_seq(end);
    x0 = x_seq(end, :)';

    if t_start >= t_end
        break; 
    end

    [mode, x0, param] = reset_n_transition(mode, x0, t_start, param);
end