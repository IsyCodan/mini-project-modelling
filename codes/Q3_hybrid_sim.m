%% Clean Up
clear all
close all
clc

%% Define Parameter struct
% all physical quantities are given in si units
param.rm = 0.01; % Motor radius
param.Km = 0.06; % Motor constant
param.Jm = 7e-6; % Motor Inertia
param.Lm = 0.005; % Motor induction
param.Rm = 3; % Motor resistor
param.mu_r = 0.02; % Rolling friction coefficient
param.rb = 0.005; % bearing radius
param.mu_l_x = 1; % linear bearing x damping
param.mu_l_y = 1; % linear bearing y damping
param.m = 0.5; % mass of print head

% collecting parameters as seen in report
param.mu_t = param.rm^2*(((param.mu_l_x+param.mu_l_y)/8) + ((1.5*param.mu_r)/(param.rb^2))); 
param.mu_del = param.rm^2*(((param.mu_l_x-param.mu_l_y)/4) + (param.mu_r/(param.rb^2))); 
param.J = 0.5*param.m*param.rm^2 + param.Jm;

param.P = (2*param.mu_t + param.mu_del)/param.J; % coefficient for x terms
param.Q = (2*param.mu_t - param.mu_del)/param.J; % coefficient for y terms

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
param.t_interrupt = 0; % reserved time parameter to save time of interruption
param.t_saved = 0; % reserved time parameter to continue trajectory at stopped printing location
param.d_saved = 0; % reserved parameter for saving dispensed filament length
param.park_velocity = 1e-1; % parking velocity

%% Simulation
% build desired trajectory for x and y
T_ellipse = 5; % period time for full ellipse
w_ellipse = 2*pi/T_ellipse; % angular velocity for ellipse
x_center = 2e-1; % x - coordinate of center
y_center = 1.5e-1; % y - coordinate of center
r_a_ellipse = 1e-1; % radius of ellipse in x
r_b_ellipse = 7.5e-2; % radius of ellipse in y

% anonymous functions for desired ellipse trajectory and the corresponding
% derivatives
desired_x = @(t) [x_center + r_a_ellipse*cos(w_ellipse*t);
                 -r_a_ellipse*w_ellipse*sin(w_ellipse*t);
                 -r_a_ellipse*w_ellipse^2*cos(w_ellipse*t);
                 r_a_ellipse*w_ellipse^3*sin(w_ellipse*t)];
desired_y = @(t) [y_center + r_b_ellipse*sin(w_ellipse*t);
             r_b_ellipse*w_ellipse*cos(w_ellipse*t);
             -r_b_ellipse*w_ellipse^2*sin(w_ellipse*t);
             -r_b_ellipse*w_ellipse^3*cos(w_ellipse*t)];

% initial values for trajectory
desired_x_0 = desired_x(0);
desired_y_0 = desired_y(0);
desired_i_x_0 = 1/(param.Km*param.rm)*(param.J*desired_x_0(3) + param.J*param.P*desired_x_0(2));
desired_i_y_0 = 1/(param.Km*param.rm)*(param.J*desired_y_0(3) + param.J*param.Q*desired_y_0(2));

% to have anonymous functions available
param.trajectory_x = desired_x;
param.trajectory_y = desired_y;

% initial state for simulation
x0 = [desired_x_0(1) desired_x_0(2) desired_i_x_0 desired_y_0(1) desired_y_0(2) desired_i_y_0 param.T_high sqrt(param.Rn/param.Rtn*(param.T_high-param.T_e)) sqrt(param.Rn/param.Rtn*(param.T_high-param.T_e))/param.Rn 0]';
t_start = 0; % start time
t_end = 70; % end time
mode = 'print'; % initial mode

% initialize logging matrices
t_log = []; x_log = []; mode_log = [];

% run until end time is reached
while t_start < t_end
    ode_func = @(t,x) sys(x,input_func(t,desired_x,desired_y,mode,param),param);
    event_func = @(t,x) guards(t,x,mode,param);
    
    % define event 'controlled' ode solver, which quits solving as soon as
    % guard is hit
    opts = odeset('Events',event_func, 'MaxStep', 1e-2);
    [t_seq, x_seq, t_of_evt, x_of_event, idx_of_event]= ode45(ode_func, [t_start, t_end], x0, opts);
    
    % log data
    t_log = [t_log; t_seq];
    x_log = [x_log; x_seq];
    mode_log = [mode_log; repmat(string(mode), length(t_seq), 1)];
    
    % update starting time and initial condition for next continuous simulation
    t_start = t_seq(end);
    x0 = x_seq(end, :)';

    % break condition to not overwrite x0 with reset map
    if t_start >= t_end
        break;
    end
    
    % reset and transition map
    [mode, x0, param] = reset_n_transition(mode, x0, t_start, param);
end

%% Plot section
% Settings
% Set plotting of axis lable to use latex interpreter
figure
set(0, 'DefaultLineLineWidth', 1.2);
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');
set(gcf,'renderer','Painters')
set(gca,'LooseInset',get(gca,'TightInset'))
close

% Get color def of plot for later
pl_colors = cell(8,1);
figure
hold on
grid on
for k=1:8
    ind = plot([0,1]+0.1*k, [0,1]);
    pl_colors{k} = ind.Color;
end
close

w = 650;
h = 325;

figure; % XY
plot(x_log(:,1)*1e2,x_log(:,4)*1e2)
xlabel('$x\,/\,$cm')
ylabel('$y\,/\,$cm')
axis equal

figure; % i_x
plot(t_log,x_log(:,3))
xlabel('$t\,/\,$s')
ylabel('$i_{\rm x}\,/\,$A')

figure; % i_y
plot(t_log,x_log(:,6))
xlabel('$t\,/\,$s')
ylabel('$i_{\rm y}\,/\,$A')

figure; % T_n
plot(t_log,x_log(:,7))
xlabel('$t\,/\,$s')
ylabel('$T_{\rm n}\,/\,^\circ$C')

figure; % u_c
plot(t_log,x_log(:,8))
xlabel('$t\,/\,$s')
ylabel('$u_{\rm c}\,/\,$V')

figure; % i_n
plot(t_log,x_log(:,9))
xlabel('$t\,/\,$s')
ylabel('$i_{\rm n}\,/\,$A')

figure; % d_p
plot(t_log,x_log(:,10))
xlabel('$t\,/\,$s')
ylabel('$d_{\rm p}\,/\,$m')