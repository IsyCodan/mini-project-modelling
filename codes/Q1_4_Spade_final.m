%Defining variables (Taken random sensible values)

rm = 0.01; %Motor radius
Km = 0.06; %Motor constant
Jm = 7e-6; %Motor Inertia
Lm = 0.005; %Motor induction
Rm = 3; %Motor resistor
mu_rad = 0.02; %Rolling friction coefficient
rb = 0.005; %bearing radius
bx = 1; %linear bearing x damping
by = 1; %linear bearing y damping
m = 0.5; %mass of print head

%Defining combinations of variables for ease of substitution in the matrices

mu_t = rm^2*(((bx+by)/8) + ((1.5*mu_rad)/(rb^2)));
mu_del = rm^2*(((bx-by)/4) + (mu_rad/(rb^2)));
J = 0.5*m*rm^2 + Jm;

P = (2*mu_t + mu_del)/J;
Q = (2*mu_t - mu_del)/J;


%State Space for x

A_x = [0 1 0; 0 -P (rm*Km)/J; 0 -(Km/(rm*Lm)) -(Rm/Lm)];
B_x = [0; 0; 1/Lm];
C_x = [1 0 0];
D_x = 0;

%State Space for y

A_y = [0 1 0; 0 -Q (rm*Km)/J; 0 -(Km/(rm*Lm)) -(Rm/Lm)];
B_y = [0; 0; 1/Lm];
C_y = [1 0 0];
D_y = 0;

%%---PLOTTING---%%

%%----SPADE-----%%

Tend = 70; %time to finish printing (faster means more input voltage)

xWay = [ ...
    0, -0.12, -0.30, -0.52, -0.72, -0.84, ...
    -0.86, -0.80, -0.67, -0.48, -0.30, -0.15, ...
    -0.12, -0.20, -0.35, -0.35,  0.35,  0.35, ...
    0.20,  0.12,  0.15,  0.30,  0.48,  0.67, ...
    0.80,  0.86,  0.84,  0.72,  0.52,  0.30, ...
    0.12,  0];

yWay = [ ...
    1.00,  0.83,  0.76,  0.84,  0.78,  0.60, ...
    0.38,  0.15, -0.02, -0.10, -0.06,  0.10, ...
    0.20, -0.15, -0.35, -0.65, -0.65, -0.35, ...
    -0.15,  0.20,  0.10, -0.06, -0.10, -0.02, ...
    0.15,  0.38,  0.60,  0.78,  0.84,  0.76, ...
    0.83,  1.00];

dxWay = diff(xWay);
dyWay = diff(yWay);

sWay = [0, cumsum(sqrt(dxWay.^2 + dyWay.^2))];

Ndense = 1000; %%Larger N means longer time to execute
sdense = linspace(0,sWay(end),Ndense);

x = spline(sWay, xWay, sdense);
y = spline(sWay, yWay, sdense);

dsFine = sqrt(diff(x).^2 + diff(y).^2);
sArc = [0, cumsum(dsFine)];

N = 4001;
sQuery = linspace(0, sArc(end), N+1);
sQuery(end) = [];             % Avoid duplicate endpoint

x_ref = interp1(sArc, x, sQuery, 'pchip');
y_ref = interp1(sArc, y, sQuery, 'pchip');

% Time vector
t = (0:N-1)*(Tend/N);
dt = Tend/N;

% Periodic finite-difference derivative operator
d_dt = @(q) (circshift(q,[0 -1]) ...
    - circshift(q,[0 1]))/(2*dt);

%derivative equations for trajectories

x_dot = d_dt(x_ref);
x_dot_dot = d_dt(x_dot);
x_dot_dot_dot = d_dt(x_dot_dot);

y_dot = d_dt(y_ref);
y_dot_dot = d_dt(y_dot);
y_dot_dot_dot = d_dt(y_dot_dot);

%Expressing input as a fn of derivatives of trajectories

x_coeff_1 = (J*Lm)/(rm*Km);
x_coeff_2 = (((Lm*J*P)/(Km*rm)) + ((J*Rm)/(Km*rm)));
x_coeff_3 = (Km/rm + (Rm*J*P)/(Km*rm));

ux = x_coeff_1*x_dot_dot_dot - x_coeff_2*x_dot_dot - x_coeff_3*x_dot;

y_coeff_1 = (J*Lm)/(rm*Km);
y_coeff_2 = (((Lm*J*Q)/(Km*rm)) + ((J*Rm)/(Km*rm)));
y_coeff_3 = (Km/rm + (Rm*J*Q)/(Km*rm));

uy = y_coeff_1*y_dot_dot_dot - y_coeff_2*y_dot_dot + y_coeff_3*y_dot;

%convert to actual motor voltage

ul = ux + uy;
ur = ux - uy;

%%Plot
figure;
plot(x_ref, y_ref, 'LineWidth',1.5);
grid on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');
exportgraphics(gcf, 'plots/spade.pdf','ContentType','vector');
%title("Trajectory to plot");

%%Inputs

figure;
plot(t,ul, 'LineWidth',1.5);
grid on;
xlabel('time (s)');
ylabel('u_l (V)');
exportgraphics(gcf, 'plots/spade_u_l.pdf','ContentType','vector');
%title("Left Motor Voltage input needed");

figure;
plot(t,ur, 'LineWidth',1.5);
grid on;
xlabel('time (s)');
ylabel('u_r (V)');
exportgraphics(gcf, 'plots/spade_u_r.pdf','ContentType','vector');
%title("Right Motor Voltage input needed");