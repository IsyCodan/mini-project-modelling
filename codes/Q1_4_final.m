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
%%----ELLIPSE---%%

rx = 0.01; %Semi x axis
ry = 0.04; %Semi y axis

%Keep rx and ry equal for a circle

xc = 0; %x coordinate of centre
yc = 0; %y coordinate of centre
w = 1; %angular frequency (random baseline value)

Tend = 2*pi/w;
N = 2000;
t = linspace(0,Tend,N);

%parametric equations for circle
x = xc + rx*cos(w*t);
x_dot = -rx*w*sin(w*t);
x_dot_dot = -rx*w*w*cos(w*t);
x_dot_dot_dot = rx*w*w*w*sin(w*t);

y = yc + ry*sin(w*t);
y_dot = ry*w*cos(w*t);
y_dot_dot = -ry*w*w*sin(w*t);
y_dot_dot_dot = -ry*w*w*w*cos(w*t);

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

%%Circle Plot
figure;
plot(x, y, 'LineWidth', 1.5);
grid on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');
exportgraphics(gcf, 'plots/circle.pdf','ContentType','vector');
%title("Trajectory to plot");

%%Inputs

figure;
plot(t,ul,'LineWidth', 1.5);
grid on;
xlabel('time (s)');
ylabel('u_l (V)');
exportgraphics(gcf, 'plots/circle_u_l.pdf','ContentType','vector');
%title("Left Motor Voltage input needed");

figure;
plot(t,ur,'LineWidth', 1.5);
grid on;
xlabel('time (s)');
ylabel('u_r (V)');
exportgraphics(gcf, 'plots/circle_u_r.pdf','ContentType','vector');
%title("Right Motor Voltage input needed");



