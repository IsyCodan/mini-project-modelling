rm = 1; %Motor radius
Km = 5; %Motor constant
Jm = 7; %Motor Inertia
Lm = 10; %Motor induction
Rm = 20; %Motor resistor


%State Space for x

A_x = [0 1 0; 0 0 (rm*Km)/Jm; 0 -(Km/(rm*Lm)) -(Rm/Lm)];
B_x = [0; 0; 1/Lm];
C_x = [1 0 0];
D_x = 0;

%State Space for y

A_y = [0 1 0; 0 0 (rm*Km)/Jm; 0 -(Km/(rm*Lm)) -(Rm/Lm)];
B_y = [0; 0; 1/Lm];
C_y = [1 0 0];
D_y = 0;

%%---PLOTTING---%%
%%----ELLIPSE---%%

rx = 0.01; %Semi x axis
ry = 0.04; %Semi y axis
xc = 0; %x coordinate of centre
yc = 0; %y coordinate of centre
w = 1; %angular frequency

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

ux = ((Lm*Jm)/(rm*Km))*x_dot_dot_dot + ((Rm*Jm)/(rm*Km))*x_dot_dot + (Km/rm)*x_dot;

uy = ((Lm*Jm)/(rm*Km))*y_dot_dot_dot + ((Rm*Jm)/(rm*Km))*y_dot_dot + (Km/rm)*y_dot;

%convert to actual motor voltage

ul = ux + uy;
ur = ux - uy;

%%Circle Plot
figure;
plot(x, y);
axis equal;
xlabel('x (m)');
ylabel('y (m)');
title("Trajectory to plot");

%%Inputs

figure;
plot(t,ul);
xlabel('time (s)');
ylabel('u_l (V)');
title("Left Motor Voltage input needed");

figure;
plot(t,ur);
xlabel('time (s)');
ylabel('u_r (V)');
title("Right Motor Voltage input needed");



