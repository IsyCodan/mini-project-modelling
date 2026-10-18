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
%%----SPADE-----%%

Tend = 70; %faster means more input voltage

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

x_dot = d_dt(x_ref);
x_dot_dot = d_dt(x_dot);
x_dot_dot_dot = d_dt(x_dot_dot);

y_dot = d_dt(y_ref);
y_dot_dot = d_dt(y_dot);
y_dot_dot_dot = d_dt(y_dot_dot);

ux = ((Lm*Jm)/(rm*Km))*x_dot_dot_dot + ((Rm*Jm)/(rm*Km))*x_dot_dot + (Km/rm)*x_dot;

uy = ((Lm*Jm)/(rm*Km))*y_dot_dot_dot + ((Rm*Jm)/(rm*Km))*y_dot_dot + (Km/rm)*y_dot;

%convert to actual motor voltage

ul = ux + uy;
ur = ux - uy;

%%Plot
figure;
plot(x_ref, y_ref);
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