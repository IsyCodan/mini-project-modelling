 function dx = sys(x,u,param)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    % unpack x and u vectors
    x_x = x(1:3);
    x_y = x(4:6);
    x_n = x(7:9);
    x_d = x(10);
    u_x = u(1);
    u_y = u(2);
    u_n = u(3);
    u_d = u(4);

    rm = param.rm;
    Km = param.Km;
    J = param.J;
    Lm = param.Lm;
    Rm = param.Rm;
    P = param.P;
    Q = param.Q;
    Rn = param.Rn;
    Ln = param.Ln;
    Cn = param.Cn;
    Rtn = param.Rtn;
    Ctn = param.Ctn;
    T_e = param.T_e;

    A_x = [0 1 0; 0 -P (rm*Km)/J; 0 -(Km/(rm*Lm)) -(Rm/Lm)];
    B_x = [0; 0; 1/Lm];
    % C_x = [1 0 0];
    % D_x = 0;

    dx_x = A_x*x_x+B_x*u_x;

    A_y = [0 1 0; 0 -Q (rm*Km)/J; 0 -(Km/(rm*Lm)) -(Rm/Lm)];
    B_y = [0; 0; 1/Lm];
    % C_y = [1 0 0];
    % D_y = 0;

    dx_y = A_y*x_y+B_y*u_y;

    dx_n = [x_n(2)^2/(Rn*Ctn)-(x_n(1)-T_e)/(Rtn*Ctn);
            x_n(3)/Cn-x_n(2)/(Rn*Cn);
            (u_n-x_n(2))/Ln];

    dx_d = u_d*param.dot_d_standard*(1 + sqrt(x_x(2)^2+x_y(2)^2)); % standard filament extrusion plus scaled velocity of printed path  
    
    dx=[dx_x; 
        dx_y; 
        dx_n;
        dx_d];
end