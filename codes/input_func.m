function u = input_func(t,desired_x,desired_y,mode,param)
%UNTITLED4 Summary of this function goes here
%   desired_x and desired_y should be vectors of derivations of the desired
%   trajectory for x and y
    T_e = param.T_e;
    T_high = param.T_high;
    Rn = param.Rn;
    Rtn = param.Rtn;

    J = param.J;
    Lm = param.Lm;
    Km = param.Km;
    rm = param.rm;
    Rm = param.Rm;
    P = param.P;
    Q = param.Q;
    
    % coefficients needed to compute the 
    dddot_x_coeff = (J*Lm)/(rm*Km);
    ddot_x_coeff = (((Lm*J*P)/(Km*rm)) + ((J*Rm)/(Km*rm)));
    dot_x_coeff = (Km/rm + (Rm*J*P)/(Km*rm));
    dddot_y_coeff = (J*Lm)/(rm*Km);
    ddot_y_coeff = (((Lm*J*Q)/(Km*rm)) + ((J*Rm)/(Km*rm)));
    dot_y_coeff = (Km/rm + (Rm*J*Q)/(Km*rm));

    x_t = desired_x(t-param.t_saved);
    y_t = desired_y(t-param.t_saved);

    switch mode
        case 'print'
            u_x = dddot_x_coeff*x_t(4) + ddot_x_coeff*x_t(3) + dot_x_coeff*x_t(2);
            u_y = dddot_y_coeff*y_t(4) + ddot_y_coeff*y_t(3) + dot_y_coeff*y_t(2);
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
            u_d = 1;
        case 'stop'
            u_x = 0;
            u_y = 0;
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
            u_d = 0;
        case 'park'
            u_x = -dot_x_coeff*param.park_velocity;
            u_y = -dot_y_coeff*param.park_velocity;
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
            u_d = 0;
        case 'park_x' % move to wiping location
            u_x = -dot_x_coeff*param.park_velocity;
            u_y = 0;
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
            u_d = 0;
        case 'park_y'
            u_x = 0;
            u_y = -dot_y_coeff*param.park_velocity;
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
            u_d = 0;
        case 'wipe'
            u_x = 0;
            u_y = 0;
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
            u_d = 1;
        case 'cool'
            u_x = 0;
            u_y = 0;
            u_n = 0;
            u_d = 0;
        case 'heat'
            u_x = 0;
            u_y = 0;
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
            u_d = 0;
        case 'return' % return to wiping location
            u_x = dot_x_coeff*param.x_saved/sqrt(param.x_saved^2+param.y_saved^2)*param.park_velocity;
            u_y = dot_y_coeff*param.y_saved/sqrt(param.x_saved^2+param.y_saved^2)*param.park_velocity;
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
            u_d = 0;
        otherwise
            u_x = 0;
            u_y = 0;
            u_n = 0;
            u_d = 0;
    end
    u = [u_x;
         u_y;
         u_n;
         u_d];
end