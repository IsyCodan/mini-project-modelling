function u = input_func(t,desired_x,desired_y,mode,param)
%UNTITLED4 Summary of this function goes here
%   desired_x and desired_y should be vectors of derivations of the desired
%   trajectory for x and y
    T_e = param.T_e;
    T_high = param.T_high;
    Rn = param.Rn;
    Rtn = param.Rtn;

    dddot_coeff = (J*Lm)/(rm*Km);
    ddot_coeff = (((Lm*J*P)/(Km*rm)) + ((J*Rm)/(Km*rm)));
    dot_coeff = (Km/rm + (Rm*J*P)/(Km*rm));

    switch mode
        case 'print'
            u_x = dddot_coeff*desired_x(4) + ddot_coeff*desired_x(3) + dot_coeff*desired_x(2);
            u_y = dddot_coeff*desired_y(4) + ddot_coeff*desired_y(3) + dot_coeff*desired_y(2);
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
        case 'stop'
            u_x = 0;
            u_y = 0;
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
        case 'save'
            u_x = 0;
            u_y = 0;
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
        case 'move' % move to wiping location
            u_x = dddot_coeff*desired_x(4) + ddot_coeff*desired_x(3) + dot_coeff*desired_x(2);
            u_y = dddot_coeff*desired_y(4) + ddot_coeff*desired_y(3) + dot_coeff*desired_y(2);
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
        case 'wipe'
            u_x = 0;
            u_y = 0;
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
        case 'cool'
            u_x = 0;
            u_y = 0;
            u_n = 0;
        case 'load' % reload filament
            u_x = 0;
            u_y = 0;
            u_n = 0;
        case 'heat'
            u_x = 0;
            u_y = 0;
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
        case 'return' % return to wiping location
            u_x = dddot_coeff*desired_x(4) + ddot_coeff*desired_x(3) + dot_coeff*desired_x(2);
            u_y = dddot_coeff*desired_y(4) + ddot_coeff*desired_y(3) + dot_coeff*desired_y(2);
            u_n = sqrt(Rn/Rtn*(T_high-T_e));
        otherwise
            u_x = 0;
            u_y = 0;
            u_n = 0;
    end
    u = [u_x;
         u_y;
         u_n];
end