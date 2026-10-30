function [mode, x0, param] = reset_n_transition(mode, x0, t_start, param)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    switch mode
        case 'print' % saves x, y and interruption time
            param.x_saved = x0(1);
            param.y_saved = x0(4);
            param.t_interrupt = t_start;
            mode = 'stop';
        case 'stop' % set velocities and motor current to 0, save x and y
            x0(2) = 0;
            x0(3) = 0;
            x0(5) = 0;
            x0(6) = 0;
            param.d_saved = x0(10);
            mode = 'park';
        case 'park' % chooses next parking state depending on distance
            if x0(1)>x0(4)
                x0(4:6) = 0;
                mode = 'park_x';
            else
                x0(1:3) = 0;
                mode = 'park_y';
            end
        case 'park_x' % set evrtg motor related to 0
            x0(1:6) = 0;
            mode = 'wipe';
        case 'park_y' % set evrtg motor related to 0
            x0(1:6) = 0;
            mode = 'wipe';
        case 'wipe'
            mode = 'cool';
        case 'cool' % sets nozzle states to zero except temperature
            x0(7) = param.T_low; 
            x0(8) = 0; 
            x0(9) = 0;
            x0(10) = 0;
            mode = 'heat';
        case 'heat' % sets nozzle states to corresponding eq. values
            x0(7) = param.T_high; 
            x0(8) = sqrt(param.Rn/param.Rtn*(param.T_high-param.T_e)); 
            x0(9) = sqrt(param.Rn/param.Rtn*(param.T_high-param.T_e))/param.Rn;
            mode = 'return';
        case 'return' % sets x0 to corresp. values to resume to trajectory
            param.t_saved = param.t_saved + t_start - param.t_interrupt;
            traj_x = param.trajectory_x(t_start-param.t_saved);
            traj_y = param.trajectory_y(t_start-param.t_saved);
            x0(1) = param.x_saved;
            x0(2) = traj_x(2);
            x0(3) = 1/(param.Km*param.rm)*(param.J*traj_x(3) + param.J*param.P*traj_x(2));
            x0(4) = param.y_saved;
            x0(5) = traj_y(2);
            x0(6) = 1/(param.Km*param.rm)*(param.J*traj_y(3) + param.J*param.Q*traj_y(2));
            mode = 'print';
    end
end