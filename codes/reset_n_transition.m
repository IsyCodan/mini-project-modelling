function [mode, x0, param] = reset_n_transition(mode, x0, t_start, param)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    switch mode
        case 'print'
            mode = 'stop';
        case 'stop' 
            x0(2) = 0;
            x0(5) = 0;
            param.x_saved = x0(1);
            param.y_saved = x0(4);
            param.t_saved = t_start;
            param.d_saved = x0(10);
            mode = 'park_x';
        case 'park_x'
            x0(1) = 0;
            x0(2) = 0;
            mode = 'park_y';
        case 'park_y'
            x0(4) = 0;
            x0(5) = 0;
            mode = 'wipe';
        case 'wipe'
            mode = 'cool';
        case 'cool'
            x0(10) = 0;
            mode = 'heat';
        case 'heat'
            mode = 'return';
        case 'return'
            x0(1) = param.x_saved;
            x0(4) = param.y_saved;
            mode = 'print';
    end
end