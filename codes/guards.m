function [g,isterminal,direction] = guards(t,x,mode,param)
%UNTITLED Summary of this function goes here
%   define g, where g >= 0
    isterminal = 1; % terminates integration if event function hits zero
    direction = 0; % detected all zeros, for increasing and decreasing event function

    % extract all state equations
    x_x = x(1:3);
    x_y = x(4:6);
    x_n = x(7:9);
    x_d = x(10);

    epsilon = 1e-3; % small value to account for solver step sizes
    
    switch mode
        case 'print'
            g = x_d - param.L_filament;
        case 'stop'
            g = sqrt(x_x(2)^2+x_y(2)^2) - epsilon;
        case 'park_x' % park at x = 0
            g = x_x(1);
        case 'park_y' % park at y = 0
            g = x_y(1);
        case 'wipe'
            g = (x_d - param.d_saved) - param.L_wipe;
        case 'cool'
            g = x_n(1) - param.T_low;
        case 'heat'
            g = x_n(1) - param.T_high;
        case 'return' % return to saved location
            g = sqrt((x_x(1)-param.x_saved)^2+(x_y(1)-param.y_saved)^2)-epsilon;
    end
end