function [Constraints,Objective] = getConstObj_twin(gridSz, dt, p, alpha_low, alpha_up, ctr_obj_gain, nx, nu, x, u);
    %initial/final condition & limits
    u_up = [1;  1; 1; 1; 1];%; 0; 0];%; 2*pi/dt+1e-6; 2*pi/dt+1e-6];
    u_lw = [-1; -1; -1; -1; -1];%; 0; 0];%; -2*pi/dt-1e-6; -2*pi/dt-1e-6];
    x_up = [inf; inf; pi/3; pi/3; 200; 200; inf; inf; inf; inf; inf; inf; inf; inf];
    x_lw = [-inf; -inf; -pi/3; -pi/3; 1; 1; -inf; -inf; -inf; -inf; -inf; -inf; -inf; -inf];
    x0_up = [0; inf; pi/3; pi/3; 200; 200; pi; pi; 0; 0; 0; 0; inf; inf];
    x0_lw = [0; -inf; -pi/3; -pi/3; 1; 1; -pi; -pi; 0; 0; 0; 0; -inf; -inf];
    cyl_idx = [1, 2, 3, 4, 5, 6, 9, 10, 11, 12, 13, 14];

    %% Contraints & Objective
    Constraints = [];
    %   intial/final conditions
    
    %   initial state limits
    Constraints = [Constraints, x(:, 1) >= x0_lw, x(:, 1) <= x0_up];
    
    %   state limits
    Constraints = [Constraints, x<=ones(nx, gridSz).*x_up, x>=ones(nx, gridSz).*x_lw, u(1, :).*x(2, :) <= zeros(1, gridSz)];
    
    %   other limits
    sigma_dot = x(2, :);
    r_gen = p(1);
    vw = p(13);
    theta1 = x(3, :);
    va1_u = x(5, :);
    theta2 = x(4, :);
    va2_u = x(6, :);
    va1_v = x(13, :);
    va2_v = x(14, :);
    
    r1_dot = sigma_dot*r_gen;
    va1_r = -r1_dot+vw;
    va2_r = r1_dot+vw;
    Constraints = [Constraints, va1_r./va1_u <= (alpha_up-theta1), va1_r./va1_u >= (alpha_low-theta1),...
                   va2_r./va2_u <= (alpha_up-theta2), va2_r./va2_u >= (alpha_low-theta2),...
                   va1_v./va1_u <= 10*pi/180, va1_v./va1_u >= -10*pi/180,...
                   va2_v./va2_u <= 10*pi/180, va2_v./va2_u >= -10*pi/180];
    
    %   control limits
    Constraints = [Constraints, u<=ones(nu, gridSz).*u_up, u>=ones(nu, gridSz).*u_lw];
    
    %dynamics and objective
    Objective = u(1, 1).*x(2, 1)+sum(abs(u(:, 1)-u(:, end)))*ctr_obj_gain;
    x_dotk = dynamic_twin(x(:, end), u(:, end), p);
    x_dotk1 = dynamic_twin(x(:, 1), u(:, 1), p);
    Constraints = [Constraints, x(cyl_idx, 1) == x(cyl_idx, end)+dt/2*(x_dotk(cyl_idx)+x_dotk1(cyl_idx)),...
                    x([7], 1)+2*pi == x([7], end)+dt/2*(x_dotk([7])+x_dotk1([7])),...
                    x([8], 1)-2*pi == x([8], end)+dt/2*(x_dotk([8])+x_dotk1([8]))];
    for m = 1 : gridSz-1
      x_dotk = dynamic_twin(x(:, m), u(:, m), p);
      x_dotk1 = dynamic_twin(x(:, m+1), u(:, m+1), p);
      Constraints = [Constraints, x(:, m+1) == x(:, m)+dt/2*(x_dotk+x_dotk1)];%discrete time dynamics, equality cnst
      Objective = Objective + u(1, m+1).*x(2, m+1)+sum(abs(u(:, m+1)-u(:, m)))*ctr_obj_gain; %objective func, max energy
    end
end

