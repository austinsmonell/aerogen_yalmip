function [Constraints,Objective] = getConstObj(gridSz, dt, nx, nu, x, u, p, x0_lw, x0_up, x_lw, x_up, u_up, u_lw, alpha_lim, cyl_idx, ctr_obj_gain)
    %% Contraints & Objective
    Constraints = [];
    %   initial state limits
    Constraints = [Constraints, x(:, 1) >= x0_lw, x(:, 1) <= x0_up];
    
    %   boundary constraints
    
    %   state limits
    Constraints = [Constraints, x<=ones(nx, gridSz).*x_up, x>=ones(nx, gridSz).*x_lw];
    
    %   other limits
    sigma_dot = x(2, :);
    r_gen = p(1);
    vw = p(13);
    theta1 = x(4, :);
    va1_xy = x(3, :);
    
    r1_dot = sigma_dot*r_gen;
    va1_r = -r1_dot+vw;
    Constraints = [Constraints, va1_r./va1_xy <= tan(alpha_lim-theta1), va1_r./va1_xy >= tan(-alpha_lim-theta1)];
    
    %   control limits
    Constraints = [Constraints, u<=ones(nu, gridSz).*u_up, u>=ones(nu, gridSz).*u_lw];
    
    %dynamics and objective
    Objective = u(1, 1).*x(2, 1)+sum(abs(u(:, 1)-u(:, end)))*ctr_obj_gain;
    x_dotk = dynamic_single(x(:, end), u(:, end), p);
    x_dotk1 = dynamic_single(x(:, 1), u(:, 1), p);
    Constraints = [Constraints, x(cyl_idx, 1) == x(cyl_idx, end)+dt/2*(x_dotk(cyl_idx)+x_dotk1(cyl_idx)),...
                                x([5], 1)+2*pi == x([5], end)+dt/2*(x_dotk([5])+x_dotk1([5]))];
    for m = 1 : gridSz-1
      x_dotk = dynamic_single(x(:, m), u(:, m), p);
      x_dotk1 = dynamic_single(x(:, m+1), u(:, m+1), p);
      Constraints = [Constraints, x(:, m+1) == x(:, m)+dt/2*(x_dotk+x_dotk1)];%discrete time dynamics, equality cnst
      Objective = Objective + u(1, m+1).*x(2, m+1)+sum(abs(u(:, m+1)-u(:, m)))*ctr_obj_gain; %objective func, max energy
    end
end

