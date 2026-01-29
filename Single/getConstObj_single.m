function [Constraints,Objective] = getConstObj_single(gridSz, dt, p, ctr_obj_gain, nx, nu, x, u, solve_reelin, x0, xf, u0)
    %initial/final condition & limits
    %1:sigma 2:sigma_dot, 3:va1_u, 4: theta, 5:psi, 6:x1, 7:x2, 8:va1_v
    %1:m_ctr, 2:de1(theta_dot), 3:dr1(psi_Dot)
    u_up = [1; 1; 1];
    u_lw = [-1; -1; -1];
    x_up = [inf;  inf; 200; pi/2; inf; inf; inf; inf];
    x_lw = [-inf; -inf; 1; -pi/2; -inf; -inf; -inf; -inf];
    x0_up = [0; inf; 200; pi/3; 0; 0; 0; inf];
    x0_lw = [0; 0; 1; -pi/3; 0;  0; 0; -inf];
    cyl_idx = [2, 3, 4, 6, 7, 8];

    %% Contraints & Objective
    Constraints = [];
    %   initial state limits
    if solve_reelin
%         u_up = [1; 1; 1];
        Constraints = [Constraints, x(:, 1) == x0, x(:, 1) == x0];
        Constraints = [Constraints, x(:, end) == xf, x(:, end) == xf];
        Constraints = [Constraints, u(:, 1) == u0, u(:, 1) == u0];
        Constraints = [Constraints, u(:, end) == u0, u(:, end) == u0];
    else
        Constraints = [Constraints, x(:, 1) >= x0_lw, x(:, 1) <= x0_up];
    end
    
    %   path constraints
    
    %   state limits
    Constraints = [Constraints, x<=ones(nx, gridSz).*x_up, x>=ones(nx, gridSz).*x_lw];
    
    %   other limits
    % extract parameters
    r_gen = p(1);
    moi_g = p(2);
    m_ac = p(3);
    rho = p(4);
    S = p(5);
    CL0 = p(6);
    CLa = p(7);
    CD0 = p(8);
    e = p(9);
    vw = p(10);
    CYb = p(11);
    g = p(12);
    m_teth = p(13);
    CD_eff_teth = p(14);
    CDds = p(17);
    omega = p(18);

    sigma_dot = x(2, :);
    theta1 = x(4, :);
    va1_u = x(3, :);
    va1_v = x(8, :);
    alpha_up =  p(16);
    alpha_low = p(15);
    beta_lim = p(21);

    r1_dot = sigma_dot*r_gen;
    va1_r = -r1_dot+vw;
    alpha1 = va1_r./va1_u+theta1;
    Cl1 = (CL0+CLa*alpha1);%done
    Cd1 = (CD0+CD_eff_teth+e*Cl1.^2);
    F1_zb = -sin(alpha1).*Cd1 - cos(alpha1).*Cl1;%done
    F1_xb = -cos(alpha1).*Cd1 + sin(alpha1).*Cl1;%done
    F1_aero_r = F1_xb.*(-sin(theta1))+F1_zb.*(cos(theta1));%done


    Constraints = [Constraints, alpha1 <= (alpha_up*pi/180), alpha1 >= (alpha_low*pi/180),...
                   va1_v./va1_u <= beta_lim*pi/180, va1_v./va1_u >= -beta_lim*pi/180,...
                   F1_aero_r <= 0];%may need to relax to >0 without warmstart
    
    %   control limits
    Constraints = [Constraints, u<=ones(nu, gridSz).*u_up, u>=ones(nu, gridSz).*u_lw];
    
    %dynamics and objective
    %intial and final states
    Objective = u(1, 1).*x(2, 1)+sum(abs(u(:, 1)-u(:, end)))*ctr_obj_gain;
    x_dotk = dynamic_single(x(:, end), u(:, end), p);
    x_dotk1 = dynamic_single(x(:, 1), u(:, 1), p);
    if ~solve_reelin
        Constraints = [Constraints, x(cyl_idx, 1) == x(cyl_idx, end)+dt/2*(x_dotk(cyl_idx)+x_dotk1(cyl_idx)),...
                                    x([5], 1)+2*pi == x([5], end)+dt/2*(x_dotk([5])+x_dotk1([5]))];
    end
    %loops through each step
    for m = 1 : gridSz-1
      x_dotk = dynamic_single(x(:, m), u(:, m), p);
      x_dotk1 = dynamic_single(x(:, m+1), u(:, m+1), p);
      Constraints = [Constraints, x(:, m+1) == x(:, m)+dt/2*(x_dotk+x_dotk1)];%discrete time dynamics, equality cnst
      Objective = Objective + u(1, m+1).*x(2, m+1)+sum(abs(u(:, m+1)-u(:, m)).*[1; 1; 1])*ctr_obj_gain; %objective func, max energy
    end
end

