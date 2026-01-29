function [Constraints,Objective] = getConstObj_twin(gridSz, dt, p, ctr_obj_gain, nx, nu, x, u)
    %initial/final condition & limits
    %1:sigma 2:sigma_dot, 3:theta1, 4:theta2, 5:va1_u, 6:va2_u, 7:psi1, 8:psi2, 9:x1, 10:y1, 11:x2, 12:y2, 13:va1_v, 14:va2_v
    %1:m_ctr, 2:de1(theta1_dot), 3:de2(theta2_dot), 4:dr1(psi1_dot), 4:dr2(psi2_dot)
    u_up = [1;  1; 1; 1; 1];%; 0; 0];%; 2*pi/dt+1e-6; 2*pi/dt+1e-6];
    u_lw = [-1; -1; -1; -1; -1];%; 0; 0];%; -2*pi/dt-1e-6; -2*pi/dt-1e-6];
    x_up = [inf; inf; pi/2; pi/2; 200; 200; inf; inf; inf; inf; inf; inf; inf; inf];
    x_lw = [-inf; -inf; -pi/2; -pi/2; 1; 1; -inf; -inf; -inf; -inf; -inf; -inf; -inf; -inf];
    x0_up = [0; inf; pi/2; pi/2; 200; 200; pi; pi; 0; 0; 0; 0; inf; inf];
    x0_lw = [0; -inf; -pi/2; -pi/2; 1; 1; -pi; -pi; 0; 0; 0; 0; -inf; -inf];
    cyl_idx = [1, 2, 3, 4, 5, 6, 9, 10, 11, 12, 13, 14];

    %% Contraints & Objective
    Constraints = [];
    %   intial/final conditions
    
    %   initial state limits
    Constraints = [Constraints, x(:, 1) >= x0_lw, x(:, 1) <= x0_up];
    
    %   state limits
    Constraints = [Constraints, x<=ones(nx, gridSz).*x_up, x>=ones(nx, gridSz).*x_lw, u(1, :).*x(2, :) <= zeros(1, gridSz)];
    
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
    theta1 = x(3, :);
    va1_u = x(5, :);
    theta2 = x(4, :);
    va2_u = x(6, :);
    va1_v = x(13, :);
    va2_v = x(14, :);
    alpha_up =  p(16);
    alpha_low = p(15);
    beta_lim = p(21);
    
    r1_dot = sigma_dot*r_gen;
    va1_r = -r1_dot+vw;
    va2_r = r1_dot+vw;
    alpha1 = va1_r./va1_u+theta1;
    alpha2 = va2_r./va2_u+theta2;
    Cl1 = (CL0+CLa*alpha1);%done
    Cl2 = (CL0+CLa*alpha2);%done
    Cd1 = (CD0+CD_eff_teth+e*Cl1.^2);
    Cd2 = (CD0+CD_eff_teth+e*Cl2.^2);
    F1_zb = -sin(alpha1).*Cd1 - cos(alpha1).*Cl1;%done
    F1_xb = -cos(alpha1).*Cd1 + sin(alpha1).*Cl1;%done
    F2_zb = -sin(alpha2).*Cd2 - cos(alpha2).*Cl2;%done
    F2_xb = -cos(alpha2).*Cd2 + sin(alpha2).*Cl2;%done
    F1_aero_r = F1_xb.*(-sin(theta1))+F1_zb.*(cos(theta1));%done
    F2_aero_r = F2_xb.*(-sin(theta2))+F2_zb.*(cos(theta2));%done

    Constraints = [Constraints, alpha1 <= (alpha_up*pi/180), alpha1 >= (alpha_low*pi/180),...
                   alpha2 <= (alpha_up*pi/180), alpha2 >= (alpha_low*pi/180),...
                   va1_v./va1_u <= beta_lim*pi/180, va1_v./va1_u >= -beta_lim*pi/180,...
                   va2_v./va2_u <= beta_lim*pi/180, va2_v./va2_u >= -beta_lim*pi/180,...
                   F1_aero_r <= 0, F2_aero_r <= 0];%may need to relax to >0 without warmstart
    
    %   control limits
    Constraints = [Constraints, u<=ones(nu, gridSz).*u_up, u>=ones(nu, gridSz).*u_lw];
    
    %dynamics and objective
    %intial and final states
    Objective = u(1, 1).*x(2, 1)+sum(abs(u(:, 1)-u(:, end)))*ctr_obj_gain;
    x_dotk = dynamic_twin(x(:, end), u(:, end), p);
    x_dotk1 = dynamic_twin(x(:, 1), u(:, 1), p);
    Constraints = [Constraints, x(cyl_idx, 1) == x(cyl_idx, end)+dt/2*(x_dotk(cyl_idx)+x_dotk1(cyl_idx)),...
                    x([7], 1)+2*pi == x([7], end)+dt/2*(x_dotk([7])+x_dotk1([7])),...
                    x([8], 1)-2*pi == x([8], end)+dt/2*(x_dotk([8])+x_dotk1([8]))];
    %loops through each step
    for m = 1 : gridSz-1
      x_dotk = dynamic_twin(x(:, m), u(:, m), p);
      x_dotk1 = dynamic_twin(x(:, m+1), u(:, m+1), p);
      Constraints = [Constraints, x(:, m+1) == x(:, m)+dt/2*(x_dotk+x_dotk1)];%discrete time dynamics, equality cnst
      Objective = Objective + u(1, m+1).*x(2, m+1)+sum(abs(u(:, m+1)-u(:, m)))*ctr_obj_gain; %objective func, max energy
    end
end

