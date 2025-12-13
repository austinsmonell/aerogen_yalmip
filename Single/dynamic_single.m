function dx = dynamic_single(x, u, p)
%% extract variables
% States: theta, theta_dot
sigma = x(1,:);
sigma_dot = x(2,:);
va1_u = x(3, :);
theta1 = x(4, :);
psi1 = x(5, :);
x1 = x(6, :);
y1 = x(7, :);
va1_v = x(8, :);

%Controls: dist1
m_ctr = u(1, :)*20000;
de1 = u(2, :)*1;
ds1 = 0;
dr1 = u(3, :)*4*pi/(p(19)+p(20));

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
%% dynamics
r1_dot = sigma_dot*r_gen;%done
va1_r = -r1_dot+vw;%done
alpha1 = theta1+va1_r./va1_u;%done
beta1 = va1_v./va1_u;

q1 = 0.5*rho*(va1_r.^2+va1_u.^2+va1_v.^2);%done
Cl1 = (CL0+CLa*alpha1);%done
L1 = q1*S.*Cl1;%done
Cd1 = (CD0+CD_eff_teth+e*Cl1.^2+CDds*ds1);
D1 = q1*S.*Cd1;%done
F1_zb = -sin(alpha1).*D1 - cos(alpha1).*L1;%done
F1_xb = -cos(alpha1).*D1 + sin(alpha1).*L1;%done
F1_yb = q1*S.*(CYb*beta1);

x1_dot = va1_u.*cos(psi1)-va1_v.*sin(psi1);
y1_dot = -va1_u.*sin(psi1)-va1_v.*cos(psi1);
psi1_dot = dr1;
theta1_dot = de1;
a_c = va1_u.*psi1_dot;
va1_u_dot = (F1_xb.*cos(theta1)+F1_zb.*sin(theta1)+(m_ac+m_teth/2-omega*rho)*g*sin(psi1))/(m_ac+m_teth/3)+a_c.*sin(beta1);
va1_v_dot = (F1_yb+(m_ac+m_teth/2-omega*rho)*g*cos(psi1))/(m_ac+m_teth/3)-a_c.*cos(beta1);
F1_aero_r = F1_xb.*(-sin(theta1))+F1_zb.*(cos(theta1));%done

sigma_dot_dot = (-F1_aero_r*r_gen+m_ctr)/(moi_g+(m_ac+m_teth)*r_gen^2);
%% package derivitives
dx = [sigma_dot; sigma_dot_dot; va1_u_dot; theta1_dot; psi1_dot; x1_dot; y1_dot; va1_v_dot];
end