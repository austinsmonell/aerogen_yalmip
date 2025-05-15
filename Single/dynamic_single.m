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
m_ctr = u(1, :)*20000000;
de1 = u(2, :)*2;
ds1 = 0;
dr1 = u(3, :)*4*pi/(p(31)+p(32));

% extract parameters
r_gen = p(1);
moi_g = p(2);
d = p(5);
m_ac = p(6);
rho = p(7);
S = p(8);
CL0 = p(9);
CLa = p(10);
CD0 = p(11);
e = p(12);
vw = p(13);
moi_ac_m = p(14);
d_c = p(15);
d_b = S/d_c;
CMa = p(16);
CMq = p(17);
CMde = p(18);
CNb = p(19);
CNr = p(20);
CNdr = p(21);
CYb = p(22);
moi_ac_n = p(23);
CDb = p(24);
g = p(25);
m_teth = p(26);
CD_eff_teth = p(27);
CDds = p(29);
omega = p(30);
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