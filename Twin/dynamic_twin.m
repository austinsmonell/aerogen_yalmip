function dx = dynamic_twin(x, u, p)
%% extract variables
% States: theta, theta_dot
sigma = x(1,:);
sigma_dot = x(2,:);
theta1 = x(3, :);
theta2 = x(4, :);
va1_u = x(5, :);
va2_u = x(6, :);
psi1 = x(7, :);
psi2 = x(8, :);
x1 = x(9, :);
y1 = x(10, :);
x2 = x(11, :);
y2 = x(12, :);
va1_v = x(13, :);
va2_v = x(14, :);

%Controls: dist1
m_ctr = u(1, :)*p(22);
de1 = u(2, :)*p(23);
de2 = u(3, :)*p(23);
dr1 = u(4, :)*(p(24)/(p(19)+p(20)));
dr2 = u(5, :)*(p(24)/(p(19)+p(20)));
ds1 = 0;
ds2 = 0;


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
r1_dot = sigma_dot*r_gen;
va1_r = -r1_dot+vw;
va2_r = r1_dot+vw;
alpha1 = theta1+va1_r./va1_u;
alpha2 = theta2+va2_r./va2_u;
beta1 = va1_v./va1_u;
beta2 = va2_v./va2_u;

q1 = 0.5*rho*(va1_r.^2+va1_u.^2+va1_v.^2);
q2 = 0.5*rho*(va2_r.^2+va2_u.^2+va2_v.^2);

Cl1 = (CL0+CLa*alpha1);
L1 = q1*S.*Cl1;
Cl2 = (CL0+CLa*alpha2);
L2 = q2*S.*Cl2;

Cd1 = (CD0+CD_eff_teth+e*Cl1.^2+CDds*ds1);
D1 = q1*S.*Cd1;
Cd2 = (CD0+CD_eff_teth+e*Cl2.^2+CDds*ds2);
D2 = q2*S.*Cd2;

F1_zb = -sin(alpha1).*D1 - cos(alpha1).*L1;
F1_xb = -cos(alpha1).*D1 + sin(alpha1).*L1;
F2_zb = -sin(alpha2).*D2 - cos(alpha2).*L2;
F2_xb = -cos(alpha2).*D2 + sin(alpha2).*L2;
F1_yb = q1*S.*(CYb*beta1);
F2_yb = q2*S.*(CYb*beta2);

x1_dot = va1_u.*cos(psi1)-va1_v.*sin(psi1);
y1_dot = -va1_u.*sin(psi1)-va1_v.*cos(psi1);
psi1_dot = dr1;
x2_dot = va2_u.*cos(psi2)-va2_v.*sin(psi2);
y2_dot = -va2_u.*sin(psi2)-va2_v.*cos(psi2);
psi2_dot = dr2;

theta1_dot = de1;
theta2_dot = de2;
a_u1 = va1_v.*psi1_dot;
a_v1 = -va1_u.*psi1_dot;
a_u2 = va2_v.*psi2_dot;
a_v2 = -va2_u.*psi2_dot;
va1_u_dot = (F1_xb.*cos(theta1)+ F1_zb.*sin(theta1)+(m_ac+m_teth/2-omega*rho)*g*sin(psi1))/(m_ac+m_teth/3)+a_u1;
va2_u_dot = (F2_xb.*cos(theta2)+ F2_zb.*sin(theta2)+(m_ac+m_teth/2-omega*rho)*g*sin(psi2))/(m_ac+m_teth/3)+a_u2;
va1_v_dot = (F1_yb+(m_ac+m_teth/2-omega*rho)*g*cos(psi1))/(m_ac+m_teth/3)+a_v1;
va2_v_dot = (F2_yb+(m_ac+m_teth/2-omega*rho)*g*cos(psi2))/(m_ac+m_teth/3)+a_v2;

F1_aero_r = F1_xb.*(-sin(theta1))+F1_zb.*(cos(theta1));
F2_aero_r = F2_xb.*(-sin(theta2))+F2_zb.*(cos(theta2));

sigma_dot_dot = ((-F1_aero_r+F2_aero_r)*r_gen+m_ctr)/(moi_g+2*(m_ac+m_teth)*r_gen^2);
%% package derivitives
dx = [sigma_dot; sigma_dot_dot; theta1_dot; theta2_dot; va1_u_dot; va2_u_dot; psi1_dot; psi2_dot; x1_dot; y1_dot; x2_dot; y2_dot; va1_v_dot; va2_v_dot];%; theta1_dot_dot];
end