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


% theta1_dot = x(8, :);

%Controls: dist1
m_ctr = u(1, :)*30000;
de1 = u(2, :)*1;
de2 = u(3, :)*1;
dr1 = u(4, :)*(4*pi/(p(19)+p(20)));
dr2 = u(5, :)*(4*pi/(p(19)+p(20)));
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
r1_dot = sigma_dot*r_gen;%done
va1_r = -r1_dot+vw;%done
va2_r = r1_dot+vw;%done
alpha1 = theta1+va1_r./va1_u;%done
alpha2 = theta2+va2_r./va2_u;%done
beta1 = va1_v./va1_u;
beta2 = va2_v./va2_u;

q1 = 0.5*rho*(va1_r.^2+va1_u.^2+va1_v.^2);%done
q2 = 0.5*rho*(va2_r.^2+va2_u.^2+va2_v.^2);%done

Cl1 = (CL0+CLa*alpha1);%done
L1 = q1*S.*Cl1;%done
Cl2 = (CL0+CLa*alpha2);%done
L2 = q2*S.*Cl2;%done

Cd1 = (CD0+CD_eff_teth+e*Cl1.^2+CDds*ds1);
D1 = q1*S.*Cd1;%done
Cd2 = (CD0+CD_eff_teth+e*Cl2.^2+CDds*ds2);
D2 = q2*S.*Cd2;%done

F1_zb = -sin(alpha1).*D1 - cos(alpha1).*L1;%done
F1_xb = -cos(alpha1).*D1 + sin(alpha1).*L1;%done
F2_zb = -sin(alpha2).*D2 - cos(alpha2).*L2;%done
F2_xb = -cos(alpha2).*D2 + sin(alpha2).*L2;%done
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
a_c1 = sqrtm(va1_u.^2+va1_v.^2).*psi1_dot;
a_c2 = sqrtm(va2_u.^2+va2_v.^2).*psi2_dot;
va1_u_dot = (F1_xb.*cos(theta1)+ F1_zb.*sin(theta1)+(m_ac+m_teth/2-omega*rho)*g*sin(psi1))/(m_ac+m_teth/3)+a_c1.*sin(beta1)*1;
va2_u_dot = (F2_xb.*cos(theta2)+ F2_zb.*sin(theta2)+(m_ac+m_teth/2-omega*rho)*g*sin(psi2))/(m_ac+m_teth/3)+a_c2.*sin(beta2)*1;
va1_v_dot = (F1_yb+(m_ac+m_teth/2-omega*rho)*g*cos(psi1))/(m_ac+m_teth/3)-a_c1.*cos(beta1)*1;
va2_v_dot = (F2_yb+(m_ac+m_teth/2-omega*rho)*g*cos(psi2))/(m_ac+m_teth/3)-a_c2.*cos(beta2)*1;

F1_aero_r = F1_xb.*(-sin(theta1))+F1_zb.*(cos(theta1));%done
F2_aero_r = F2_xb.*(-sin(theta2))+F2_zb.*(cos(theta2));%done

sigma_dot_dot = ((-F1_aero_r+F2_aero_r)*r_gen+m_ctr)/(moi_g+2*(m_ac+m_teth)*r_gen^2);
%% package derivitives
dx = [sigma_dot; sigma_dot_dot; theta1_dot; theta2_dot; va1_u_dot; va2_u_dot; psi1_dot; psi2_dot; x1_dot; y1_dot; x2_dot; y2_dot; va1_v_dot; va2_v_dot];%; theta1_dot_dot];
end