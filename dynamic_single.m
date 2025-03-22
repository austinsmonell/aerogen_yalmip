function dx = dynamic_single(x, u, p)
%% extract variables
% States: theta, theta_dot
sigma = x(1,:);
sigma_dot = x(2,:);
x1 = x(3, :);
x1_dot = x(4, :);
y1 = x(5, :);
y1_dot = x(6, :);
% theta1 = x(5, :);
% theta1_dot = x(6, :);
% y1 = x(7, :);
% y1_dot = x(8, :);
% psi1 = x(9, :);
% psi1_dot = x(10, :);

%Controls: dist1
m_ctr = u(1, :);
de1 = u(2, :);
dr1 = u(3, :);
% dr1 = u(3, :);
% ds1 = u(4, :);

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
va1_xy = sqrtm(x1_dot.^2+y1_dot.^2);
% alpha1 = range_angle(atan2(va1_r, va1_xy));

q1 = 0.5*rho*(va1_r.^2+va1_xy.^2);%done
Cl1 = (CL0);
L1 = q1*S.*Cl1;%done
F1_zb = -L1;

x1_dot_dot = de1;
y1_dot_dot = dr1;

F1_aero_r = F1_zb;

sigma_dot_dot = (-F1_aero_r*r_gen+m_ctr)/(moi_g+(m_ac+m_teth)*r_gen^2);
%% package derivitives
dx = [sigma_dot; sigma_dot_dot; x1_dot; x1_dot_dot; y1_dot; y1_dot_dot];
end