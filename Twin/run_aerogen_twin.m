clc
clear
close all

yalmip('clear')
addpath('..\')


% Define horizon
tf = 10;
gridSz = 60;
dt = tf/(gridSz-1);
timeVec = linspace(0, tf, gridSz);

% Define variables/params
nx = 4; 
nu = 3;
x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:va, 4: theta, 5:psi, 6:x1, 7:x2
u = sdpvar(nu, gridSz);%1:m_ctr, 2:de1(theta_dot), 3:ds1, 4:dr1(psi_Dot)
p = getParams();

%initial/final condition & limits
u_up = [10000;  1; 1];%; 2*pi/dt+1e-6; 2*pi/dt+1e-6];
u_lw = [-10000; -1; -1];%; -2*pi/dt-1e-6; -2*pi/dt-1e-6];
x_up = [inf;  inf; pi/4; pi/4];%; inf; inf];%; inf];
x_lw = [-inf; -inf; -pi/4; -pi/4];%; -inf; -inf];%; -inf];
x0_up = [0;   inf; pi/4; pi/4];%; 0; 0];%; inf];
x0_lw = [0;   -inf; -pi/4; -pi/4];%; 0; 0];%; -inf];

%% Contraints & Objective
Constraints = [];
%   intial/final conditions

%   initial state limits
Constraints = [Constraints, x(:, 1) >= x0_lw, x(:, 1) <= x0_up];

%   boundary constraints
% % Constraints = [Constraints, abs(x(7, 1) - (x(7, end)-2*pi))<= 1e-6, abs(x(8, 1) - (x(8, end)+2*pi))<= 1e-6];

%   state limits
Constraints = [Constraints, x<=ones(nx, gridSz).*x_up, x>=ones(nx, gridSz).*x_lw, u(1, :).*x(2, :) <= zeros(1, gridSz)];

%   other limits
sigma_dot = x(2, :);
r_gen = p(1);
vw = p(13);
theta1 = x(3, :);
va1_xy = 35;
theta2 = x(4, :);
va2_xy = 35;
alpha_lim = 30*pi/180;

r1_dot = sigma_dot*r_gen;
va1_r = -r1_dot+vw;
va2_r = r1_dot+vw;
Constraints = [Constraints, va1_r./va1_xy <= tan(alpha_lim-theta1), va1_r./va1_xy >= tan(-alpha_lim-theta1),...
               va2_r./va2_xy <= tan(alpha_lim-theta2), va2_r./va2_xy >= tan(-alpha_lim-theta2)];

%   control limits
Constraints = [Constraints, u<=ones(nu, gridSz).*u_up, u>=ones(nu, gridSz).*u_lw];

%dynamics and objective
cyl_idx = [[1, 2, 3, 4]];
Objective = u(1, 1).*x(2, 1) + abs(u(1, 1)-u(1, end))*0.01+abs(u(2, 1)-u(2, end))*0.1+abs(u(3, 1)-u(3, end))*0.1;
x_dotk = dynamic_twin(x(:, end), u(:, end), p);
x_dotk1 = dynamic_twin(x(:, 1), u(:, 1), p);
Constraints = [Constraints, x(cyl_idx, 1) == x(cyl_idx, end)+dt/2*(x_dotk(cyl_idx)+x_dotk1(cyl_idx))];
for m = 1 : gridSz-1
  x_dotk = dynamic_twin(x(:, m), u(:, m), p);
  x_dotk1 = dynamic_twin(x(:, m+1), u(:, m+1), p);
  Constraints = [Constraints, x(:, m+1) == x(:, m)+dt/2*(x_dotk+x_dotk1)];%discrete time dynamics, equality cnst
  Objective = Objective + u(1, m+1).*x(2, m+1)+abs(u(1, m+1)-u(1, m))*0.01+abs(u(2, m+1)-u(2, m))*0.1+abs(u(3, m+1)-u(3, m))*0.1; %objective func, max energy
end

% Set some options for YALMIP and solver
options = sdpsettings('solver','ipopt');

% Solve the problem
sol = optimize(Constraints,Objective,options);

% Analyze error flags
if sol.problem == 0
 % Extract and display value
 states = value(x);
 ctrs = value(u);
 plot_aerogen_twin(states,ctrs,p,timeVec)
 disp(-value(Objective)/gridSz*(tf/60/60))

else
 disp('Hmm, something went wrong!');
 sol.info
 yalmiperror(sol.problem)
end
