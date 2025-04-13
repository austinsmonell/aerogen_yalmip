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
nx = 7; 
nu = 4; 
x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:va, 4: theta, 5:psi, 6:x1, 7:x2
u = sdpvar(nu, gridSz);%1:m_ctr, 2:de1(theta_dot), 3:ds1, 4:dr1(psi_Dot)
p = getParams(); p(31) = tf; p(32) = dt;

%initial/final condition & limits
u_up = [0;  1; 1; 1];
u_lw = [-1; -1; 0; -1];
x_up = [inf;  inf; inf; pi/4; inf; inf;  inf];%; inf];
x_lw = [-inf; -inf;  0; -pi/4; -inf; -inf; -inf];%; -inf];
x0_up = [0;   inf;   inf; pi/4;   0; 0;    0];%; inf];
x0_lw = [0;   0;  1; -pi/4;   0;  0;    0];%; -inf];

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
alpha_lim = 30*pi/180;

r1_dot = sigma_dot*r_gen;
va1_r = -r1_dot+vw;
Constraints = [Constraints, va1_r./va1_xy <= tan(alpha_lim-theta1), va1_r./va1_xy >= tan(-alpha_lim-theta1)];

%   control limits
Constraints = [Constraints, u<=ones(nu, gridSz).*u_up, u>=ones(nu, gridSz).*u_lw];

%dynamics and objective
cyl_idx = [2, 3, 4, 6, 7];
Objective = u(1, 1).*x(2, 1)+sum(abs(u(:, 1)-u(:, end)))*1;
x_dotk = dynamic_single(x(:, end), u(:, end), p);
x_dotk1 = dynamic_single(x(:, 1), u(:, 1), p);
Constraints = [Constraints, x(cyl_idx, 1) == x(cyl_idx, end)+dt/2*(x_dotk(cyl_idx)+x_dotk1(cyl_idx)),...
                            x([5], 1)+2*pi == x([5], end)+dt/2*(x_dotk([5])+x_dotk1([5]))];
for m = 1 : gridSz-1
  x_dotk = dynamic_single(x(:, m), u(:, m), p);
  x_dotk1 = dynamic_single(x(:, m+1), u(:, m+1), p);
  Constraints = [Constraints, x(:, m+1) == x(:, m)+dt/2*(x_dotk+x_dotk1)];%discrete time dynamics, equality cnst
  Objective = Objective + u(1, m+1).*x(2, m+1)+sum(abs(u(:, m+1)-u(:, m)))*1; %objective func, max energy
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
    plot_aerogen_single(states,ctrs,p,timeVec)

else
    disp('Hmm, something went wrong!');
    sol.info
    yalmiperror(sol.problem)
end
