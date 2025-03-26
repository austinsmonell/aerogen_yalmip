clc
clear
close all

yalmip('clear')


% Define horizon
tf = 10;
gridSz = 60;
dt = tf/(gridSz-1);
timeVec = linspace(0, tf, gridSz);

% Define variables/params
nx = 7; 
nu = 4; 
x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:x1, 4:x1_dot, 5:theta1, 6:theta1_dot, 7:y1, 8:y1_dot, 9:psi1, 10:psi1_dot
u = sdpvar(nu, gridSz);%1:de1, 2:m_ctr, 3:dr1, 4:ds1
p = getParamSingle();

%initial/final condition & limits
u_up = [0;  1; 0.1; 2*pi/tf+1e-6];
u_lw = [-20000; -1; 0; -2*pi/tf-1e-6];
x_up = [inf;  inf; inf; pi/4; inf; inf;  inf];%; inf];
x_lw = [-inf; -inf;  0; -pi/4; -inf; -inf; -inf];%; -inf];
x0_up = [0;   50;   inf; pi/4;   0; 0;    0];%; inf];
x0_lw = [0;   0;  1; -pi/4;   0;  0;    0];%; -inf];

%% Contraints & Objective
Constraints = [];
%   intial/final conditions

%   initial state limits
Constraints = [Constraints, x(:, 1) >= x0_lw, x(:, 1) <= x0_up];

%   boundary constraints
Constraints = [Constraints, x(1, 1) <= x(1, end),...
               abs(x(5, 1) - (x(5, end)-2*pi))<= 1e-6];

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
Objective = u(1, 1).*x(2, 1);
x_dotk = dynamic_single(x(:, end), u(:, end), p);
x_dotk1 = dynamic_single(x(:, 1), u(:, 1), p);
Constraints = [Constraints, x([2, 3, 4, 6, 7], 1) == x([2, 3, 4, 6, 7], end)+dt/2*(x_dotk([2, 3, 4, 6, 7])+x_dotk1([2, 3, 4, 6, 7]))];
for m = 1 : gridSz-1
  x_dotk = dynamic_single(x(:, m), u(:, m), p);
  x_dotk1 = dynamic_single(x(:, m+1), u(:, m+1), p);
  Constraints = [Constraints, x(:, m+1) == x(:, m)+dt/2*(x_dotk+x_dotk1)];%discrete time dynamics, equality cnst
  Objective = Objective + u(1, m+1).*x(2, m+1)+abs(u(2, m)-u(2, m+1))*1000; %objective func, max energy
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
 disp(-value(Objective)/gridSz*(tf/60/60))

else
 disp('Hmm, something went wrong!');
 sol.info
 yalmiperror(sol.problem)
end
