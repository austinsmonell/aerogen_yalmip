clc
clear
close all

yalmip('clear')


% Define horizon
tf = 5;
gridSz = 50;
dt = tf/gridSz;
timeVec = linspace(0, tf, gridSz+1);

% Define variables/params
nx = 4; 
nu = 2; 
x = sdpvar(nx,gridSz+1);%1:sigma 2:sigma_dot, 3:x1, 4:x1_dot, 5:theta1, 6:theta1_dot, 7:y1, 8:y1_dot, 9:psi1, 10:psi1_dot
u = sdpvar(nu, gridSz);%1:de1, 2:m_ctr, 3:dr1, 4:ds1
p = getParamSingle();

%initial/final condition & limits
u_up = [0; 10];
u_lw = [-400; -10];
x_up = [inf; inf; inf; 50];
x_lw = [-inf; -inf; -inf; -50];
x0_up = [0; 10; 0; 20];
x0_lw = [0; 0; 0; 0];
xf_up = [inf; inf; 0; 20];
xf_lw = [-inf; -inf; 0; 0];
%% Contraints & Objective
Constraints = [];
%   intial/final conditions
%   initial state limits
Constraints = [Constraints, x(:, 1) >= x0_lw, x(:, 1) <= x0_up];
%   final state limits
Constraints = [Constraints, x(:, end) >= xf_lw, x(:, end) <= xf_up];
%   boundary constraints
Constraints = [Constraints, x(2, 1) == x(2, end), x(3, 1) == x(3, end), x(4, 1) == x(4, end)];

%path constraints
%   state limits
Constraints = [Constraints, x<=ones(nx, gridSz+1).*x_up, x>=ones(nx, gridSz+1).*x_lw];

%   control limits
Constraints = [Constraints, u<=ones(nu, gridSz).*u_up, u>=ones(nu, gridSz).*u_lw];

%dynamics and objective
Objective = 0;
for m = 1 : gridSz
  Constraints = [Constraints, x(:, m+1) == x(:, m)+dt*dynamic_single(x(:, m), u(:, m), p)];%discrete time dynamics, equality cnst
  Objective = Objective + u(1, m).*x(2, m); %objective func, max energy
end



% Set some options for YALMIP and solver
options = sdpsettings('solver','ipopt','verbose',1);

% Solve the problem
sol = optimize(Constraints,Objective,options);

% Analyze error flags
if sol.problem == 0
 % Extract and display value
 states = value(x);
 ctrs = value(u);
 plot_aerogen_single(states,ctrs, timeVec)
 disp(-value(Objective)*0.000277778)

else
 disp('Hmm, something went wrong!');
 sol.info
 yalmiperror(sol.problem)
end
