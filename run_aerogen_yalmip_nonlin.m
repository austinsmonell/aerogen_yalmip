clc
clear
close all

yalmip('clear')


% Define horizon
tf = 10;
gridSz = 30;
dt = tf/gridSz;
timeVec = linspace(0, tf, gridSz+1);

% Define variables/params
nx = 6; 
nu = 3; 
x = sdpvar(nx,gridSz+1);%1:sigma 2:sigma_dot, 3:x1, 4:x1_dot, 5:theta1, 6:theta1_dot, 7:y1, 8:y1_dot, 9:psi1, 10:psi1_dot
u = sdpvar(nu, gridSz);%1:de1, 2:m_ctr, 3:dr1, 4:ds1
p = getParamSingle();

%initial/final condition & limits
u_up = [0; 10; 1];
u_lw = [-10000; -10; -1];
x_up = [inf; inf; inf; inf; inf; 60];
x_lw = [-inf; -inf; -inf; -inf; -inf; 10];
x0_up = [0; 50; 0; 0; 0; 15];
x0_lw = [0; 1; 0; 0; 0; 15];

%% Contraints & Objective
Constraints = [];
%   intial/final conditions

%   initial state limits
Constraints = [Constraints, x(:, 1) >= x0_lw, x(:, 1) <= x0_up];

%   boundary constraints
Constraints = [Constraints, x(1, 1) <= x(1, end), x(2, 1) == x(2, end), abs(x(3, 1) - x(3, end))<= 1e-10, abs(x(4, 1) - x(4, end))<= 1e-10];

%path constraints

%   state limits
Constraints = [Constraints, x<=ones(nx, gridSz+1).*x_up, x>=ones(nx, gridSz+1).*x_lw];

%   other limits
% vw = p(13);
% r_gen = p(1);
% sigma_dot = x(2, :);
% r1_dot = sigma_dot.*r_gen;%done
% va1_r = -r1_dot+vw;%done
% va1_xy = 30;
% Constraints = [Constraints, va1_r ./ va1_xy <= tan(0.3), va1_r ./ va1_xy >= tan(-0.3)];

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
