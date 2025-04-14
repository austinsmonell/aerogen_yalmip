clc
clear
close all

yalmip('clear')
addpath('..\')

save_soln = 0;
use_guess = 1;
save_name = 'Solns/soln1';
% Define horizon
tf = 10;
gridSz = 60;
dt = tf/(gridSz-1);
timeVec = linspace(0, tf, gridSz);
ctr_obj_gain = 10;
alpha_lim = 30*pi/180;
wind_spd = 12;

% Define variables/params
nx = 7; 
nu = 3; 
x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:va, 4: theta, 5:psi, 6:x1, 7:x2
u = sdpvar(nu, gridSz);%1:m_ctr, 2:de1(theta_dot), 3:dr1(psi_Dot)
p = getParams(); p(31) = tf; p(32) = dt; p(13) = wind_spd;
if use_guess
    load(strcat(save_name, '_states.mat'));
    load(strcat(save_name, '_ctrs.mat'));
    assign(x, states);
    assign(u, ctrs);
end

%initial/final condition & limits
u_up = [0; 1; 1];
u_lw = [-1; -1; -1];
x_up = [inf;  inf; 100; pi/4; inf; inf; inf];
x_lw = [-inf; -inf; 1; -pi/4; -inf; -inf; -inf];
x0_up = [0; inf; 100; pi/4; pi; 0; 0];
x0_lw = [0; 0; 1; -pi/4; -pi;  0; 0];

%% Contraints & Objective
cyl_idx = [2, 3, 4, 6, 7];
[Constraints,Objective] = getConstObj(gridSz, dt, nx, nu, x, u, p, x0_lw, x0_up, x_lw, x_up, u_up, u_lw, alpha_lim, cyl_idx, ctr_obj_gain);

% Set some options for YALMIP and solver
if use_guess
    options = sdpsettings('solver','ipopt', 'usex0', 1);
else
    options = sdpsettings('solver','ipopt');
end

% Solve the problem
sol = optimize(Constraints,Objective,options);

% Analyze error flags
if sol.problem == 0
    % Extract and display value
    states = value(x);
    ctrs = value(u);
    plot_aerogen_single(states,ctrs,p,timeVec)
    if save_soln
        save(strcat(save_name, '_states.mat'), 'states')
        save(strcat(save_name, '_ctrs.mat'), 'ctrs')
    end
else
    disp('Hmm, something went wrong!');
    sol.info
    yalmiperror(sol.problem)
end
