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
nx = 12; 
nu = 5;
x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:theta1, 4:theta2, 5:va1, 6:va2, 7:psi1, 8:psi2, 9:x1, 10:y1, 11:x2, 12:y2
u = sdpvar(nu, gridSz);%1:m_ctr, 2:de1(theta1_dot), 3:de2(theta2_dot), 4:dr1(psi1_dot), 4:dr2(psi2_dot)
p = getParams(); p(31) = tf; p(32) = dt; p(13) = wind_spd;
if use_guess
    load(strcat(save_name, '_states.mat'));
    load(strcat(save_name, '_ctrs.mat'));
    assign(x, states);
    assign(u, ctrs);
end

%initial/final condition & limits
u_up = [1;  1; 1; 1; 1];%; 0; 0];%; 2*pi/dt+1e-6; 2*pi/dt+1e-6];
u_lw = [-1; -1; -1; -1; -1];%; 0; 0];%; -2*pi/dt-1e-6; -2*pi/dt-1e-6];
x_up = [inf;  inf; pi/4; pi/4; 200; 200; inf; inf; inf; inf; inf; inf];
x_lw = [-inf; -inf; -pi/4; -pi/4; 1; 1; -inf; -inf; -inf; -inf; -inf; -inf];
x0_up = [0;   inf; pi/4; pi/4; 200; 200; pi; pi; 0; 0; 0; 0];
x0_lw = [0;   -inf; -pi/4; -pi/4; 1; 1; -pi; -pi; 0; 0; 0; 0];

%% Contraints & Objective
cyl_idx = [1, 2, 3, 4, 5, 6, 9, 10, 11, 12];
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
    plot_aerogen_twin(states,ctrs,p,timeVec)
    if save_soln
        save(strcat(save_name, '_states.mat'), 'states')
        save(strcat(save_name, '_ctrs.mat'), 'ctrs')
    end
else
    disp('Hmm, something went wrong!');
    sol.info
    yalmiperror(sol.problem)
end
