clc
clear
close all

yalmip('clear')
addpath('..\')

save_soln = 0;
use_guess = 0;
save_name = 'Solns/soln_0kg_12mps';
% Define horizon
tf = 10;
gridSz = 60;
max_time = 600;
dt = tf/(gridSz-1);
timeVec = linspace(0, tf, gridSz);
ctr_obj_gain = 10;
alpha_up = 18*pi/180;
alpha_low = -18*pi/180;
wind_spd = 8;
m_ac = 50;
p = getParams(); p(31) = tf; p(32) = dt; p(13) = wind_spd; p(6) = m_ac;

%% Define variables/params
nx = 14; 
nu = 5;
x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:theta1, 4:theta2, 5:va1_u, 6:va2_u, 7:psi1, 8:psi2, 9:x1, 10:y1, 11:x2, 12:y2, 13:va1_v, 14:va2_v
u = sdpvar(nu, gridSz);%1:m_ctr, 2:de1(theta1_dot), 3:de2(theta2_dot), 4:dr1(psi1_dot), 4:dr2(psi2_dot)

%% Contraints & Objective
[Constraints,Objective] = getConstObj_twin(gridSz, dt, p, alpha_low, alpha_up, ctr_obj_gain, nx, nu, x, u);

%% Set some options for YALMIP and solver
if use_guess
    load(strcat(save_name, '_states.mat'));
    load(strcat(save_name, '_ctrs.mat'));
    assign(x, states);
    assign(u, ctrs);
    options = sdpsettings('solver','ipopt', 'usex0', 1, 'ipopt.max_cpu_time', max_time);
else
    options = sdpsettings('solver','ipopt', 'ipopt.max_cpu_time', max_time);
end

%% Solve the problem
sol = optimize(Constraints,Objective,options);

%% Analyze error flags
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
