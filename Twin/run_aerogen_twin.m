%% Cleanup
clc
clear
close all
yalmip('clear')%clears yalmip cache
addpath('..\')%directory with other results for warmstart

%% Run Parameters
save_soln = 0;%set to 1 to save solution to Solns/...
use_guess = 0;%set to 1 to use solution to warm start optimizer
save_name = 'Solns/warmstart_0kg_12mps';
load_name = 'Solns/warmstart_0kg_12mps';
% load_path = strcat('../../aerogen_yalmip_results/Twin/', 'res15', '/');
% load_name = strcat(load_path, 'soln_', string(100),'kg_', string(12), 'mps');

tf = 10;%time horizon
gridSz = 60;%grid size
max_time = 500;%optimizer timeout
dt = tf/(gridSz-1);%time step
timeVec = linspace(0, tf, gridSz);%discrete time vector
ctr_obj_gain = 10;%control objective gain (penalizes large control steps),(10:no warm start, 5:with warm start, 2:solve higher order solution [high mass])
wind_spd = 12;%wind speed
m_ac = 0;%aircraft mass

%% Define variables/params
p = getParams(); p(19) = tf; p(20) = dt; p(10) = wind_spd; p(3) = m_ac;
nx = 14; %number of states
nu = 5; %number of decision variables
x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:theta1, 4:theta2, 5:va1_u, 6:va2_u, 7:psi1, 8:psi2, 9:x1, 10:y1, 11:x2, 12:y2, 13:va1_v, 14:va2_v
u = sdpvar(nu, gridSz);%1:m_ctr, 2:de1(theta1_dot), 3:de2(theta2_dot), 4:dr1(psi1_dot), 4:dr2(psi2_dot)

%% Contraints & Objective
[Constraints,Objective] = getConstObj_twin(gridSz, dt, p, ctr_obj_gain, nx, nu, x, u);

%% Assign warmstart and/or options for YALMIP and solver
if use_guess
    load(strcat(load_name, '_states.mat'));
    load(strcat(load_name, '_ctrs.mat'));
    assign(x, states);
    assign(u, ctrs);
    options = sdpsettings('solver','ipopt', 'usex0', 1, 'ipopt.max_cpu_time', max_time, 'ipopt.tol', 1e-4, 'ipopt.dual_inf_tol', 1e-4, 'ipopt.constr_viol_tol', 1e-4);
else
    options = sdpsettings('solver','ipopt', 'ipopt.max_cpu_time', max_time, 'ipopt.tol', 1e-4, 'ipopt.dual_inf_tol', 1e-4, 'ipopt.constr_viol_tol', 1e-4);
end

%% Solve the problem
sol = optimize(Constraints,Objective,options);

%% Plot Solutions/Analyze error flags
if sol.problem == 0
    states = value(x);%extract states
    ctrs = value(u);%extract decision variable
    plot_aerogen_twin(states,ctrs,p,timeVec);%plot states and trajectory
    if save_soln %save states and decision variables
        save(strcat(save_name, '_states.mat'), 'states')
        save(strcat(save_name, '_ctrs.mat'), 'ctrs')
    end
else
    disp('Hmm, something went wrong!');
    sol.info
    yalmiperror(sol.problem)
end
