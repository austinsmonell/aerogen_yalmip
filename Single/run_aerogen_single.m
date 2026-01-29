%% Cleanup
clc
clear
close all
yalmip('clear')%clears yalmip cache
addpath('..\')%directory with other results for warmstart

%% Run Parameters
save_soln = 0;%set to 1 to save solution to Solns/...
use_guess = 0;%set to 1 to use solution to warm start optimizer
solve_reelin = 1;%set to 1 to solve reel-in phase for coorisponding traction phase in save_path
num_loops = 6;%number of traction cycles to concatinate for reel-in
save_path = 'Solns/';%location of traction solution for reel-in
save_name = 'Solns/warmstart_0kg_12mps';
load_name = 'Solns/warmstart_0kg_12mps';

tf = 10;%time horizon
gridSz = 60;%grid size
max_time = 500;%optimizer timeout
dt = tf/(gridSz-1);%time step
timeVec = linspace(0, tf, gridSz);%discrete time vector
ctr_obj_gain = 10;%control objective gain (penalizes large control steps),(10:no warm start or warmstart)
wind_spd = 12;%wind speed
m_ac = 0;%aircraft mass

%% Define variables/params
p = getParams(); p(19) = tf; p(20) = dt; p(10) = wind_spd; p(3) = m_ac;
nx = 8;  %number of states
nu = 3;  %number of decision variables
x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:va1_u, 4: theta, 5:psi, 6:x1, 7:x2, 8:va1_v
u = sdpvar(nu, gridSz);%1:m_ctr, 2:de1(theta_dot), 3:dr1(psi_Dot)

    
%% Contraints & Objective
if solve_reelin
    traction_name = strcat(save_path, 'warmstart_', string(m_ac),'kg_', string(wind_spd), 'mps');
    load(strcat(traction_name, '_states.mat'));
    load(strcat(traction_name, '_ctrs.mat'));
    x0 = states(:, end);
    xf = states(:, 1);
    x0(1) = x0(1)*num_loops;
    x0(5) = x0(5)*num_loops;
    xf(5) = x0(5);
    u0 = ctrs(:, 1);
end
[Constraints,Objective] = getConstObj_single(gridSz, dt, p, ctr_obj_gain, nx, nu, x, u, solve_reelin, x0, xf, u0);

%% Assign WarmSet some options for YALMIP and solver
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
    plot_aerogen_single(states,ctrs,p,timeVec);%plot states and trajectory
    if save_soln %save states and decision variables
        save_name = strcat(save_path, 'warmstart_', string(m_ac),'kg_', string(wind_spd), 'mps');
        if solve_reelin%save reel-in phase
            save(strcat(save_name, '_reelin_states.mat'), 'states')
            save(strcat(save_name, '_reelin_ctrs.mat'), 'ctrs')
        else%save traction phase
            save(strcat(save_name, '_states.mat'), 'states')
            save(strcat(save_name, '_ctrs.mat'), 'ctrs')
        end
    end
else
    disp('Hmm, something went wrong!');
    sol.info
    yalmiperror(sol.problem)
end
