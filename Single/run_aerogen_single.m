clc
clear
close all

yalmip('clear')
addpath('..\')

save_soln = 1;
use_guess = 0;
solve_reelin = 1;
num_loops = 6;
save_path = 'Solns/';
save_name = 'Solns/warmstart_0kg_12mps';
% Define horizon
tf = 10;
gridSz = 80;
dt = tf/(gridSz-1);
timeVec = linspace(0, tf, gridSz);
ctr_obj_gain = 10;
alpha_lim = 18*pi/180;
wind_spd = 12;
m_ac = 0;
p = getParams(); p(19) = tf; p(20) = dt; p(10) = wind_spd; p(3) = m_ac;

%% Define variables/params
nx = 8; 
nu = 3; 
x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:va1_u, 4: theta, 5:psi, 6:x1, 7:x2, 8:va1_v
u = sdpvar(nu, gridSz);%1:m_ctr, 2:de1(theta_dot), 3:dr1(psi_Dot)

    
%% Contraints & Objective

x0 = [];
xf = [];
u0 = [];
if solve_reelin
    load_name = strcat(save_path, 'warmstart_', string(m_ac),'kg_', string(wind_spd), 'mps');
    load(strcat(load_name, '_states.mat'));
    load(strcat(load_name, '_ctrs.mat'));
    x0 = states(:, end);
    xf = states(:, 1);
    x0(1) = x0(1)*num_loops;
    x0(5) = x0(5)*num_loops;
    xf(5) = x0(5);
    u0 = ctrs(:, 1);
end
[Constraints,Objective] = getConstObj_single(gridSz, dt, p, ctr_obj_gain, nx, nu, x, u, solve_reelin, x0, xf, u0);

%% Set some options for YALMIP and solver
if use_guess
    load(strcat(save_name, '_states.mat'));
    load(strcat(save_name, '_ctrs.mat'));
    assign(x, states);
    assign(u, ctrs);
    options = sdpsettings('solver','ipopt', 'usex0', 1);
else
    options = sdpsettings('solver','ipopt');
end

%% Solve the problem
sol = optimize(Constraints,Objective,options);

%% Analyze error flags
if sol.problem == 0
    % Extract and display value
    states = value(x);
    ctrs = value(u);
    plot_aerogen_single(states,ctrs,p,timeVec)
    if save_soln
            save_name = strcat(save_path, 'warmstart_', string(m_ac),'kg_', string(wind_spd), 'mps');
        if solve_reelin
            save(strcat(save_name, '_reelin_states.mat'), 'states')
            save(strcat(save_name, '_reelin_ctrs.mat'), 'ctrs')
        else
            save(strcat(save_name, '_states.mat'), 'states')
            save(strcat(save_name, '_ctrs.mat'), 'ctrs')
        end
    end
else
    disp('Hmm, something went wrong!');
    sol.info
    yalmiperror(sol.problem)
end
