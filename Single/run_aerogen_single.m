clc
clear
close all

yalmip('clear')
addpath('..\')

save_soln = 0;
use_guess = 1;
save_name = 'Solns/soln_0kg_12mps';
% Define horizon
tf = 10;
gridSz = 60;
dt = tf/(gridSz-1);
timeVec = linspace(0, tf, gridSz);
ctr_obj_gain = 10;
alpha_lim = 18*pi/180;
wind_spd = 12;
m_ac = 6000;
p = getParams(); p(31) = tf; p(32) = dt; p(13) = wind_spd; p(6) = m_ac;

%% Define variables/params
nx = 8; 
nu = 3; 
x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:va1_u, 4: theta, 5:psi, 6:x1, 7:x2, 8:va1_v
u = sdpvar(nu, gridSz);%1:m_ctr, 2:de1(theta_dot), 3:dr1(psi_Dot)
    
%% Contraints & Objective
[Constraints,Objective] = getConstObj_single(gridSz, dt, p, alpha_lim, ctr_obj_gain, nx, nu, x, u);

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
        save(strcat(save_name, '_states.mat'), 'states')
        save(strcat(save_name, '_ctrs.mat'), 'ctrs')
    end
else
    disp('Hmm, something went wrong!');
    sol.info
    yalmiperror(sol.problem)
end
