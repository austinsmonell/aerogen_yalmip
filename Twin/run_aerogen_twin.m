% RUN_AEROGEN_TWIN  Main script for twin-kite airborne wind energy optimization.
%
%   This script solves the optimal control problem for a twin-kite tethered
%   aircraft system. The two kites operate in counter-phase on a shared tether:
%   when kite 1 reels out (generating power), kite 2 reels in, enabling
%   continuous power generation without a separate reel-in phase.
%
%   The optimization uses YALMIP with IPOPT solver and supports warm-starting
%   from previously saved solutions.
%
%   Usage:
%     1. Set configuration flags in "Run Parameters" section
%     2. Adjust physical parameters (wind speed, aircraft mass, etc.)
%     3. Run the script
%
%   Outputs:
%     - Plots of optimized states and trajectory for both kites
%     - Optionally saves solution to .mat files for warm-starting

%% Cleanup
clc
clear
close all
yalmip('clear')                     % Clear YALMIP internal cache
addpath('..\')                      % Add parent directory (contains getParams.m)

%% Run Parameters
% --- Configuration Flags ---
save_soln = 0;                      % Set to 1 to save solution to Solns/
use_guess = 0;                      % Set to 1 to warm-start from specified solution in load_name

% --- File Paths ---
save_name = 'Solns/warmstart_0kg_12mps';
load_name = 'Solns/warmstart_0kg_12mps';

% --- Optimization Settings ---
tf = 10;                            % Time horizon [s]
gridSz = 60;                        % Number of discretization grid points
max_time = 500;                     % IPOPT solver timeout [s]
dt = tf/(gridSz-1);                 % Time step [s]
timeVec = linspace(0, tf, gridSz);  % Discrete time vector
ctr_obj_gain = 10;                  % Control smoothness penalty weight (10:no warm start, 5:with warm start, 2:high mass)

% --- Physical Parameters ---
wind_spd = 12;                      % Wind speed [m/s]
m_ac = 0;                           % Aircraft mass [kg]

%% Define Decision Variables and Parameters
% Load default parameters and override with user settings
p = getParams(); p(19) = tf; p(20) = dt; p(10) = wind_spd; p(3) = m_ac;

% State and control dimensions
nx = 14;                            % Number of states (7 per kite + 2 shared tether)
nu = 5;                             % Number of controls (1 motor + 2 per kite)

% YALMIP decision variables
x = sdpvar(nx,gridSz);              % States:   [sigma, sigma_dot, theta1, theta2, va1_u, va2_u, psi1, psi2, x1, y1, x2, y2, va1_v, va2_v]
u = sdpvar(nu, gridSz);             % Controls: [m_ctr, de1 (pitch rate), de2 (pitch rate), dr1 (yaw rate), dr2 (yaw rate)]

%% Constraints & Objective
% Build constraint set and objective function
[Constraints,Objective] = getConstObj_twin(gridSz, dt, p, ctr_obj_gain, nx, nu, x, u);

%% Solver Configuration
if use_guess
    % Warm-start: load previous solution as initial guess
    load(strcat(load_name, '_states.mat'));
    load(strcat(load_name, '_ctrs.mat'));
    assign(x, states);
    assign(u, ctrs);
    options = sdpsettings('solver','ipopt', 'usex0', 1, 'ipopt.max_cpu_time', max_time, 'ipopt.tol', 1e-4, 'ipopt.dual_inf_tol', 1e-4, 'ipopt.constr_viol_tol', 1e-4);
else
    % Cold-start: let IPOPT find initial point
    options = sdpsettings('solver','ipopt', 'ipopt.max_cpu_time', max_time, 'ipopt.tol', 1e-4, 'ipopt.dual_inf_tol', 1e-4, 'ipopt.constr_viol_tol', 1e-4);
end

%% Solve Optimization Problem
sol = optimize(Constraints,Objective,options);

%% Process Results
if sol.problem == 0
    % Optimization successful - extract solution
    states = value(x);              % Optimal state trajectory
    ctrs = value(u);                % Optimal control trajectory

    % Visualize results
    plot_aerogen_twin(states,ctrs,p,timeVec);

    % Save solution if requested
    if save_soln
        save(strcat(save_name, '_states.mat'), 'states')
        save(strcat(save_name, '_ctrs.mat'), 'ctrs')
    end
else
    % Optimization failed - display error information
    disp('Hmm, something went wrong!');
    sol.info
    yalmiperror(sol.problem)
end
