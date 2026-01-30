% RUN_AEROGEN_SINGLE  Main script for single-kite airborne wind energy optimization.
%
%   This script solves the optimal control problem for a single-kite tethered
%   aircraft system. It can solve either:
%     - Traction phase: Periodic power-generating orbit (default)
%     - Reel-in phase: Return trajectory after traction cycles
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
%     - Plots of optimized states and trajectory
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
solve_reelin = 0;                   % Set to 1 to solve reel-in phase (requires traction solution)
num_loops = 6;                      % Number of traction cycles to concatenate for reel-in

% --- File Paths ---
save_path = 'Solns/';               % Directory for saving/loading solutions
save_name = 'Solns/warmstart_0kg_12mps';
load_name = 'Solns/warmstart_0kg_12mps';

% --- Optimization Settings ---
tf = 10;                            % Time horizon [s]
gridSz = 60;                        % Number of discretization grid points
max_time = 500;                     % IPOPT solver timeout [s]
dt = tf/(gridSz-1);                 % Time step [s]
timeVec = linspace(0, tf, gridSz);  % Discrete time vector
ctr_obj_gain = 10;                  % Control smoothness penalty weight(10:no warm start or warmstart)

% --- Physical Parameters ---
wind_spd = 12;                      % Wind speed [m/s]
m_ac = 0;                           % Aircraft mass [kg]

%% Define Decision Variables and Parameters
% Load default parameters and override with user settings
p = getParams(); p(19) = tf; p(20) = dt; p(10) = wind_spd; p(3) = m_ac;

% State and control dimensions
nx = 8;                             % Number of states
nu = 3;                             % Number of controls

% YALMIP decision variables
x = sdpvar(nx,gridSz);              % States:   [sigma, sigma_dot, va_u, theta, psi, x, y, va_v]
u = sdpvar(nu, gridSz);             % Controls: [m_ctr, de (pitch rate), dr (yaw rate)]

%% Constraints & Objective
% Initialize boundary conditions (only used for reel-in phase)
x0 = [];xf = []; u0 = [];

if solve_reelin
    % Load traction phase solution to define reel-in boundary conditions
    traction_name = strcat(save_path, 'warmstart_', string(m_ac),'kg_', string(wind_spd), 'mps');
    load(strcat(traction_name, '_states.mat'));
    load(strcat(traction_name, '_ctrs.mat'));

    % Set initial state as end of traction phase (scaled by num_loops)
    x0 = states(:, end);
    xf = states(:, 1);
    x0(1) = x0(1)*num_loops;        % Scale sigma by number of loops
    x0(5) = x0(5)*num_loops;        % Scale psi by number of loops
    xf(5) = x0(5);
    u0 = ctrs(:, 1);
end

% Build constraint set and objective function
[Constraints,Objective] = getConstObj_single(gridSz, dt, p, ctr_obj_gain, nx, nu, x, u, solve_reelin, x0, xf, u0);

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
    plot_aerogen_single(states,ctrs,p,timeVec);

    % Save solution if requested
    if save_soln
        save_name = strcat(save_path, 'warmstart_', string(m_ac),'kg_', string(wind_spd), 'mps');
        if solve_reelin
            % Save reel-in phase solution
            save(strcat(save_name, '_reelin_states.mat'), 'states')
            save(strcat(save_name, '_reelin_ctrs.mat'), 'ctrs')
        else
            % Save traction phase solution
            save(strcat(save_name, '_states.mat'), 'states')
            save(strcat(save_name, '_ctrs.mat'), 'ctrs')
        end
    end
else
    % Optimization failed - display error information
    disp('Hmm, something went wrong!');
    sol.info
    yalmiperror(sol.problem)
end
