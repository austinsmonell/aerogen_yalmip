function [Constraints,Objective] = getConstObj_single(gridSz, dt, p, ctr_obj_gain, nx, nu, x, u, solve_reelin, x0, xf, u0)
% GETCONSTOBJ_SINGLE  Build constraints and objective for single-kite optimal control problem.
%
%   [Constraints, Objective] = getConstObj_single(gridSz, dt, p, ctr_obj_gain, ...
%                                                  nx, nu, x, u, solve_reelin, x0, xf, u0)
%
%   Inputs:
%       gridSz        - Number of discretization grid points
%       dt            - Time step size [s]
%       p             - Parameter vector (see extract parameters section)
%       ctr_obj_gain  - Weight for control smoothness penalty in objective
%       nx            - Number of states
%       nu            - Number of controls
%       x             - YALMIP state decision variables [nx x gridSz]
%       u             - YALMIP control decision variables [nu x gridSz]
%       solve_reelin  - Boolean flag: true for reel-in phase, false for power generation
%       x0, xf        - Initial and final state vectors (used when solve_reelin = true)
%       u0            - Initial control vector (used when solve_reelin = true)
%
%   Outputs:
%       Constraints   - YALMIP constraint set
%       Objective     - YALMIP objective expression (to be maximized for power)
%
%   State vector x (indices 1-8):
%       1: sigma      - Normalized tether length (reel-out ratio)
%       2: sigma_dot  - Tether reel-out rate [1/s]
%       3: va1_u      - Apparent wind velocity, body x-component [m/s]
%       4: theta      - Pitch angle [rad]
%       5: psi        - Azimuth/yaw angle [rad]
%       6: x1         - Position state 1
%       7: x2         - Position state 2
%       8: va1_v      - Apparent wind velocity, body y-component [m/s]
%
%   Control vector u (indices 1-3):
%       1: m_ctr      - Motor/generator control torque (normalized)
%       2: de1        - Elevator deflection (controls pitch rate) [normalized]
%       3: dr1        - Rudder deflection (controls yaw rate) [normalized]

    %% Bounds Definition
    % Control bounds (normalized to [-1, 1])
    u_up = [1; 1; 1];
    u_lw = [-1; -1; -1];

    % State bounds
    %   - va1_u (airspeed) must be positive and bounded
    %   - theta (pitch) limited to avoid extreme attitudes
    x_up = [inf;  inf; 200; pi/2; inf; inf; inf; inf];
    x_lw = [-inf; -inf; 1; -pi/2; -inf; -inf; -inf; -inf];

    % Initial state bounds (for power generation phase)
    %   - sigma starts at 0 (beginning of reel-out)
    %   - pitch angle limited to +/- 60 deg at start
    %   - position states start at origin
    x0_up = [0; inf; 200; pi/3; 0; 0; 0; inf];
    x0_lw = [0; 0; 1; -pi/3; 0;  0; 0; -inf];

    % Indices for cyclic continuity constraints (excludes sigma and psi)
    cyl_idx = [2, 3, 4, 6, 7, 8];

    %% Constraints Initialization
    Constraints = [];

    %% Boundary Conditions
    if solve_reelin
        % Reel-in phase: fix initial and final states/controls
        Constraints = [Constraints, x(:, 1) == x0, x(:, 1) == x0];
        Constraints = [Constraints, x(:, end) == xf, x(:, end) == xf];
        Constraints = [Constraints, u(:, 1) == u0, u(:, 1) == u0];
        Constraints = [Constraints, u(:, end) == u0, u(:, end) == u0];
    else
        % Power generation phase: initial state within feasible bounds
        Constraints = [Constraints, x(:, 1) >= x0_lw, x(:, 1) <= x0_up];
    end

    %% State Limits (path constraints)
    Constraints = [Constraints, x<=ones(nx, gridSz).*x_up, x>=ones(nx, gridSz).*x_lw];

    %% Extract System Parameters
    % Generator and tether properties
    r_gen = p(1);           % Generator drum radius [m]
    CD_eff_teth = p(14);    % Effective tether drag coefficient

    % Aerodynamic coefficients
    CL0 = p(6);             % Lift coefficient at zero AoA
    CLa = p(7);             % Lift curve slope [1/rad]
    CD0 = p(8);             % Parasitic drag coefficient
    e = p(9);               % Oswald efficiency factor (induced drag)

    % Environmental and physical constants
    vw = p(10);             % Wind speed [m/s]
    g = p(12);              % Gravitational acceleration [m/s^2]

    % Flight envelope limits (from parameter vector)
    alpha_low = p(15);      % Minimum angle of attack [deg]
    alpha_up = p(16);       % Maximum angle of attack [deg]
    beta_lim = p(21);       % Sideslip angle limit [deg]

    %% Aerodynamic Constraints
    % Extract states for aerodynamic calculations
    sigma_dot = x(2, :);    % Tether reel rate
    theta1 = x(4, :);       % Pitch angle
    va1_u = x(3, :);        % Airspeed, body x-component
    va1_v = x(8, :);        % Airspeed, body y-component

    % Compute tether radial velocity and apparent wind
    r1_dot = sigma_dot*r_gen;
    va1_r = -r1_dot + vw;

    % Angle of attack calculation
    alpha1 = va1_r./va1_u + theta1;

    % Lift and drag coefficients (parabolic drag polar)
    Cl1 = CL0 + CLa*alpha1;
    Cd1 = CD0 + CD_eff_teth + e*Cl1.^2;

    % Aerodynamic forces in body frame
    F1_zb = -sin(alpha1).*Cd1 - cos(alpha1).*Cl1;   % Body z-axis force
    F1_xb = -cos(alpha1).*Cd1 + sin(alpha1).*Cl1;   % Body x-axis force

    % Radial component of aerodynamic force (must be tensile for tether)
    F1_aero_r = F1_xb.*(-sin(theta1)) + F1_zb.*(cos(theta1));

    % Flight envelope constraints
    Constraints = [Constraints, ...
                   alpha1 <= (alpha_up*pi/180), ...      % Max angle of attack
                   alpha1 >= (alpha_low*pi/180), ...     % Min angle of attack
                   va1_v./va1_u <= beta_lim*pi/180, ...  % Max sideslip
                   va1_v./va1_u >= -beta_lim*pi/180, ... % Min sideslip
                   F1_aero_r <= 0];                      % Tether must remain in tension, relax if having trouble converging without warmstart (>0)
    
    %% Control Limits
    Constraints = [Constraints, u<=ones(nu, gridSz).*u_up, u>=ones(nu, gridSz).*u_lw];

    %% Dynamics Constraints (Trapezoidal Collocation)
    % Initialize objective: power output + control smoothness penalty
    % Power = motor torque * reel rate (u(1) * x(2))
    Objective = u(1, 1).*x(2, 1) + sum(abs(u(:, 1) - u(:, end)))*ctr_obj_gain;

    % Evaluate dynamics at trajectory endpoints for periodicity
    x_dotk = dynamic_single(x(:, end), u(:, end), p);
    x_dotk1 = dynamic_single(x(:, 1), u(:, 1), p);

    % Cyclic continuity constraints (for periodic power generation orbits)
    if ~solve_reelin
        % Most states return to initial value after one cycle
        Constraints = [Constraints, ...
            x(cyl_idx, 1) == x(cyl_idx, end) + dt/2*(x_dotk(cyl_idx) + x_dotk1(cyl_idx))];
        % Azimuth angle (psi) advances by 2*pi per cycle
        Constraints = [Constraints, ...
            x([5], 1) + 2*pi == x([5], end) + dt/2*(x_dotk([5]) + x_dotk1([5]))];
    end

    %% Collocation Loop (trapezoidal integration)
    for m = 1 : gridSz-1
        % Evaluate dynamics at current and next grid points
        x_dotk = dynamic_single(x(:, m), u(:, m), p);
        x_dotk1 = dynamic_single(x(:, m+1), u(:, m+1), p);

        % Trapezoidal collocation: x_{k+1} = x_k + dt/2 * (f_k + f_{k+1})
        Constraints = [Constraints, x(:, m+1) == x(:, m) + dt/2*(x_dotk + x_dotk1)];

        % Accumulate objective: power generation + control rate penalty
        Objective = Objective + u(1, m+1).*x(2, m+1) + ...
                    sum(abs(u(:, m+1) - u(:, m)))*ctr_obj_gain;
    end
end

