function [Constraints,Objective] = getConstObj_twin(gridSz, dt, p, ctr_obj_gain, nx, nu, x, u)
% GETCONSTOBJ_TWIN  Build constraints and objective for twin-kite optimal control problem.
%
%   [Constraints, Objective] = getConstObj_twin(gridSz, dt, p, ctr_obj_gain, ...
%                                                nx, nu, x, u)
%
%
%   Inputs:
%       gridSz        - Number of discretization grid points
%       dt            - Time step size [s]
%       p             - Parameter vector (see extract parameters section)
%       ctr_obj_gain  - Weight for control smoothness penalty in objective
%       nx            - Number of states (14)
%       nu            - Number of controls (5)
%       x             - YALMIP state decision variables [nx x gridSz]
%       u             - YALMIP control decision variables [nu x gridSz]
%
%   Outputs:
%       Constraints   - YALMIP constraint set
%       Objective     - YALMIP objective expression (to be maximized for power)
%
%   State vector x (indices 1-14):
%       1:  sigma     - Normalized tether length (reel-out ratio)
%       2:  sigma_dot - Tether reel-out rate [1/s]
%       3:  theta1    - Kite 1 pitch angle [rad]
%       4:  theta2    - Kite 2 pitch angle [rad]
%       5:  va1_u     - Kite 1 apparent wind velocity, body x-component [m/s]
%       6:  va2_u     - Kite 2 apparent wind velocity, body x-component [m/s]
%       7:  psi1      - Kite 1 azimuth/yaw angle [rad]
%       8:  psi2      - Kite 2 azimuth/yaw angle [rad]
%       9:  x1        - Kite 1 position state 1
%       10: y1        - Kite 1 position state 2
%       11: x2        - Kite 2 position state 1
%       12: y2        - Kite 2 position state 2
%       13: va1_v     - Kite 1 apparent wind velocity, body y-component [m/s]
%       14: va2_v     - Kite 2 apparent wind velocity, body y-component [m/s]
%
%   Control vector u (indices 1-5):
%       1: m_ctr - Motor/generator control torque (normalized)
%       2: de1   - Kite 1 elevator deflection (controls pitch rate) [normalized]
%       3: de2   - Kite 2 elevator deflection (controls pitch rate) [normalized]
%       4: dr1   - Kite 1 rudder deflection (controls yaw rate) [normalized]
%       5: dr2   - Kite 2 rudder deflection (controls yaw rate) [normalized]

    %% Bounds Definition
    % Control bounds (normalized to [-1, 1])
    u_up = [1;  1; 1; 1; 1];
    u_lw = [-1; -1; -1; -1; -1];

    % State bounds
    %   - theta1, theta2 (pitch angles) limited to +/- 90 deg
    %   - va1_u, va2_u (airspeeds) must be positive and bounded
    x_up = [inf; inf; pi/2; pi/2; 200; 200; inf; inf; inf; inf; inf; inf; inf; inf];
    x_lw = [-inf; -inf; -pi/2; -pi/2; 1; 1; -inf; -inf; -inf; -inf; -inf; -inf; -inf; -inf];

    % Initial state bounds (for periodic orbit)
    %   - sigma starts at 0 (beginning of cycle)
    %   - psi1, psi2 (azimuth) limited to +/- 180 deg at start
    %   - position states start at origin
    x0_up = [0; inf; pi/2; pi/2; 200; 200; pi; pi; 0; 0; 0; 0; inf; inf];
    x0_lw = [0; -inf; -pi/2; -pi/2; 1; 1; -pi; -pi; 0; 0; 0; 0; -inf; -inf];

    % Indices for cyclic continuity constraints (excludes psi1 and psi2)
    cyl_idx = [1, 2, 3, 4, 5, 6, 9, 10, 11, 12, 13, 14];

    %% Constraints Initialization
    Constraints = [];

    %% Boundary Conditions
    % Initial state within feasible bounds (for periodic orbit optimization)
    Constraints = [Constraints, x(:, 1) >= x0_lw, x(:, 1) <= x0_up];

    %% State Limits (path constraints)
    % Apply state bounds at all grid points
    % Additional constraint: motor torque and reel rate must have opposite signs
    % (ensures power is always being generated, not consumed)
    Constraints = [Constraints, ...
                   x <= ones(nx, gridSz).*x_up, ...
                   x >= ones(nx, gridSz).*x_lw, ...
                   u(1, :).*x(2, :) <= zeros(1, gridSz)];

    %% Extract System Parameters
    % Generator and tether properties
    r_gen = p(1);           % Generator drum radius [m]
    CD_eff_teth = p(14);    % Effective tether drag coefficient

    % Aerodynamic coefficients (same for both kites)
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
    theta1 = x(3, :);       % Kite 1 pitch angle
    theta2 = x(4, :);       % Kite 2 pitch angle
    va1_u = x(5, :);        % Kite 1 airspeed, body x-component
    va2_u = x(6, :);        % Kite 2 airspeed, body x-component
    va1_v = x(13, :);       % Kite 1 airspeed, body y-component
    va2_v = x(14, :);       % Kite 2 airspeed, body y-component

    % Compute tether radial velocity and apparent wind for each kite
    % Note: kites move in opposite directions (counter-phase operation)
    r1_dot = sigma_dot*r_gen;
    va1_r = -r1_dot + vw;   % Kite 1: reeling out when sigma_dot > 0
    va2_r = r1_dot + vw;    % Kite 2: reeling in when sigma_dot > 0

    % Angle of attack calculation for each kite
    alpha1 = va1_r./va1_u + theta1;
    alpha2 = va2_r./va2_u + theta2;

    % Lift and drag coefficients (parabolic drag polar)
    Cl1 = CL0 + CLa*alpha1;
    Cl2 = CL0 + CLa*alpha2;
    Cd1 = CD0 + CD_eff_teth + e*Cl1.^2;
    Cd2 = CD0 + CD_eff_teth + e*Cl2.^2;

    % Aerodynamic forces in body frame - Kite 1
    F1_zb = -sin(alpha1).*Cd1 - cos(alpha1).*Cl1;   % Body z-axis force
    F1_xb = -cos(alpha1).*Cd1 + sin(alpha1).*Cl1;   % Body x-axis force

    % Aerodynamic forces in body frame - Kite 2
    F2_zb = -sin(alpha2).*Cd2 - cos(alpha2).*Cl2;   % Body z-axis force
    F2_xb = -cos(alpha2).*Cd2 + sin(alpha2).*Cl2;   % Body x-axis force

    % Radial component of aerodynamic force (must be tensile for tether)
    F1_aero_r = F1_xb.*(-sin(theta1)) + F1_zb.*(cos(theta1));
    F2_aero_r = F2_xb.*(-sin(theta2)) + F2_zb.*(cos(theta2));

    % Flight envelope constraints for both kites
    Constraints = [Constraints, ...
                   alpha1 <= (alpha_up*pi/180), ...      % Kite 1 max AoA
                   alpha1 >= (alpha_low*pi/180), ...     % Kite 1 min AoA
                   alpha2 <= (alpha_up*pi/180), ...      % Kite 2 max AoA
                   alpha2 >= (alpha_low*pi/180), ...     % Kite 2 min AoA
                   va1_v./va1_u <= beta_lim*pi/180, ...  % Kite 1 max sideslip
                   va1_v./va1_u >= -beta_lim*pi/180, ... % Kite 1 min sideslip
                   va2_v./va2_u <= beta_lim*pi/180, ...  % Kite 2 max sideslip
                   va2_v./va2_u >= -beta_lim*pi/180, ... % Kite 2 min sideslip
                   F1_aero_r <= 0, ...                   % Kite 1 tether tension, relax if having trouble converging without warmstart (>0)
                   F2_aero_r <= 0];                      % Kite 2 tether tension, relax if having trouble converging without warmstart (>0)
    
    %% Control Limits
    Constraints = [Constraints, u<=ones(nu, gridSz).*u_up, u>=ones(nu, gridSz).*u_lw];

    %% Dynamics Constraints (Trapezoidal Collocation)
    % Initialize objective: power output + control smoothness penalty
    % Power = motor torque * reel rate (u(1) * x(2))
    Objective = u(1, 1).*x(2, 1) + sum(abs(u(:, 1) - u(:, end)))*ctr_obj_gain;

    % Evaluate dynamics at trajectory endpoints for periodicity
    x_dotk = dynamic_twin(x(:, end), u(:, end), p);
    x_dotk1 = dynamic_twin(x(:, 1), u(:, 1), p);

    % Cyclic continuity constraints (for periodic power generation orbits)
    % Most states return to initial value after one cycle
    Constraints = [Constraints, ...
        x(cyl_idx, 1) == x(cyl_idx, end) + dt/2*(x_dotk(cyl_idx) + x_dotk1(cyl_idx))];
    % Azimuth angles advance by 2*pi per cycle (kites orbit in opposite directions)
    Constraints = [Constraints, ...
        x([7], 1) + 2*pi == x([7], end) + dt/2*(x_dotk([7]) + x_dotk1([7])), ... % psi1 advances +2*pi
        x([8], 1) - 2*pi == x([8], end) + dt/2*(x_dotk([8]) + x_dotk1([8]))];    % psi2 advances -2*pi

    %% Collocation Loop (trapezoidal integration)
    for m = 1 : gridSz-1
        % Evaluate dynamics at current and next grid points
        x_dotk = dynamic_twin(x(:, m), u(:, m), p);
        x_dotk1 = dynamic_twin(x(:, m+1), u(:, m+1), p);

        % Trapezoidal collocation: x_{k+1} = x_k + dt/2 * (f_k + f_{k+1})
        Constraints = [Constraints, x(:, m+1) == x(:, m) + dt/2*(x_dotk + x_dotk1)];

        % Accumulate objective: power generation + control rate penalty
        Objective = Objective + u(1, m+1).*x(2, m+1) + ...
                    sum(abs(u(:, m+1) - u(:, m)))*ctr_obj_gain;
    end
end

