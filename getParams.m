function p = getParams()
% GETPARAMS  Return default parameter vector for airborne wind energy system.
%
%   p = getParams()
%
%   Returns a parameter vector containing physical properties, aerodynamic
%   coefficients, and control limits for the kite-based wind energy system.
%
%   Output:
%       p - Parameter vector [24 x 1] with the following indices:
%           1:  r_gen       - Generator drum radius [m]
%           2:  moi_g       - Generator moment of inertia [kg*m^2]
%           3:  m_ac        - Aircraft mass [kg]
%           4:  rho         - Air density [kg/m^3]
%           5:  S           - Wing reference area [m^2]
%           6:  CL0         - Lift coefficient at zero angle of attack
%           7:  CLa         - Lift curve slope [1/rad]
%           8:  CD0         - Parasitic drag coefficient
%           9:  e           - Oswald efficiency factor (induced drag)
%           10: vw          - Wind speed [m/s]
%           11: CYb         - Side force coefficient w.r.t. sideslip [1/rad]
%           12: g           - Gravitational acceleration [m/s^2]
%           13: m_teth      - Tether mass [kg]
%           14: CD_eff_teth - Effective tether drag coefficient
%           15: alpha_min   - Minimum angle of attack limit [deg]
%           16: alpha_max   - Maximum angle of attack limit [deg]
%           17: CDds        - Spoiler drag coefficient
%           18: omega       - Buoyancy correction factor
%           19: tf          - Final time (set by optimizer)
%           20: dt          - Time step (set by optimizer)
%           21: beta_lim    - Sideslip angle limit [deg]
%           22: m_ctr_mag   - Motor torque scaling magnitude [N*m]
%           23: de_mag      - Elevator deflection scaling magnitude [rad/s]
%           24: dr_mag      - Rudder deflection scaling magnitude [rad/cycle]

    %% Generator and Tether Properties
    r_gen = 0.05;           % Generator drum radius [m]
    moi_g = 0.01;           % Generator moment of inertia [kg*m^2]
    m_ac = 1;               % Aircraft mass [kg]
    m_teth = 20;            % Tether mass [kg]
    CD_eff_teth = 0.12;     % Effective tether drag coefficient

    %% Atmospheric Properties
    rho = 1.23;             % Air density [kg/m^3] (sea level, standard)
    vw = 12.0;              % Wind speed [m/s]
    g = 9.81;               % Gravitational acceleration [m/s^2]

    %% Aerodynamic Coefficients
    S = 5.0;                % Wing reference area [m^2]
    CL0 = 0.3;              % Lift coefficient at zero AoA
    CLa = 5.0;              % Lift curve slope [1/rad]
    CD0 = 0.02;             % Parasitic drag coefficient
    e = 0.02;               % Oswald efficiency factor (induced drag)
    CYb = -6.3;             % Side force coefficient w.r.t. sideslip [1/rad]
    CDds = 1.0;             % Spoiler drag coefficient, not used
    omega = 0.0;            % Buoyancy correction factor, zero

    %% Flight Envelope Limits
    alpha_min = -10;        % Minimum angle of attack [deg]
    alpha_max = 18;         % Maximum angle of attack [deg]
    beta_lim = 15;          % Sideslip angle limit [deg]

    %% Time Parameters (placeholders, set by optimizer)
    tf = 0;                 % Final time [s]
    dt = 0;                 % Time step [s]

    %% Control Scaling Magnitudes
    m_ctr_mag = 30000;      % Motor torque magnitude [N*m]
    de_mag = 1;             % Elevator deflection magnitude [rad/s]
    dr_mag = 4*pi;          % Rudder deflection magnitude [rad/cycle] (4*pi per cycle)

    %% Assemble Parameter Vector
    % Order must match indices documented in function header
    p = [r_gen; moi_g; m_ac; rho; S; CL0; CLa; CD0; e; vw; CYb; g; m_teth; CD_eff_teth; alpha_min; alpha_max; CDds; omega; tf; dt; beta_lim;m_ctr_mag; de_mag; dr_mag];
end