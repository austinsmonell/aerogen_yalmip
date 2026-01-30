function dx = dynamic_single(x, u, p)
% DYNAMIC_SINGLE  Compute state derivatives for single-kite airborne wind energy system.
%
%   dx = dynamic_single(x, u, p)
%
%   Computes the continuous-time dynamics dx/dt = f(x, u, p) for a single-kite
%   tethered aircraft system used in airborne wind energy generation.
%
%   Inputs:
%       x - State vector [8 x N] (can be vectorized for multiple points)
%       u - Control vector [3 x N]
%       p - Parameter vector (see getParams.m for indices)
%
%   Output:
%       dx - State derivatives [8 x N]
%
%   State vector x:
%       1: sigma      - Generator angle [rad]
%       2: sigma_dot  - Generator angular rate [rad/s]
%       3: va1_u      - Apparent wind velocity, body-wind frame x-component [m/s]
%       4: theta      - Pitch angle [rad]
%       5: psi        - Azimuth/yaw angle [rad]
%       6: x1         - Position x-coordinate (wind frame)
%       7: y1         - Position y-coordinate (wind frame)
%       8: va1_v      - Apparent wind velocity, body-wind frame y-component [m/s]
%
%   Control vector u:
%       1: m_ctr - Motor/generator torque (normalized)
%       2: de1   - Elevator deflection (pitch rate command)
%       3: dr1   - Rudder deflection (yaw rate command)

%% extract variables
sigma = x(1,:);         % Generator angle [rad]
sigma_dot = x(2,:);     % Generator angular rate [rad/s]
va1_u = x(3, :);        % Airspeed, body-wind frame x-component [m/s]
theta1 = x(4, :);       % Pitch angle [rad]
psi1 = x(5, :);         % Azimuth angle [rad]
x1 = x(6, :);           % Position x (wind frame)
y1 = x(7, :);           % Position y (wind frame)
va1_v = x(8, :);        % Airspeed, body-wind frame y-component [m/s]

%% Controls
m_ctr = u(1, :)*p(22);                  % Motor torque [N*m](scaled by m_ctr magnitude)
de1 = u(2, :)*p(23);                    % Elevator: pitch rate command [rad/s] (scaled by de magnitude)
dr1 = u(3, :)*(p(24)/(p(19)+p(20)));    % Rudder: yaw rate command [rad/s] (scaled by dr magnitude)
ds1 = 0;                                % Spoiler deflection (unused)

%% extract parameters
% --- Generator and Tether Properties ---
r_gen = p(1);           % Generator drum radius [m]
moi_g = p(2);           % Generator moment of inertia [kg*m^2]
m_ac = p(3);            % Aircraft mass [kg]
m_teth = p(13);         % Tether mass [kg]
CD_eff_teth = p(14);    % Effective tether drag coefficient

% --- Aerodynamic Coefficients ---
rho = p(4);             % Air density [kg/m^3]
S = p(5);               % Wing reference area [m^2]
CL0 = p(6);             % Lift coefficient at zero AoA
CLa = p(7);             % Lift curve slope [1/rad]
CD0 = p(8);             % Parasitic drag coefficient
e = p(9);               % Oswald efficiency factor (induced drag)
CYb = p(11);            % Side force coefficient w.r.t. sideslip [1/rad]
CDds = p(17);           % Spoiler drag coefficient, zero influence

% --- Environmental Constants ---
vw = p(10);             % Wind speed [m/s]
g = p(12);              % Gravitational acceleration [m/s^2]
omega = p(18);          % Buoyancy correction factor, zero influence

%% dynamics
% --- Kinematics and Aerodynamic Angles ---
r1_dot = sigma_dot*r_gen;               % Tether reel speed [m/s]
va1_r = -r1_dot+vw;                     % Apparent wind, radial component [m/s]
alpha1 = theta1+va1_r./va1_u;           % Angle of attack [rad]
beta1 = va1_v./va1_u;                   % Sideslip angle [rad]

% --- Aerodynamic Forces ---
q1 = 0.5*rho*(va1_r.^2+va1_u.^2+va1_v.^2);  % Dynamic pressure [Pa]
Cl1 = (CL0+CLa*alpha1);                 % Lift coefficient (linear model)
L1 = q1*S.*Cl1;                         % Lift force [N]
Cd1 = (CD0+CD_eff_teth+e*Cl1.^2+CDds*ds1);  % Drag coefficient (parabolic polar)
D1 = q1*S.*Cd1;                         % Drag force [N]

% --- Body Frame Forces ---
F1_zb = -sin(alpha1).*D1 - cos(alpha1).*L1; % Body z-axis force [N]
F1_xb = -cos(alpha1).*D1 + sin(alpha1).*L1; % Body x-axis force [N]
F1_yb = q1*S.*(CYb*beta1);              % Side force [N]

% --- Position Dynamics (wind frame) ---
x1_dot = va1_u.*cos(psi1)-va1_v.*sin(psi1);     % Wind frame x velocity
y1_dot = -va1_u.*sin(psi1)-va1_v.*cos(psi1);    % Wind frame y velocity

% --- Attitude Dynamics ---
psi1_dot = dr1;                         % Yaw rate command
theta1_dot = de1;                       % Pitch rate command

% --- Coriolis Terms ---
a_u = va1_v.*psi1_dot;                  % Coriolis term: yaw to u-velocity
a_v = -va1_u.*psi1_dot;                 % Coriolis term: yaw to v-velocity

% --- Effective Masses ---
m_grav = m_ac+m_teth/2-omega*rho;       % Effective gravitational mass (with buoyancy non-infleuntial)
m_inertia = m_ac+m_teth/3;              % Effective inertial mass

% --- Velocity Dynamics (body-wind frame) ---
va1_u_dot = (F1_xb.*cos(theta1)+F1_zb.*sin(theta1)+m_grav*g*sin(psi1))/m_inertia+a_u;    % Body-wind frame x-velocity derivative
va1_v_dot = (F1_yb+m_grav*g*cos(psi1))/m_inertia+a_v;                                    % Body-wind frame y-velocity derivative

% --- Generator Dynamics ---
F1_aero_r = F1_xb.*(-sin(theta1))+F1_zb.*(cos(theta1));     % Radial aerodynamic force [N]

% Generator angular acceleration
sigma_dot_dot = (-F1_aero_r*r_gen+m_ctr)/(moi_g+(m_ac+m_teth)*r_gen^2);  % Generator angular acceleration

%% package derivitives
dx = [sigma_dot;        % 1: d(sigma)/dt
      sigma_dot_dot;    % 2: d(sigma_dot)/dt
      va1_u_dot;        % 3: d(va1_u)/dt
      theta1_dot;       % 4: d(theta)/dt
      psi1_dot;         % 5: d(psi)/dt
      x1_dot;           % 6: d(x1)/dt
      y1_dot;           % 7: d(y1)/dt
      va1_v_dot];       % 8: d(va1_v)/dt
end