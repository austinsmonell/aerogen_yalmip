function dx = dynamic_twin(x, u, p)
% DYNAMIC_TWIN  Compute state derivatives for twin-kite airborne wind energy system.
%
%   dx = dynamic_twin(x, u, p)
%
%   Computes the continuous-time dynamics dx/dt = f(x, u, p) for a twin-kite
%   tethered aircraft system. The two kites operate in counter-phase: when
%   kite 1 reels out (generating power), kite 2 reels in, and vice versa.
%
%   Inputs:
%       x - State vector [14 x N] (can be vectorized for multiple points)
%       u - Control vector [5 x N]
%       p - Parameter vector (see getParams.m for indices)
%
%   Output:
%       dx - State derivatives [14 x N]
%
%   State vector x:
%       1:  sigma     - Generator angle [rad]
%       2:  sigma_dot - Generator angular rate [rad/s]
%       3:  theta1    - Kite 1 pitch angle [rad]
%       4:  theta2    - Kite 2 pitch angle [rad]
%       5:  va1_u     - Kite 1 apparent wind velocity, body-wind frame x-component [m/s]
%       6:  va2_u     - Kite 2 apparent wind velocity, body-wind frame x-component [m/s]
%       7:  psi1      - Kite 1 azimuth/yaw angle [rad]
%       8:  psi2      - Kite 2 azimuth/yaw angle [rad]
%       9:  x1        - Kite 1 position x-coordinate (wind frame)
%       10: y1        - Kite 1 position y-coordinate (wind frame)
%       11: x2        - Kite 2 position x-coordinate (wind frame)
%       12: y2        - Kite 2 position y-coordinate (wind frame)
%       13: va1_v     - Kite 1 apparent wind velocity, body-wind frame y-component [m/s]
%       14: va2_v     - Kite 2 apparent wind velocity, body-wind frame y-component [m/s]
%
%   Control vector u:
%       1: m_ctr - Motor/generator torque (normalized)
%       2: de1   - Kite 1 elevator deflection (pitch rate command)
%       3: de2   - Kite 2 elevator deflection (pitch rate command)
%       4: dr1   - Kite 1 rudder deflection (yaw rate command)
%       5: dr2   - Kite 2 rudder deflection (yaw rate command)

%% extract variables
% --- Generator States ---
sigma = x(1,:);         % Generator angle [rad]
sigma_dot = x(2,:);     % Generator angular rate [rad/s]

% --- Kite 1 States ---
theta1 = x(3, :);       % Pitch angle [rad]
va1_u = x(5, :);        % Airspeed, body-wind frame x-component [m/s]
psi1 = x(7, :);         % Azimuth angle [rad]
x1 = x(9, :);           % Position x (wind frame)
y1 = x(10, :);          % Position y (wind frame)
va1_v = x(13, :);       % Airspeed, body-wind frame y-component [m/s]

% --- Kite 2 States ---
theta2 = x(4, :);       % Pitch angle [rad]
va2_u = x(6, :);        % Airspeed, body-wind frame x-component [m/s]
psi2 = x(8, :);         % Azimuth angle [rad]
x2 = x(11, :);          % Position x (wind frame)
y2 = x(12, :);          % Position y (wind frame)
va2_v = x(14, :);       % Airspeed, body-wind frame y-component [m/s]

%% Controls
m_ctr = u(1, :)*p(22);                  % Motor torque [N*m] (scaled by m_ctr magnitude)
de1 = u(2, :)*p(23);                    % Kite 1 elevator: pitch rate command [rad/s] (scaled by de magnitude)
de2 = u(3, :)*p(23);                    % Kite 2 elevator: pitch rate command [rad/s] (scaled by de magnitude)
dr1 = u(4, :)*(p(24)/(p(19)+p(20)));    % Kite 1 rudder: yaw rate command [rad/s] (scaled by dr magnitude)
dr2 = u(5, :)*(p(24)/(p(19)+p(20)));    % Kite 2 rudder: yaw rate command [rad/s] (scaled by dr magnitude)
ds1 = 0;                                % Kite 1 spoiler deflection (unused)
ds2 = 0;                                % Kite 2 spoiler deflection (unused)


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
va1_r = -r1_dot+vw;                     % Kite 1 apparent wind, radial [m/s] (reeling out)
va2_r = r1_dot+vw;                      % Kite 2 apparent wind, radial [m/s] (reeling in)
alpha1 = theta1+va1_r./va1_u;           % Kite 1 angle of attack [rad]
alpha2 = theta2+va2_r./va2_u;           % Kite 2 angle of attack [rad]
beta1 = va1_v./va1_u;                   % Kite 1 sideslip angle [rad]
beta2 = va2_v./va2_u;                   % Kite 2 sideslip angle [rad]

% --- Aerodynamic Forces ---
q1 = 0.5*rho*(va1_r.^2+va1_u.^2+va1_v.^2);  % Kite 1 dynamic pressure [Pa]
q2 = 0.5*rho*(va2_r.^2+va2_u.^2+va2_v.^2);  % Kite 2 dynamic pressure [Pa]
Cl1 = (CL0+CLa*alpha1);                 % Kite 1 lift coefficient
L1 = q1*S.*Cl1;                         % Kite 1 lift force [N]
Cl2 = (CL0+CLa*alpha2);                 % Kite 2 lift coefficient
L2 = q2*S.*Cl2;                         % Kite 2 lift force [N]
Cd1 = (CD0+CD_eff_teth+e*Cl1.^2+CDds*ds1);  % Kite 1 drag coefficient
D1 = q1*S.*Cd1;                         % Kite 1 drag force [N]
Cd2 = (CD0+CD_eff_teth+e*Cl2.^2+CDds*ds2);  % Kite 2 drag coefficient
D2 = q2*S.*Cd2;                         % Kite 2 drag force [N]

% --- Body Frame Forces ---
F1_zb = -sin(alpha1).*D1 - cos(alpha1).*L1; % Kite 1 body z-axis force [N]
F1_xb = -cos(alpha1).*D1 + sin(alpha1).*L1; % Kite 1 body x-axis force [N]
F2_zb = -sin(alpha2).*D2 - cos(alpha2).*L2; % Kite 2 body z-axis force [N]
F2_xb = -cos(alpha2).*D2 + sin(alpha2).*L2; % Kite 2 body x-axis force [N]
F1_yb = q1*S.*(CYb*beta1);              % Kite 1 side force [N]
F2_yb = q2*S.*(CYb*beta2);              % Kite 2 side force [N]

% --- Position Dynamics (wind frame) ---
x1_dot = va1_u.*cos(psi1)-va1_v.*sin(psi1);     % Kite 1 wind frame x velocity
y1_dot = -va1_u.*sin(psi1)-va1_v.*cos(psi1);    % Kite 1 wind frame y velocity
x2_dot = va2_u.*cos(psi2)-va2_v.*sin(psi2);     % Kite 2 wind frame x velocity
y2_dot = -va2_u.*sin(psi2)-va2_v.*cos(psi2);    % Kite 2 wind frame y velocity

% --- Attitude Dynamics ---
theta1_dot = de1;                       % Kite 1 pitch rate command
theta2_dot = de2;                       % Kite 2 pitch rate command
psi1_dot = dr1;                         % Kite 1 yaw rate command
psi2_dot = dr2;                         % Kite 2 yaw rate command


% --- Euler Frame Acceleration Terms ---
a_u1 = va1_v.*psi1_dot;                 % Kite 1 coupling: yaw to u-velocity
a_v1 = -va1_u.*psi1_dot;                % Kite 1 coupling: yaw to v-velocity
a_u2 = va2_v.*psi2_dot;                 % Kite 2 coupling: yaw to u-velocity
a_v2 = -va2_u.*psi2_dot;                % Kite 2 coupling: yaw to v-velocity

% --- Effective Masses ---
m_grav = m_ac+m_teth/2-omega*rho;       % Effective gravitational mass (with buoyancy non-infleuntial)
m_inertia = m_ac+m_teth/3;              % Effective inertial mass

% --- Velocity Dynamics (body-wind frame) ---
va1_u_dot = (F1_xb.*cos(theta1)+ F1_zb.*sin(theta1)+m_grav*g*sin(psi1))/m_inertia+a_u1;     % Kite 1 body-wind frame x-velocity derivative
va2_u_dot = (F2_xb.*cos(theta2)+ F2_zb.*sin(theta2)+m_grav*g*sin(psi2))/(m_ac+m_teth/3)+a_u2;   % Kite 2 body-wind frame x-velocity derivative
va1_v_dot = (F1_yb+m_grav*g*cos(psi1))/m_inertia+a_v1;                                      % Kite 1 body-wind frame y-velocity derivative
va2_v_dot = (F2_yb+m_grav*g*cos(psi2))/m_inertia+a_v2;                                      % Kite 2 body-wind frame y-velocity derivative

% --- Generator Dynamics ---
F1_aero_r = F1_xb.*(-sin(theta1))+F1_zb.*(cos(theta1));     % Kite 1 radial aerodynamic force [N]
F2_aero_r = F2_xb.*(-sin(theta2))+F2_zb.*(cos(theta2));     % Kite 2 radial aerodynamic force [N]

% Generator angular acceleration (kite forces act in opposite directions on shared tether)
sigma_dot_dot = ((-F1_aero_r+F2_aero_r)*r_gen+m_ctr)/(moi_g+2*(m_ac+m_teth)*r_gen^2);

%% package derivitives
dx = [sigma_dot;        % 1:  d(sigma)/dt
      sigma_dot_dot;    % 2:  d(sigma_dot)/dt
      theta1_dot;       % 3:  d(theta1)/dt
      theta2_dot;       % 4:  d(theta2)/dt
      va1_u_dot;        % 5:  d(va1_u)/dt
      va2_u_dot;        % 6:  d(va2_u)/dt
      psi1_dot;         % 7:  d(psi1)/dt
      psi2_dot;         % 8:  d(psi2)/dt
      x1_dot;           % 9:  d(x1)/dt
      y1_dot;           % 10: d(y1)/dt
      x2_dot;           % 11: d(x2)/dt
      y2_dot;           % 12: d(y2)/dt
      va1_v_dot;        % 13: d(va1_v)/dt
      va2_v_dot];       % 14: d(va2_v)/dt
end