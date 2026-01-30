function  plot_aerogen_single(x,u,p,time)

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
    %% Generator
    plot_rows = 6; plot_cols = 2;fig_val = 1;
    figure
    

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, sigma/(2*pi), 'LineWidth',2)
    title('$\sigma$', 'Interpreter', 'latex', 'FontSize',18, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$\sigma$ [revs]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, sigma_dot/(2*pi), 'LineWidth',2)
    title('$\dot{\sigma}$', 'Interpreter', 'latex', 'FontSize',18, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$\dot{\sigma}$ [rev/s]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, x1, 'LineWidth',2)
    plot(time, y1, 'LineWidth',2)
    title('Position', 'Interpreter', 'latex', 'FontSize',14, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('Position [m]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    legend('$^wX$', '$^wY$', 'Interpreter', 'latex', 'FontSize',10, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, va1_u, 'LineWidth',2)
    title('$V_a$', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$V_a$ [mps]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, psi1*180/pi, 'LineWidth',2)
    title('$\psi$', 'Interpreter', 'latex', 'FontSize',16, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$\psi$ [deg]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, dr1*180/pi, 'LineWidth',2)
    title('$\dot{\psi}_c$', 'Interpreter', 'latex', 'FontSize',16, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$\dot{\psi}_c$ [deg/s]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, theta1*180/pi, 'LineWidth',2)
    plot(time, alpha1*180/pi, 'LineWidth',2)
    title('$\theta\;\&\;\alpha$', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$\theta\;\&\;\alpha$ [deg]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    legend('$\theta$', '$\alpha$', 'Interpreter', 'latex', 'FontSize',10, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, de1*180/pi, 'LineWidth',2)
    title('$\dot{\theta}_c$', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$\dot{\theta}_c$ [deg/s]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, F1_aero_r, 'LineWidth',2)
    title('$^wF_r$', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$^wF_r$ [N]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, beta1*180/pi, 'LineWidth',2)
    title('$\beta$', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$\beta$ [deg]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;
    
    subplot(plot_rows, plot_cols, fig_val)
    plot(time, m_ctr, 'LineWidth',2)
    title('$M_c$', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$M_c$ [Nm]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, -m_ctr.*sigma_dot./1000, 'LineWidth',2)
    title('Power Generated', 'Interpreter', 'latex', 'FontSize',14, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('Power [kW]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    

    sgtitle(strcat('Single-Kite States: Wind=', string(vw), 'mps Mass=', string(m_ac), 'kg'), 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    set(gcf, 'Position',  [100, 100, 1200, 1000])
    fprintf('Single Average Power: %2.1f kW\n',sum((-m_ctr(2:end).*x(2, 2:end)/1000).*diff(time))/time(end));

    %% 3D Plot
    figure
    hold on
    grid on
    box on
    axis equal 
    view([35 40])
    x1 = x(6, :); y1 = -x(1, :)*p(1); z1 = x(7, :);

%     plot3(x1, y1, z1)
    plot3([x1 0],ones(size([y1 0]))*max([x1 0])+2,[z1 0],'k--')
    patch(x1, y1, z1, -m_ctr.*sigma_dot./1000, 'EdgeColor', 'interp', 'LineWidth', 5, 'FaceColor', 'none');
    title(strcat('Single-Kite Trajectory: Wind=', string(vw), 'mps Mass=', string(m_ac), 'kg'), 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('$^wX$ [m]', 'Interpreter', 'latex', 'FontSize',20, 'FontWeight','bold')
    ylabel('$^wZ$ [m]', 'Interpreter', 'latex', 'FontSize',20, 'FontWeight','bold')
    zlabel('$^wY$ [m]', 'Interpreter', 'latex', 'FontSize',20, 'FontWeight','bold')

    xlim([min(x1)-10, max(x1)+10]);
    ylim([min(y1)-2, max(y1)+2]);
    zlim([min(z1)-10, max(z1)+10]);
    
    colormap((parula(256)))
    cb = colorbar;
    cb.Label.String = '$P$ [kW]';  % Replace with your label, e.g., 'Altitude (m)'
    cb.Label.Interpreter = 'latex';  % Optional: for math symbols like '$z$ (km)'
    cb.Label.FontSize = 20;
    set(gcf, 'Position',  [1400, 100, 1000, 1000]);

    n_arrows = 4;
%     idx = round(linspace(1, length(x1)-1, n_arrows));
    idx = mod(round((1:n_arrows)' * length(x1) / n_arrows), length(x1))+1;    
    arrow_pos = [x1(idx)', y1(idx)', z1(idx)'];  % Start points
    arrow_dir = [diff([x1'; 0]), diff([y1'; 0]), diff([z1'; 0])];  % Tangent directions
    for i = 1:length(arrow_dir)
        arrow_dir(i, :) = normalize(arrow_dir(i,:), 'norm');
    end

    qv = quiver3(arrow_pos(:,1), arrow_pos(:,2), arrow_pos(:,3), ...
        arrow_dir(idx,1), arrow_dir(idx,2), arrow_dir(idx,3));
    qv.LineWidth = 3;
    qv.AutoScaleFactor = 0.3;
    qv.Marker = '.';
    qv.MarkerFaceColor = 'black';
    qv.Color = 'black';
    qv.MaxHeadSize = 0.3;
    

end