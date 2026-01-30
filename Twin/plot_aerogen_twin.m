function  plot_aerogen_twin(x,u,p,time)

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
    
    
    % --- Coriolis Terms ---
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
    plot(time, x2, 'LineWidth',2)
    plot(time, y2, 'LineWidth',2)
    title('Position', 'Interpreter', 'latex', 'FontSize',14, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('Position [m]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    legend('$^wX_1$', '$^wY_1$','$^wX_2$', '$^wY_2$', 'Interpreter', 'latex', 'FontSize',10, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, va1_u, 'LineWidth',2)
    plot(time, va2_u, 'LineWidth',2)
    title('$V_a$', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$V_a$ [mps]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    legend('$V_{a1}$', '$V_{a1}$', 'Interpreter', 'latex', 'FontSize',10, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, psi1*180/pi, 'LineWidth',2)
    plot(time, psi2*180/pi, 'LineWidth',2)
    title('$\psi$', 'Interpreter', 'latex', 'FontSize',16, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$\psi$ [deg]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    legend('$\psi_1$', '$\psi_2$', 'Interpreter', 'latex', 'FontSize',10, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, dr1*180/pi, 'LineWidth',2)
    plot(time, dr2*180/pi, 'LineWidth',2)
    title('$\dot{\psi}_c$', 'Interpreter', 'latex', 'FontSize',16, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$\dot{\psi}_c$ [deg/s]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    legend('$\dot{\psi}_{c1}$', '$\dot{\psi}_{c2}$', 'Interpreter', 'latex', 'FontSize',10, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, theta1*180/pi, 'LineWidth',2)
    plot(time, alpha1*180/pi, 'LineWidth',2)
    plot(time, theta2*180/pi, 'LineWidth',2)
    plot(time, alpha2*180/pi, 'LineWidth',2)
    title('$\theta\;\&\;\alpha$', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$\theta\;\&\;\alpha$ [deg]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    legend('$\theta_1$', '$\alpha_1$','$\theta_2$', '$\alpha_2$', 'Interpreter', 'latex', 'FontSize',10, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, de1*180/pi, 'LineWidth',2)
    plot(time, de2*180/pi, 'LineWidth',2)
    title('$\dot{\theta}_c$', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$\dot{\theta}_c$ [deg/s]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    legend('$\dot{\theta}_{c1}$', '$\dot{\theta}_{c2}$', 'Interpreter', 'latex', 'FontSize',10, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, F1_aero_r, 'LineWidth',2)
    plot(time, F2_aero_r, 'LineWidth',2)
    title('$^wF_r$', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$^wF_r$ [N]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    legend('$^wF_{r1}$', '$^wF_{r2}$', 'Interpreter', 'latex', 'FontSize',10, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, beta1*180/pi, 'LineWidth',2)
    plot(time, beta2*180/pi, 'LineWidth',2)
    title('$\beta$', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('$\beta$ [deg]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    legend('$\beta_1$', '$\beta_2$', 'Interpreter', 'latex', 'FontSize',10, 'FontWeight','bold')
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
    plot(time, (-m_ctr.*sigma_dot)/1000, 'LineWidth',2)
    title('Power Generated', 'Interpreter', 'latex', 'FontSize',14, 'FontWeight','bold')
    xlabel('Time [s]')
    ylabel('Power [kW]', 'Interpreter', 'latex', 'FontSize',13, 'FontWeight','bold')
    grid on
    box on
    fig_val = fig_val+1;

    

    sgtitle(strcat('V-Twin Kite States: Wind=', string(vw), 'mps Mass=', string(m_ac), 'kg'), 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    set(gcf, 'Position',  [100, 100, 1200, 1000])
    fprintf('V-Twin Average Power: %2.1f kW\n',sum((-m_ctr(2:end).*x(2, 2:end)/1000).*diff(time))/time(end));

    %% 3D Plot
    figure(2)
    hold on
    grid on
    box on
    axis equal
    view([35 40])
    x1 = x(9, :); y1 = x(1, :)*p(1); z1 = x(10, :);
    x2 = x(11, :); y2 = -x(1, :)*p(1); z2 = x(12, :);

%     plot3(x(9, :), x(10, :), x(1, :)*p(1))
%     plot3(x(11, :), x(12, :), -x(1, :)*p(1))
    plot3([x1 0],ones(size([y1 0]))*0,[z1 0],'k--')
    patch(x1, y1, z1, -m_ctr.*sigma_dot./1000, 'EdgeColor', 'interp', 'LineWidth', 5, 'FaceColor', 'none');
    plot3([x2 0],ones(size([y2 0]))*0,[z2 0],'k--')
    patch(x2, y2, z2, -m_ctr.*sigma_dot./1000, 'EdgeColor', 'interp', 'LineWidth', 5, 'FaceColor', 'none');
    title(strcat('V-Twin Kite Trajectory: Wind=', string(vw), 'mps Mass=', string(m_ac), 'kg'), 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('$^wX$ [m]', 'Interpreter', 'latex', 'FontSize',20, 'FontWeight','bold')
    ylabel('$^wZ$ [m]', 'Interpreter', 'latex', 'FontSize',20, 'FontWeight','bold')
    zlabel('$^wY$ [m]', 'Interpreter', 'latex', 'FontSize',20, 'FontWeight','bold')
%     legend('XYZ1','XYZ2')
    xlim([min([x1 x2])-10, max([x1 x2])+10]);
    ylim([min([y1 y2])-2, max([y1 y2])+2]);
    zlim([min([z1 z2])-10, max([z1 z2])+10]);

    colormap((parula(256)))
    cb = colorbar;
    cb.Label.String = '$P$ [kW]';  % Replace with your label, e.g., 'Altitude (m)'
    cb.Label.Interpreter = 'latex';  % Optional: for math symbols like '$z$ (km)'
    cb.Label.FontSize = 20; 
    set(gcf, 'Position',  [1400, 100, 1000, 1000]);


    n_arrows = 3;
%     idx = round(linspace(1, length(x1)-1, n_arrows));
    idx = mod(round((1:n_arrows)' * length(x1) / n_arrows), length(x1))+1;    
    arrow_pos = [x1(idx)', y1(idx)', z1(idx)'];  % Start points
    arrow_dir = [diff([x1'; 0]), diff([y1'; 0]), diff([z1'; 0])];  % Tangent directions
    
    for i = 1:length(arrow_dir)
        arrow_dir(i, :) = normalize(arrow_dir(i,:), 'norm');
    end
    qv = quiver3(arrow_pos(:,1), arrow_pos(:,2), arrow_pos(:,3), ...
        arrow_dir(idx,1), arrow_dir(idx,2), arrow_dir(idx,3), 0.3, 'AutoScale', 'off');
    qv.LineWidth = 3;
    qv.Color = 'black';
    qv.MaxHeadSize = 10;

    n_arrows = 3;
%     idx = round(linspace(1, length(x1)-1, n_arrows));
    idx = mod(round((1:n_arrows)' * length(x2) / n_arrows), length(x2))+1;    
    arrow_pos = [x2(idx)', y2(idx)', z2(idx)'];  % Start points
    arrow_dir = [diff([x2'; 0]), diff([y2'; 0]), diff([z2'; 0])];  % Tangent directions
    for i = 1:length(arrow_dir)
        arrow_dir(i, :) = normalize(arrow_dir(i,:), 'norm');
    end

    qv = quiver3(arrow_pos(:,1), arrow_pos(:,2), arrow_pos(:,3), ...
        arrow_dir(idx,1), arrow_dir(idx,2), arrow_dir(idx,3), 0.3, 'AutoScale', 'off');
    qv.LineWidth = 3;
    qv.Color = 'black';
    qv.MaxHeadSize = 10;


end