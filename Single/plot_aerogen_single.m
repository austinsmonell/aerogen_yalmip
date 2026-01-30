function  plot_aerogen_single(x,u,p,time)

%% extract variables
% States: theta, theta_dot
sigma = x(1,:);
sigma_dot = x(2,:);
va1_u = x(3, :);
theta1 = x(4, :);
psi1 = x(5, :);
x1 = x(6, :);
y1 = x(7, :);
va1_v = x(8, :);

%% Extract and Scale Control Inputs
m_ctr = u(1, :)*p(22);              % Motor torque [N*m] (scaled from normalized)
de1 = u(2, :)*p(23);                    % Elevator: pitch rate command [rad/s]
dr1 = u(3, :)*p(24)/(p(19)+p(20));   % Rudder: yaw rate command [rad/s]
ds1 = 0;   

% extract parameters
r_gen = p(1);
moi_g = p(2);
m_ac = p(3);
rho = p(4);
S = p(5);
CL0 = p(6);
CLa = p(7);
CD0 = p(8);
e = p(9);
vw = p(10);
CYb = p(11);
g = p(12);
m_teth = p(13);
CD_eff_teth = p(14);
CDds = p(17);
omega = p(18);
%% dynamics
r1_dot = sigma_dot*r_gen;%done
va1_r = -r1_dot+vw;%done
alpha1 = theta1+va1_r./va1_u;%done
beta1 = va1_v./va1_u;

q1 = 0.5*rho*(va1_r.^2+va1_u.^2+va1_v.^2);%done
Cl1 = (CL0+CLa*alpha1);%done
L1 = q1*S.*Cl1;%done
Cd1 = (CD0+CD_eff_teth+e*Cl1.^2+CDds*ds1);
D1 = q1*S.*Cd1;%done
F1_zb = -sin(alpha1).*D1 - cos(alpha1).*L1;%done
F1_xb = -cos(alpha1).*D1 + sin(alpha1).*L1;%done
F1_yb = q1*S.*(CYb*beta1);

x1_dot = va1_u.*cos(psi1)-va1_v.*sin(psi1);
y1_dot = -va1_u.*sin(psi1)-va1_v.*cos(psi1);
psi1_dot = dr1;
theta1_dot = de1;
a_u = va1_v.*psi1_dot;
a_v = -va1_u.*psi1_dot;
va1_u_dot = (F1_xb.*cos(theta1)+F1_zb.*sin(theta1)+(m_ac+m_teth/2-omega*rho)*g*sin(psi1))/(m_ac+m_teth/3)+a_u;
va1_v_dot = (F1_yb+(m_ac+m_teth/2-omega*rho)*g*cos(psi1))/(m_ac+m_teth/3)+a_v;
F1_aero_r = F1_xb.*(-sin(theta1))+F1_zb.*(cos(theta1));%done

sigma_dot_dot = (-F1_aero_r*r_gen+m_ctr)/(moi_g+(m_ac+m_teth)*r_gen^2);
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