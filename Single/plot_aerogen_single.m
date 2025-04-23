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

%Controls: dist1
m_ctr = u(1, :)*20000;
de1 = u(2, :);
ds1 = 0;
dr1 = u(3, :)*4*pi/(p(31)+p(32));

% extract parameters
r_gen = p(1);
moi_g = p(2);
d = p(5);
m_ac = p(6);
rho = p(7);
S = p(8);
CL0 = p(9);
CLa = p(10);
CD0 = p(11);
e = p(12);
vw = p(13);
moi_ac_m = p(14);
d_c = p(15);
d_b = S/d_c;
CMa = p(16);
CMq = p(17);
CMde = p(18);
CNb = p(19);
CNr = p(20);
CNdr = p(21);
CYb = p(22);
moi_ac_n = p(23);
CDb = p(24);
g = p(25);
m_teth = p(26);
CD_eff_teth = p(27);
CDds = p(29);
omega = p(30);
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

% M1 = q1*d_c*S.*(CMa*alpha1+CMq*theta1_dot+CMde*de1);%no theta dot for now

x1_dot = va1_u.*cos(psi1)+va1_v.*sin(psi1);
y1_dot = va1_u.*sin(psi1)-va1_v.*cos(psi1);
psi1_dot = dr1;
% theta1_dot_dot = M1/moi_ac_m; %no theta dot for now
theta1_dot = de1;
va1_u_dot = (F1_xb.*cos(theta1)+F1_zb.*sin(theta1)-(m_ac+m_teth/2-omega*rho)*g*sin(psi1))/(m_ac+m_teth/3);
va1_v_dot = (-F1_yb+(m_ac+m_teth/2-omega*rho)*g*cos(psi1))/(m_ac+m_teth/3);
F1_aero_r = F1_xb.*(-sin(theta1))+F1_zb.*(cos(theta1));%done

sigma_dot_dot = (-F1_aero_r*r_gen+m_ctr)/(moi_g+(m_ac+m_teth)*r_gen^2);
    %% Generator
    plot_rows = 6; plot_cols = 2;fig_val = 1;
    figure

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, sigma/(2*pi))
    title('Sigma')
    xlabel('Time [s]')
    ylabel('Sigma [revs]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, sigma_dot/(2*pi))
    title('Sigma dot')
    xlabel('Time [s]')
    ylabel('Sigma dot [rev/s]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, x1)
    plot(time, y1)
    title('Position')
    xlabel('Time [s]')
    ylabel('Position [m]')
    legend('X', 'Y')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, va1_u)
    title('Speed')
    xlabel('Time [s]')
    ylabel('U [mps]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, psi1*180/pi)
    title('Heading')
    xlabel('Time [s]')
    ylabel('Psi [deg]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, dr1*180/pi)
    title('Dr')
    xlabel('Time [s]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, theta1*180/pi)
    plot(time, alpha1*180/pi)
    title('Attitude')
    xlabel('Time [s]')
    ylabel('Attitude [deg]')
    legend('pitch', 'alpha')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, de1*180/pi)
    title('Control [de]')
    xlabel('Time [s]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, m_ctr)
    title('Gen Moment')
    xlabel('Time [s]')
    ylabel('Gen Moment [Nm]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, -m_ctr.*sigma_dot./1000)
    title('Objective')
    xlabel('Time [s]')
    ylabel('Power [kW]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, F1_aero_r)
    title('Aero Force')
    xlabel('Time [s]')
    ylabel('Aero Force[N]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, beta1*180/pi)
    title('Side Slip')
    xlabel('Time [s]')
    ylabel('beta [deg]')
    fig_val = fig_val+1;

    sgtitle(strcat('Single:', string(vw), 'mps-', string(m_ac), 'kg'))
    display(mean(-m_ctr.*x(2, :)/1000))

    %% 3D Plot
    figure
    plot3(x(6, :), x(7, :), x(1, :)*p(1))
    axis equal 
    title(strcat('Single:', string(vw), 'mps-', string(m_ac), 'kg'))
    xlabel('X [m]')
    ylabel('Y [m]')
    zlabel('Z [m]')


end