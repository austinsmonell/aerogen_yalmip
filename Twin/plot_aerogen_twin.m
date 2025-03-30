function  plot_aerogen_twin(x,u,p,time)

%% extract variables
% States: theta, theta_dot
sigma = x(1,:);
sigma_dot = x(2,:);
theta1 = x(3, :);
theta2 = x(4, :);
va1_xy = x(5, :);
va2_xy = x(6, :);
psi1 = x(7, :);
psi2 = x(8, :);
x1 = x(9, :);
y1 = x(10, :);
x2 = x(11, :);
y2 = x(12, :);

% theta1_dot = x(8, :);

%Controls: dist1
m_ctr = u(1, :);
de1 = u(2, :);
de2 = u(3, :);
dr1 = u(4, :);
dr2 = u(5, :);
ds1 = 0;
ds2 = 0;


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
va2_r = r1_dot+vw;%done
alpha1 = theta1+atan2(va1_r, va1_xy);%done
alpha2 = theta2+atan2(va2_r, va2_xy);%done

q1 = 0.5*rho*(va1_r.^2+va1_xy.^2);%done
q2 = 0.5*rho*(va2_r.^2+va2_xy.^2);%done

Cl1 = (CL0+CLa*alpha1);%done
L1 = q1*S.*Cl1;%done
Cl2 = (CL0+CLa*alpha2);%done
L2 = q2*S.*Cl2;%done

Cd1 = (CD0+CD_eff_teth+e*Cl1.^2+CDds*ds1);
D1 = q1*S.*Cd1;%done
Cd2 = (CD0+CD_eff_teth+e*Cl2.^2+CDds*ds2);
D2 = q2*S.*Cd2;%done

F1_zb = -sin(alpha1).*D1 - cos(alpha1).*L1;%done
F1_xb = -cos(alpha1).*D1 + sin(alpha1).*L1;%done
F2_zb = -sin(alpha2).*D2 - cos(alpha2).*L2;%done
F2_xb = -cos(alpha2).*D2 + sin(alpha2).*L2;%done

x1_dot = va1_xy.*cos(psi1);
y1_dot = va1_xy.*sin(psi1);
psi1_dot = dr1;
x2_dot = va1_xy.*cos(psi2);
y2_dot = va1_xy.*sin(psi2);
psi2_dot = dr2;

theta1_dot = de1;
theta2_dot = de2;
va1_xy_dot = (F1_xb.*cos(theta1)+ F1_zb.*sin(theta1)-(m_ac+m_teth/2-omega*rho)*g*sin(psi1))/(m_ac+m_teth/3);
va2_xy_dot = (F2_xb.*cos(theta2)+ F2_zb.*sin(theta2)-(m_ac+m_teth/2-omega*rho)*g*sin(psi2))/(m_ac+m_teth/3);

F1_aero_r = F1_xb.*(-sin(theta1))+F1_zb.*(cos(theta1));%done
F2_aero_r = F2_xb.*(-sin(theta2))+F2_zb.*(cos(theta2));%done

sigma_dot_dot = ((-F1_aero_r+F2_aero_r)*r_gen+m_ctr)/(moi_g+2*(m_ac+m_teth)*r_gen^2);

    %% Generator
    plot_rows = 6; plot_cols = 2;fig_val = 1;
    figure

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, x(1, :)/(2*pi))
    title('Sigma')
    xlabel('Time [s]')
    ylabel('Sigma [revs]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, x(2, :)/(2*pi))
    title('Sigma dot')
    xlabel('Time [s]')
    ylabel('Sigma dot [rev/s]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, x(9, :))
    plot(time, x(10, :))
    plot(time, x(11, :))
    plot(time, x(12, :))
    title('Position')
    xlabel('Time [s]')
    ylabel('Position [m]')
    legend('X1', 'Y1', 'X2', 'Y2')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, x(5, :))
    plot(time, x(6, :))
    title('Speed')
    xlabel('Time [s]')
    ylabel('Va [mps]')
    legend('vxy1', 'vxy2')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, x(7, :)*180/pi)
    plot(time, x(8, :)*180/pi)
    title('Heading')
    xlabel('Time [s]')
    ylabel('Psi [deg]')
    legend('psi1', 'psi2')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, u(4, :))
    plot(time, u(5, :))
    title('Rudder')
    xlabel('Time [s]')
    legend('dr1', 'dr2')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, x(3, :)*180/pi)
    plot(time, alpha1*180/pi)
    plot(time, x(4, :)*180/pi)
    plot(time, alpha2*180/pi)
    title('Attitude')
    xlabel('Time [s]')
    ylabel('Pitch [deg]')
    legend('pitch1', 'alpha1', 'pitch2', 'alpha2')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, u(2, :))
    plot(time, u(3, :))
    title('Control [de]')
    xlabel('Time [s]')
    legend('de1', 'de2')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, u(1, :))
    title('Gen Moment')
    xlabel('Time [s]')
    ylabel('Gen Moment [Nm]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, (-u(1, :).*x(2, :))/1000)
    title('Objective')
    xlabel('Time [s]')
    ylabel('Power [kW]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, F1_aero_r)
    plot(time, F2_aero_r)
    title('Aero Force')
    xlabel('Time [s]')
    legend('F1aero','F2aero')
    fig_val = fig_val+1;

  

    %% 3D Plot
    figure
    hold on
    plot3(x(9, :), x(10, :), x(1, :)*p(1))
    plot3(x(11, :), x(12, :), -x(1, :)*p(1))
    axis equal 
    title('3D')
    xlabel('X [m]')
    ylabel('Y [m]')
    legend('XYZ1','XYZ2')


end