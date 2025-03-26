function  plot_aerogen_single(x,u,p,time)
    %% Generator
    plot_rows = 5; plot_cols = 2;fig_val = 1;
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
    plot(time, x(3, :))
    plot(time, x(4, :))
    title('Position')
    xlabel('Time [s]')
    ylabel('Position [m]')
    legend('X', 'Y')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, x(6, :))
    title('Speed')
    xlabel('Time [s]')
    ylabel('Va [mps]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, x(5, :)*180/pi)
    title('Heading')
    xlabel('Time [s]')
    ylabel('Psi [deg]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, u(2, :))
    plot(time, u(3, :))
    title('Control')
    xlabel('Time [s]')
    legend('dr', 'ds')
    fig_val = fig_val+1;

    sigma_dot = x(2, :);
    r_gen = p(1);
    vw = p(13);
    theta1 = x(7, :);
    va1_xy = x(6, :);
    r1_dot = sigma_dot*r_gen;
    va1_r = -r1_dot+vw;
    alpha1 = theta1+atan2(va1_r, va1_xy);%done

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, x(7, :)*180/pi)
    plot(time, alpha1*180/pi)
    title('Attitude')
    xlabel('Time [s]')
    ylabel('Pitch [deg]')
    legend('pitch', 'alpha')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, u(4, :))
    title('Control [de]')
    xlabel('Time [s]')
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

  

    %% 3D Plot
    figure
    plot3(x(3, :), x(4, :), x(1, :)*p(1))
    axis equal 
    title('3D')
    xlabel('X [m]')
    ylabel('Y [m]')


end