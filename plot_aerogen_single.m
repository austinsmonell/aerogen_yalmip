function  plot_aerogen_single(states,ctrs,p,time)
    dt = time(2)-time(1);
    %% Generator
    plot_rows = 2; plot_cols = 2;fig_val = 1;
    figure

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, states(1, :)/(2*pi))
    title('Sigma')
    xlabel('Time [s]')
    ylabel('Sigma [revs]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, states(2, :)/(2*pi))
    title('Sigma dot')
    xlabel('Time [s]')
    ylabel('Sigma dot [rev/s]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time(1:end-1), ctrs(1, :))
    title('Gen Moment')
    xlabel('Time [s]')
    ylabel('Gen Moment [Nm]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time(1:end-1), (-ctrs(1, :).*states(2, 1:end-1))/1000)
    title('Objective')
    xlabel('Time [s]')
    ylabel('Power [kW]')
    fig_val = fig_val+1;

    %% Position
    plot_rows = 2; plot_cols = 2;fig_val = 1;
    figure

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, states(3, :))
    plot(time, states(4, :))
    title('Position')
    xlabel('Time [s]')
    ylabel('Position [m]')
    legend('X', 'Y')
    fig_val = fig_val+1;

%     subplot(plot_rows, plot_cols, fig_val)
%     hold on
%     plot(time, states(4, :))
%     plot(time, states(6, :))
%     plot(time, sqrt(states(4, :).^2+states(6, :).^2))
%     title('Speed')
%     xlabel('Time [s]')
%     ylabel('Speed [m/s]')
%     legend('X', 'Y', 'SUM')
%     fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, states(6, :))
    title('Speed')
    xlabel('Time [s]')
    ylabel('Va [mps]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, states(5, :)*180/pi)
    title('Heading')
    xlabel('Time [s]')
    ylabel('Psi [deg]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time(1:end-1), ctrs(2, :))
    plot(time(1:end-1), ctrs(3, :))
    title('Control')
    xlabel('Time [s]')
    legend('dr', 'ds')
    fig_val = fig_val+1;

    %% 3D Plot
    figure
    plot3(states(3, :), states(4, :), states(1, :)*p(1))
    axis equal 
    title('3D')
    xlabel('X [m]')
    ylabel('Y [m]')

    %% Attitude
    plot_rows = 2; plot_cols = 2;fig_val = 1;
    figure

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time, states(7, :)*180/pi)
    title('Attitude')
    xlabel('Time [s]')
    ylabel('Pitch [deg]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    hold on
    plot(time(1:end-1), ctrs(4, :))
    title('Control [de]')
    xlabel('Time [s]')
    fig_val = fig_val+1;
end