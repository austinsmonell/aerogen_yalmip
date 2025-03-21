function  plot_aerogen_single(states,ctrs, time)
    dt = time(2)-time(1);
    %%
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

    %%
    plot_rows = 2; plot_cols = 2;fig_val = 1;
    figure

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, states(3, :))
    title('X')
    xlabel('Time [s]')
    ylabel('X [m]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time, states(4, :))
    title('X dot')
    xlabel('Time [s]')
    ylabel('X dot [m/s]')
    fig_val = fig_val+1;

    subplot(plot_rows, plot_cols, fig_val)
    plot(time(1:end-1), ctrs(2, :))
    title('de')
    xlabel('Time [s]')
    ylabel('De []')
    fig_val = fig_val+1;

%     subplot(plot_rows, plot_cols, fig_val)
%     plot(time(1:end-1), (-ctrs(1, :).*states(2, 1:end-1))/dt)
%     title('Objective')
%     xlabel('Time [s]')
%     ylabel('Power [kW]')
%     fig_val = fig_val+1;
end