clc 
close all
clear

%%
plot_set = 1;
plot_curve = 1;

wind_spd = 12;
m_ac = 60;
single_set = 'res21';
twin_set = 'res25';
resPath = strcat('Single/Results/', dir(fullfile('Single/Results/',strcat(single_set, '*'))).name);

load(resPath);
for i = 1:size(results.pwr_mesh, 1)
    for j = 1:size(results.pwr_mesh, 2)
        if results.pwr_mesh(i, j) < 0
            results.pwr_mesh(i, j) = 0;
        end
    end
end
results_single = results;
resPath = strcat('Twin/Results/', dir(fullfile('Twin/Results/',strcat(twin_set, '*'))).name);
load(resPath);
results_twin = results;

%interpolate
mass_sweep = 0:0.1:300;
wnd_sweep = 4:0.01:12;
[massQ_mesh,wndQ_mesh] = meshgrid(mass_sweep,wnd_sweep);

pwr_single = interp2(results_single.m_ac_mesh, results_single.wind_spd_mesh, results_single.pwr_mesh, massQ_mesh,wndQ_mesh);
pwr_twin = interp2(results_twin.m_ac_mesh, results_twin.wind_spd_mesh, results_twin.pwr_mesh, massQ_mesh,wndQ_mesh);

%% Plot Power Sweep
if plot_set
    
    figure()
    hold on
    grid on
    view([45, 45])
    s = surf(massQ_mesh, wndQ_mesh, pwr_twin-pwr_single);
    title('Power Curve Difference', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Kite Mass [kg]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    ylabel('Wind Speed [mps]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    zlabel('Power $\Delta$ [kW]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    set(gcf, 'Position',  [100, 100, 1000, 800]);
    xticks(0:50:300);
    yticks(4:1:12);
    zticks(0:20:100);
    zlim([-1 100])
    s.FaceColor = 'interp';    % colors vary smoothly across each face
    s.EdgeColor = 'k';      % hides edges for cleaner look
    caxis([0 50])

    figure()
    hold on
    grid on
    view([45, 45])
    s = surf(massQ_mesh, wndQ_mesh, (pwr_twin-pwr_single)*100./(pwr_single+0.1));
    title('Power Curve Percent Difference', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Kite Mass [kg]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    ylabel('Wind Speed [mps]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    zlabel('Power $\Delta$ [kW]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    set(gcf, 'Position',  [100, 100, 1000, 800]);
    xticks(0:50:300);
    yticks(4:1:12);
    zticks(0:20:100);
    zlim([-1 9999999])
    s.FaceColor = 'interp';    % colors vary smoothly across each face
    s.EdgeColor = 'none';      % hides edges for cleaner look
    caxis([0 100])
    colorbar
end

%% Plot Power Curve
if plot_curve
    figure()
    hold on
    grid on
    box on
    idx = find(massQ_mesh==m_ac);
    pwr_sgl = pwr_single(idx);
    pwr_twn = pwr_twin(idx);

    wnd_sweep_interp = min(wnd_sweep):0.1:max(wnd_sweep);
    h(1) = plot(wnd_sweep_interp, interp1(wnd_sweep, pwr_sgl, wnd_sweep_interp, 'cubic'), '--', 'LineWidth',4);
    h(1).MarkerFaceColor = '#AFEEEE';
    h(2) = plot(wnd_sweep_interp, interp1(wnd_sweep, pwr_twn, wnd_sweep_interp, 'cubic'), '--', 'LineWidth',4);
    h(2).MarkerFaceColor = '#FAFAD2';
    h(3) = scatter(wnd_sweep, pwr_sgl, 150, 'filled', 'o');
    h(3).MarkerFaceColor = '#008080';
    h(4) = scatter(wnd_sweep, pwr_twn, 150, 'filled', 'o');
    h(4).MarkerFaceColor = '#FFAA00';
    labels = {'Cubic Interpolation', 'Cubic Interpolation', 'Simulated Single-Kite', 'Simulated V-Twin Kite'}; 
    neworder = [3, 1, 4, 2];  % Custom order
    lgd = legend(h(neworder), labels(neworder), 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold');
    lgd.Location = 'northwest';            % top-left inside axes
    lgd.FontSize = 14;                     % increase text size
%     legend('Cubic Interpolation', 'Cubic Interpolation', 'Simulated', 'Simulated')
    title('Single and V-Twin Kite Average Power Curve', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Wind Speed [mps]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    ylabel('Average Power [kW]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    set(gcf, 'Position',  [100, 100, 800, 600]);
    xticks(4:1:12);
    yticks(0:20:100);
end