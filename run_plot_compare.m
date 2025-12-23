clc 
close all
clear

%%
plot_set = 1;
plot_curve = 1;
use_full_cyl = 1;


wind_spd = 12;
m_ac = 60;
single_set = 'res25';
twin_set = 'res29';
if use_full_cyl
    resPath = strcat('Single/Results/', dir(fullfile('Single/Results/',strcat(single_set, 'full_*'))).name);
else
    resPath = strcat('Single/Results/', dir(fullfile('Single/Results/',strcat(single_set, '*'))).name);
end

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


pwr_single_mesh = results_single.pwr_mesh;
pwr_single_mesh(pwr_single_mesh == 0) = NaN;
pwr_twin_mesh = results_twin.pwr_mesh;
pwr_twin_mesh(pwr_twin_mesh == 0) = NaN;
%interpolate
mass_sweep = 0:20:400;
wnd_sweep = 4:2:12;
[massQ_mesh,wndQ_mesh] = meshgrid(mass_sweep,wnd_sweep);

valid = ~isnan(pwr_single_mesh);
Xv = results_single.m_ac_mesh(valid);
Yv = results_single.wind_spd_mesh(valid);
Vv = pwr_single_mesh(valid);
% 
F = scatteredInterpolant(Xv, Yv, Vv,'linear', 'linear');
pwr_single = F(massQ_mesh, wndQ_mesh);
pwr_single(pwr_single<0) = 0;

valid = ~isnan(pwr_twin_mesh);
Xv = results_twin.m_ac_mesh(valid);
Yv = results_twin.wind_spd_mesh(valid);
Vv = pwr_twin_mesh(valid);
% 
F = scatteredInterpolant(Xv, Yv, Vv,'linear', 'linear');
pwr_twin = F(massQ_mesh, wndQ_mesh);
pwr_twin(pwr_twin<0) = 0;


figure(999)
hold on
surf(massQ_mesh, wndQ_mesh, pwr_single)
surf(massQ_mesh, wndQ_mesh, pwr_twin)

zlim([0 120])

%% Plot Power Sweep
if plot_set
    
    figure()
    hold on
    grid on
    box on
    view([45, 45])
    
    Z = pwr_twin - pwr_single;
    zero_mask = (Z == 0);     % Exact zero locations

    s = surf(massQ_mesh, wndQ_mesh, Z);  % Pass custom C as 4th argument
    
    % Rest of your original formatting
    title('V-Twin Power Increase', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Kite Mass [kg]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    ylabel('Wind Speed [mps]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    zlabel('$Power \Delta$ [kW]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    set(gcf, 'Position', [100, 100, 1000, 800]);
    xticks(0:20:400);
    yticks(4:1:12);
    zticks(-100:5:100);
    zlim([-5 70])
    xlim([0 400])
    s.FaceColor = 'interp';
%     s.EdgeColor = 'k';
    s.EdgeColor = 'none';
    caxis([0 70])  % Extend caxis low end to include -1 (pink), map Z [0 40] to rest
    cb = colorbar;
    cb.Label.String = '$Power\;\Delta$ [kW]';
    cb.Label.Interpreter = 'latex';
    cb.Label.FontSize = 20;

    hold on;
    infeas = surf(massQ_mesh, wndQ_mesh, Z.*0);
    infeas.FaceColor = '#808080';
    infeas.EdgeColor = 'none';
    infeas.AlphaData = zero_mask*10;    % Match ZData size
    infeas.FaceAlpha = 'flat';      % Smooth blending; use 'flat' for per-face
    legend('', 'Infeasible', 'Interpreter', 'latex', 'FontSize',12, 'FontWeight','bold')

%     infeas.EdgeColor = 'none';        % Optional: hide edges




    Z2 = (pwr_twin-pwr_single)*100./(pwr_single);
    saturate_mask = (Z2 > 100);     % Exact zero locations
    figure()
    hold on
    grid on
    view([90, 90])
    s = surf(massQ_mesh, wndQ_mesh, Z2);
    title('V-Twin Percent Power Increase', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Kite Mass [kg]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    ylabel('Wind Speed [mps]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
%     zlabel('$\%Power\;\Delta$', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
%     set(gca,'Xdir','reverse')
    set(gcf, 'Position',  [100, 100, 1000, 800]);
    xticks(0:20:400);
    yticks(4:1:12);
    zticks(0:20:100);
    zlim([-999 99999])
    xlim([0 400])
    s.FaceColor = 'interp';    % colors vary smoothly across each face
    s.EdgeColor = 'none';      % hides edges for cleaner look
    caxis([0 100])
     
    colormap('parula')
    cb = colorbar();
    cb.Label.String = '$\%\;Power\;\Delta$';  % Replace with your label, e.g., 'Altitude (m)'
    cb.Label.Interpreter = 'latex';  % Optional: for math symbols like '$z$ (km)'
    cb.Label.FontSize = 20;

    hold on;

    saturate = surf(massQ_mesh, wndQ_mesh, Z./Z-100);
    saturate.FaceColor = 'red';
    saturate.EdgeColor = 'none';

    infeas = surf(massQ_mesh, wndQ_mesh, Z.*0);
    infeas.FaceColor = '#808080';
    infeas.EdgeColor = 'none';
    infeas.AlphaData = zero_mask*10;    % Match ZData size
    infeas.FaceAlpha = 'flat';      % Smooth blending; use 'flat' for per-face

    
    legend('', 'V-Twin Kite Feasible, Single-Kite Infeasible', 'Infeasible for Both', 'Interpreter', 'latex', 'FontSize',12, 'FontWeight','bold')
%     saturate.AlphaData = saturate_mask*10;    % Match ZData size
%     saturate.FaceAlpha = 'flat';      % Smooth blending; use 'flat' for per-face
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