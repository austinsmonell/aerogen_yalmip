clc
clear
% close all

%%
addpath('../')
plot_states = 1;
plot_set = 0;
create_set = 0;
plot_curve = 0;

wind_spd = 12;
m_ac = 200;
result_set = 'res31';

if plot_states
    
    solnPath = strcat('../../aerogen_yalmip_results/Twin/', result_set, '/soln_', string(m_ac), 'kg_', string(wind_spd), 'mps');
    load(strcat(solnPath, '_states.mat'))
    load(strcat(solnPath, '_ctrs.mat'))
    
    tf = 10;
    gridSz = 60;
    dt = tf/(gridSz-1);
    timeVec = linspace(0, tf, gridSz);
    p = getParams(); p(19) = tf; p(20) = dt;
    p(10) = wind_spd; p(3) = m_ac;
    plot_aerogen_twin(states,ctrs,p,timeVec);
end
%% Create Power Curve
wind_spd_vec = 12:-2:4;
m_ac_vec = 0:20:400;
pwr_mesh = zeros(length(wind_spd_vec), length(m_ac_vec));

if create_set
    states_files = dir(fullfile(strcat('../../aerogen_yalmip_results/Twin/', result_set), '*_states.mat'));
    ctrs_files = dir(fullfile(strcat('../../aerogen_yalmip_results/Twin/', result_set), '*_ctrs.mat'));
    for k = 1:length(states_files)
        if states_files(k).isdir
            continue;
        end
        state_file = states_files(k);
        ctr_file = ctrs_files(k);
        state_path = fullfile(state_file.folder, state_file.name);
        ctr_path = fullfile(ctr_file.folder, ctr_file.name);
        load(state_path)
        load(ctr_path)
        name = state_file.name;
        vals = regexp(name, '[\d.]+', 'match');
        vals = str2double(vals);
        mass = vals(1);
        wind = vals(2);
        
        pwr_mesh(find(wind_spd_vec == wind), find(m_ac_vec == mass)) = mean(-ctrs(1, :).*30000.*states(2, :)/1000);
    end
    [m_ac_mesh, wind_spd_mesh] = meshgrid(m_ac_vec, wind_spd_vec);
    results_name = strcat('Results/', result_set, '_', string(m_ac_vec(1)), 'to', string(m_ac_vec(end)), 'kg_', string(wind_spd_vec(1)), 'to', string(wind_spd_vec(end)), 'mps');
    results.m_ac_mesh = m_ac_mesh;
    results.wind_spd_mesh = wind_spd_mesh;
    results.pwr_mesh = pwr_mesh;
    save(strcat(results_name, '.mat'), 'results')
end

%% Plot Power Sweep
if plot_set
    resPath = strcat('Results/', dir(fullfile('Results/',strcat(result_set, '*'))).name);
    load(resPath);

    figure()
    hold on
    grid on
    box on
    zlim([0 125])
    xlim([0 400])
    view([45, 45])
    Z = results.pwr_mesh;
    zero_mask = (Z == 0);     % Exact zero locations
    s = surf(results.m_ac_mesh, results.wind_spd_mesh, results.pwr_mesh);
    title('V-Twin Kite Average Cycle Power', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Kite Mass [kg]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    ylabel('Wind Speed [mps]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    zlabel('Average Power [kW]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    set(gcf, 'Position',  [100, 100, 1000, 800]);
    xticks(0:20:400);
    yticks(4:1:12);
    zticks(0:20:140);
    s.FaceColor = 'interp';    % colors vary smoothly across each face
    s.EdgeColor = 'k';      % hides edges for cleaner look
    caxis([0 120])
    colormap((parula(256)))
    cb = colorbar;
    cb.Label.String = '$P$ [kW]';  % Replace with your label, e.g., 'Altitude (m)'
    cb.Label.Interpreter = 'latex';  % Optional: for math symbols like '$z$ (km)'
    cb.Label.FontSize = 20;

    hold on;
    infeas = surf(results.m_ac_mesh, results.wind_spd_mesh, Z.*0);
    infeas.FaceColor = '#808080';
    infeas.EdgeColor = 'none';
    infeas.AlphaData = zero_mask*10;    % Match ZData size
    infeas.FaceAlpha = 'flat';      % Smooth blending; use 'flat' for per-face
    legend('', 'Infeasible', 'Interpreter', 'latex', 'FontSize',12, 'FontWeight','bold')
%     infeas.EdgeColor = 'none';        % Optional: hide edges
end

%% Plot Power Curve
if plot_curve
    resPath = strcat('Results/', dir(fullfile('Results/',strcat(result_set, '*'))).name);
    load(resPath);
    
    figure()
    hold on
    grid on
    ylim([0 100])
    for i = 1:size(results.pwr_mesh, 1)
        for j = 1:size(results.pwr_mesh, 2)
            if results.pwr_mesh(i, j) < 0
                results.pwr_mesh(i, j) = 0;
            end
        end
    end
    idx = find(results.m_ac_mesh==m_ac);
    wnd = results.wind_spd_mesh(idx);
    pwr = results.pwr_mesh(idx);
    wnd_sweep = min(wnd):0.1:max(wnd);
    plot(wnd_sweep, interp1(wnd, pwr, wnd_sweep, 'cubic'), '--', 'LineWidth',3);
    scatter(wnd, pwr, 100, 'blue', 'filled', 'o');
    legend('Cubic Interpolation', 'Simulated')
    title('V-Twin Kite Average Power Curve', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Wind Speed [mps]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    ylabel('Average Power [kW]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    set(gcf, 'Position',  [100, 100, 1000, 800]);
    xticks(4:1:12);
    yticks(0:20:100);
end