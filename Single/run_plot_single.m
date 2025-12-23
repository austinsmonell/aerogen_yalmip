clc
clear
close all

%%
plot_states = 1;
plot_set = 1;
create_set = 1;
plot_curve = 0;
use_full_cyl = 1;
plot_mode = 2;%0:traction 1:reelin 2:full

wind_spd = 12;
m_ac = 60;
result_set = 'res25';
tf = 10;
gridSz = 60;
dt = tf/(gridSz-1);
p = getParams(); p(19) = tf; p(20) = dt;
p(10) = wind_spd; p(3) = m_ac;

if plot_states
    solnPath = strcat('../../aerogen_yalmip_results/Single/', result_set, '/soln_', string(m_ac), 'kg_', string(wind_spd), 'mps');
    if plot_mode == 2
        load(strcat(solnPath, '_full_states.mat'))
        load(strcat(solnPath, '_full_ctrs.mat'))
        load(strcat(solnPath, '_full_time.mat'))
    elseif plot_mode == 1
        load(strcat(solnPath, '_reelin_states.mat'))
        load(strcat(solnPath, '_reelin_ctrs.mat'))
        timeVec = linspace(0, tf, size(states, 2));
    else
        load(strcat(solnPath, '_states.mat'))
        load(strcat(solnPath, '_ctrs.mat'))
        timeVec = linspace(0, tf, size(states, 2));
    end
    
    
    
    plot_aerogen_single(states,ctrs,p,timeVec);
end
%% Create Power Curve
wind_spd_vec = 12:-2:4;
m_ac_vec = 0:20:400;
pwr_mesh = zeros(length(wind_spd_vec), length(m_ac_vec));

if create_set
    if use_full_cyl   
        states_files = dir(fullfile(strcat('../../aerogen_yalmip_results/Single/', result_set), '*_full_states.mat'));
        ctrs_files = dir(fullfile(strcat('../../aerogen_yalmip_results/Single/', result_set), '*_full_ctrs.mat'));
        time_files = dir(fullfile(strcat('../../aerogen_yalmip_results/Single/', result_set), '*_full_time.mat'));
    else
        states_files = dir(fullfile(strcat('../../aerogen_yalmip_results/Single/', result_set), '*mps_states.mat'));
        ctrs_files = dir(fullfile(strcat('../../aerogen_yalmip_results/Single/', result_set), '*mps_ctrs.mat'));
    end
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
        vals = extract(name, digitsPattern);
        mass = str2num(vals{1});
        wind = str2num(vals{2});
        if use_full_cyl
            time_file = time_files(k);
            time_path = fullfile(time_file.folder, time_file.name);
            load(time_path)
            pwr_mesh(find(wind_spd_vec == wind), find(m_ac_vec == mass)) = sum((-ctrs(1, 2:end).*20000.*states(2, 2:end)/1000).*diff(timeVec))/timeVec(end);
        else
            pwr_mesh(find(wind_spd_vec == wind), find(m_ac_vec == mass)) = mean(-ctrs(1, :).*20000.*states(2, :)/1000);
        end
    end
    [m_ac_mesh, wind_spd_mesh] = meshgrid(m_ac_vec, wind_spd_vec);
    if use_full_cyl
        results_name = strcat('Results/', result_set, 'full_', string(m_ac_vec(1)), 'to', string(m_ac_vec(end)), 'kg_', string(wind_spd_vec(1)), 'to', string(wind_spd_vec(end)), 'mps');
    else
        results_name = strcat('Results/', result_set, '_', string(m_ac_vec(1)), 'to', string(m_ac_vec(end)), 'kg_', string(wind_spd_vec(1)), 'to', string(wind_spd_vec(end)), 'mps');
    end
    results.m_ac_mesh = m_ac_mesh;
    results.wind_spd_mesh = wind_spd_mesh;
    results.pwr_mesh = pwr_mesh;
    save(strcat(results_name, '.mat'), 'results')
end

%% Plot Power Sweep
if plot_set
    if use_full_cyl
        resPath = strcat('Results/', dir(fullfile('Results/',strcat(result_set, 'full*'))).name);
    else
        resPath = strcat('Results/', dir(fullfile('Results/',strcat(result_set, '_*'))).name);
    end
    
    load(resPath);
    
    figure()
    hold on
    grid on
    box on
    zlim([0 120])
    xlim([0 340])
    view([45, 45])
    for i = 1:size(results.pwr_mesh, 1)
        for j = 1:size(results.pwr_mesh, 2)
            if results.pwr_mesh(i, j) < 0
                results.pwr_mesh(i, j) = 0;
            end
        end
    end
    Z = results.pwr_mesh;
    zero_mask = (Z == 0);     % Exact zero locations

    s=surf(results.m_ac_mesh, results.wind_spd_mesh, results.pwr_mesh);
    title('Single-Kite Average Cycle Power', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Kite Mass [kg]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    ylabel('Wind Speed [mps]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    zlabel('Average Power [kW]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    set(gcf, 'Position',  [100, 100, 1000, 800]);
    xticks(0:20:340);
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
    title('Single-Kite Average Power Curve', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Wind Speed [mps]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    ylabel('Average Power [kW]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    set(gcf, 'Position',  [100, 100, 1000, 800]);
    xticks(4:1:12);
    yticks(0:20:100);
end