clc
clear
% close all

%%
plot_states = 1;
plot_set = 1;
create_set = 1;
plot_curve = 0;
use_full_cyl = 1;
plot_set_mode = 2;%1:reelin 2:full

wind_spd = 12;
m_ac = 140;
result_set = 'res23';
if plot_states
    solnPath = strcat('../../aerogen_yalmip_results/Single/', result_set, '/soln_', string(m_ac), 'kg_', string(wind_spd), 'mps');
    if plot_set_mode == 2
        load(strcat(solnPath, '_full_states.mat'))
        load(strcat(solnPath, '_full_ctrs.mat'))
    elseif plot_set_mode == 1
        load(strcat(solnPath, '_reelin_states.mat'))
        load(strcat(solnPath, '_reelin_ctrs.mat'))
    else
        load(strcat(solnPath, '_states.mat'))
        load(strcat(solnPath, '_ctrs.mat'))
    end

    
    tf = 10;
    gridSz = 60;
    dt = tf/(gridSz-1);
    timeVec = linspace(0, tf*size(states, 2)/gridSz, size(states, 2));
    p = getParams(); p(19) = tf; p(20) = dt;
    p(10) = wind_spd; p(3) = m_ac;
    plot_aerogen_single(states,ctrs,p,timeVec);
end
%% Create Power Curve
wind_spd_vec = 12:-2:4;
m_ac_vec = 0:20:300;
pwr_mesh = zeros(length(wind_spd_vec), length(m_ac_vec));

if create_set
    if use_full_cyl   
        states_files = dir(fullfile(strcat('../../aerogen_yalmip_results/Single/', result_set), '*_full_states.mat'));
        ctrs_files = dir(fullfile(strcat('../../aerogen_yalmip_results/Single/', result_set), '*_full_ctrs.mat'));
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
        pwr_mesh(find(wind_spd_vec == wind), find(m_ac_vec == mass)) = mean(-ctrs(1, :).*20000.*states(2, :)/1000);
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
    
    figure(1)
    hold on
    grid on
    box on
    zlim([0 100])
    view([45, 45])
    for i = 1:size(results.pwr_mesh, 1)
        for j = 1:size(results.pwr_mesh, 2)
            if results.pwr_mesh(i, j) < 0
                results.pwr_mesh(i, j) = 0;
            end
        end
    end
    s=surf(results.m_ac_mesh, results.wind_spd_mesh, results.pwr_mesh);
    title('Single-Kite Power Curve', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    xlabel('Kite Mass [kg]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    ylabel('Wind Speed [mps]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    zlabel('Average Power [kW]', 'Interpreter', 'latex', 'FontSize',15, 'FontWeight','bold')
    set(gcf, 'Position',  [100, 100, 1000, 800]);
    xticks(0:50:300);
    yticks(4:1:12);
    zticks(0:20:100);
    s.FaceColor = 'interp';    % colors vary smoothly across each face
    s.EdgeColor = 'k';      % hides edges for cleaner look
    caxis([0 90])
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