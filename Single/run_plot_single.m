clc
clear
% close all

%%
plot_states = 1;
plot_set = 0;
create_set = 0;

wind_spd = 12;
m_ac = 250;
result_set = 'res15';
if plot_states
    solnPath = strcat('../../aerogen_yalmip_results/Single/', result_set, '/soln_', string(m_ac), 'kg_', string(wind_spd), 'mps');
    load(strcat(solnPath, '_states.mat'))
    load(strcat(solnPath, '_ctrs.mat'))
    
    tf = 10;
    gridSz = 60;
    dt = tf/(gridSz-1);
    timeVec = linspace(0, tf, gridSz);
    p = getParams(); p(19) = tf; p(20) = dt;
    p(10) = wind_spd; p(3) = m_ac;
    plot_aerogen_single(states,ctrs,p,timeVec);
end
%% Create Power Curve
wind_spd_vec = 12:-2:4;
m_ac_vec = 0:25:200;
pwr_mesh = zeros(length(wind_spd_vec), length(m_ac_vec));

if create_set
    states_files = dir(fullfile(strcat('../../aerogen_yalmip_results/Single/', result_set), '*_states.mat'));
    ctrs_files = dir(fullfile(strcat('../../aerogen_yalmip_results/Single/', result_set), '*_ctrs.mat'));
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
    results_name = strcat('Results/', result_set, '_', string(m_ac_vec(1)), 'to', string(m_ac_vec(end)), 'kg_', string(wind_spd_vec(1)), 'to', string(wind_spd_vec(end)), 'mps');
    results.m_ac_mesh = m_ac_mesh;
    results.wind_spd_mesh = wind_spd_mesh;
    results.pwr_mesh = pwr_mesh;
    save(strcat(results_name, '.mat'), 'results')
end

%% Plot Power Curve
if plot_set
    resPath = strcat('Results/', dir(fullfile('Results/',strcat(result_set, '*'))).name);
    load(resPath);
    
    figure(1)
    hold on
    grid on
    zlim([0 350])
    view([45, 45])
    surf(results.m_ac_mesh, results.wind_spd_mesh, results.pwr_mesh);
    title('Single Power Curve')
    xlabel('Mass [kg]')
    ylabel('Wind Speed [mps]')
    zlabel('Average Power [kw]')
    
end