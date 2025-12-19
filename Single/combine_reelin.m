clc
close all
clear

num_loops = 6;
result_set = 'res23';

reelin_states_files = dir(fullfile(strcat('../../aerogen_yalmip_results/Single/', result_set), '*reelin_states.mat'));
reelin_ctrs_files = dir(fullfile(strcat('../../aerogen_yalmip_results/Single/', result_set), '*reelin_ctrs.mat'));
for k = 1:length(reelin_states_files)
    if reelin_states_files(k).isdir
        continue;
    end
    reelin_state_file = reelin_states_files(k);
    reelin_ctr_file = reelin_ctrs_files(k);
    state_file = reelin_states_files(k);
    ctr_file = reelin_ctrs_files(k);

    state_file.name = erase(state_file.name,'_reelin');
    ctr_file.name = erase(ctr_file.name,'_reelin');
    
    state_path = fullfile(state_file.folder, state_file.name);
    ctr_path = fullfile(ctr_file.folder, ctr_file.name);

    reelin_state_path = fullfile(reelin_ctr_file.folder, reelin_ctr_file.name);
    reelin_ctr_path = fullfile(reelin_ctr_file.folder, reelin_ctr_file.name);
    
    load(state_path)
    load(ctr_path)
    name = state_file.name;
    vals = extract(name, digitsPattern);
    mass = str2num(vals{1});
    wind = str2num(vals{2});
%     pwr_mesh(find(wind_spd_vec == wind), find(m_ac_vec == mass)) = mean(-ctrs(1, :).*20000.*states(2, :)/1000);
end
% [m_ac_mesh, wind_spd_mesh] = meshgrid(m_ac_vec, wind_spd_vec);
% results_name = strcat('Results/', result_set, '_', string(m_ac_vec(1)), 'to', string(m_ac_vec(end)), 'kg_', string(wind_spd_vec(1)), 'to', string(wind_spd_vec(end)), 'mps');
% results.m_ac_mesh = m_ac_mesh;
% results.wind_spd_mesh = wind_spd_mesh;
% results.pwr_mesh = pwr_mesh;
% save(strcat(results_name, '.mat'), 'results')