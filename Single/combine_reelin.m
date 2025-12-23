clc
close all
clear

save_soln = 1;
tf = 10;
gridSz = 60;
dt = tf/(gridSz-1);
tf_reelin = 10;
gridSz_reelin = 80;
dt_reelin = tf_reelin/(gridSz_reelin-1);
p = getParams(); p(19) = tf; p(20) = dt;
num_loops = 6;
timeVec_traction = linspace(0, tf, gridSz);
timeVec_reelin = linspace(0, tf_reelin, gridSz_reelin);

result_set = 'res25';

set_path = strcat('../../aerogen_yalmip_results/Single/', result_set, '/');
reelin_states_files = dir(fullfile(set_path, '*reelin_states.mat'));
reelin_ctrs_files = dir(fullfile(set_path, '*reelin_ctrs.mat'));
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

    reelin_state_path = fullfile(reelin_state_file.folder, reelin_state_file.name);
    reelin_ctr_path = fullfile(reelin_ctr_file.folder, reelin_ctr_file.name);
    
    load(state_path)
    load(ctr_path)
    name = state_file.name;
    vals = extract(name, digitsPattern);
    mass = str2num(vals{1});
    wind = str2num(vals{2});
    p(10) = wind; p(3) = mass;

    states_full = states;
    ctrs_full = ctrs;
    timeVec = timeVec_traction;
    for i = 1:num_loops-1
        states_new = states;
        states_new([1, 5], :) = states_new([1, 5], :) + states_full([1, 5], end);
        states_full = [states_full, states_new];
        ctrs_full = [ctrs_full, ctrs];
        timeVec = [timeVec, timeVec_traction+timeVec(end)];
    end
    load(reelin_state_path)
    load(reelin_ctr_path)
    states(5, :) = states(5, :)+states(5, 1)*(num_loops-1);%temporary next solve remove
    states = [states_full, states];
    ctrs = [ctrs_full, ctrs];
    timeVec = [timeVec, timeVec_reelin+timeVec(end)];

%     plot_aerogen_single(states_full,ctrs_full,p,timeVec);
    
    if save_soln
        save_name = strcat(set_path, 'soln_', string(mass),'kg_', string(wind), 'mps');
        
        save(strcat(save_name, '_full_states.mat'), 'states')
        save(strcat(save_name, '_full_ctrs.mat'), 'ctrs')
        save(strcat(save_name, '_full_time.mat'), 'timeVec')
    end
end
