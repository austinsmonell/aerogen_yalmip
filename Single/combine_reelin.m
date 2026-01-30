% COMBINE_REELIN  Combine traction and reel-in phases into full power cycle trajectories.
%
%   This script processes optimized traction and reel-in phase solutions,
%   concatenating multiple traction cycles followed by the reel-in phase
%   to create complete power generation cycle trajectories.
%
%   The script:
%     1. Loads all reel-in solution files from a result set directory
%     2. Finds corresponding traction phase solutions
%     3. Concatenates num_loops traction cycles (adjusting sigma and psi)
%     4. Appends the reel-in phase
%     5. Saves the combined full-cycle trajectories
%
%   Outputs (for each mass/wind combination):
%     - *_full_states.mat: Combined state trajectory
%     - *_full_ctrs.mat:   Combined control trajectory
%     - *_full_time.mat:   Combined time vector

%% Cleanup
clc
close all
clear

%% Configuration
save_soln = 1;                      % Set to 1 to save combined solutions

% --- Traction Phase Settings ---
tf = 10;                            % Traction phase time horizon [s]
gridSz = 60;                        % Traction phase grid points
dt = tf/(gridSz-1);                 % Traction phase time step [s]

% --- Reel-in Phase Settings ---
tf_reelin = 10;                     % Reel-in phase time horizon [s]
gridSz_reelin = 60;                 % Reel-in phase grid points
dt_reelin = tf_reelin/(gridSz_reelin-1);    % Reel-in phase time step [s]

% --- Cycle Settings ---
p = getParams(); p(19) = tf; p(20) = dt;
num_loops = 6;                      % Number of traction cycles before reel-in

% --- Time Vectors ---
timeVec_traction = linspace(0, tf, gridSz);
timeVec_reelin = linspace(0, tf_reelin, gridSz_reelin);

%% File Paths
result_set = 'res27';               % Result set directory name

set_path = strcat('../../aerogen_yalmip_results/Single/', result_set, '/');

% Find all reel-in solution files in the result set
reelin_states_files = dir(fullfile(set_path, '*reelin_states.mat'));
reelin_ctrs_files = dir(fullfile(set_path, '*reelin_ctrs.mat'));

%% Process Each Solution Pair
for k = 1:length(reelin_states_files)
    % Skip directories
    if reelin_states_files(k).isdir
        continue;
    end

    % --- Get File References ---
    reelin_state_file = reelin_states_files(k);
    reelin_ctr_file = reelin_ctrs_files(k);
    state_file = reelin_states_files(k);
    ctr_file = reelin_ctrs_files(k);

    % Get corresponding traction phase files (remove '_reelin' from name)
    state_file.name = erase(state_file.name,'_reelin');
    ctr_file.name = erase(ctr_file.name,'_reelin');

    % Build full file paths
    state_path = fullfile(state_file.folder, state_file.name);
    ctr_path = fullfile(ctr_file.folder, ctr_file.name);

    reelin_state_path = fullfile(reelin_state_file.folder, reelin_state_file.name);
    reelin_ctr_path = fullfile(reelin_ctr_file.folder, reelin_ctr_file.name);

    % --- Load Traction Phase Solution ---
    load(state_path)
    load(ctr_path)

    % Extract mass and wind speed from filename
    name = state_file.name;
    vals = extract(name, digitsPattern);
    mass = str2num(vals{1});
    wind = str2num(vals{2});
    p(10) = wind; p(3) = mass;

    % --- Concatenate Traction Cycles ---
    states_full = states;
    ctrs_full = ctrs;
    timeVec = timeVec_traction;

    for i = 1:num_loops-1
        states_new = states;
        % Accumulate sigma (index 1) and psi (index 5) across cycles
        states_new([1, 5], :) = states_new([1, 5], :) + states_full([1, 5], end);
        states_full = [states_full, states_new];
        ctrs_full = [ctrs_full, ctrs];
        timeVec = [timeVec, timeVec_traction+timeVec(end)];
    end

    % --- Append Reel-in Phase ---
    load(reelin_state_path)
    load(reelin_ctr_path)

    % Adjust psi for accumulated cycles
    states(5, :) = states(5, :)+states(5, 1)*(num_loops-1);

    % Combine traction and reel-in trajectories
    states = [states_full, states];
    ctrs = [ctrs_full, ctrs];
    timeVec = [timeVec, timeVec_reelin+timeVec(end)];

    % --- Save Combined Solution ---
    if save_soln
        save_name = strcat(set_path, 'soln_', string(mass),'kg_', string(wind), 'mps');

        save(strcat(save_name, '_full_states.mat'), 'states')
        save(strcat(save_name, '_full_ctrs.mat'), 'ctrs')
        save(strcat(save_name, '_full_time.mat'), 'timeVec')
    end
end
