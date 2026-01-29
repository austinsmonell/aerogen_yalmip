%% Cleanup
clc
clear
close all 
yalmip('clear')
addpath('..\')

%% Run Parameters
save_soln = 1;
use_guess = 1;
solve_reelin = 0;
num_loops = 6;
results_run = 'res28';
save_path = strcat('../../aerogen_yalmip_results/Single/', results_run, '/');
load_name = 'Solns/warmstart_0kg_12mps';%initial warm start solution
load_path = strcat('../../aerogen_yalmip_results/Single/', 'res27', '/');
% load_name = strcat(load_path, 'soln_', string(80),'kg_', string(12), 'mps');% uncomment if using warm start from previous solution set (not typically nessesary)

tf = 10;
gridSz = 60;
max_time = 200;
dt = tf/(gridSz-1);
timeVec = linspace(0, tf, gridSz);
ctr_obj_gain = 10;

%% Define variables/params
p = getParams(); p(19) = tf; p(20) = dt;
nx = 8; 
nu = 3; 
pwr_figure = figure;
wind_spd_vec = 12:-2:4;%wind speed sweep vector
m_ac_vec = 0:20:400;%mass sweep vector
pwr_mesh = zeros(length(wind_spd_vec), length(m_ac_vec));
[m_ac_mesh, wind_spd_mesh] = meshgrid(m_ac_vec, wind_spd_vec);
% wind_spd_vec = wind_spd_vec(1);%temp

%% Sweep through mass and wind speed
for i = 1:length(m_ac_vec)
    if i > 1
        load_name = strcat(save_path, 'soln_', string(m_ac_vec(i-1)),'kg_', string(wind_spd_vec(1)), 'mps');
    end
    for j = 1:length(wind_spd_vec)
        if j > 1    
            load_name = strcat(save_path, 'soln_', string(m_ac_vec(i)),'kg_', string(wind_spd_vec(j-1)), 'mps');
        end
        yalmip('clear')%clear yalmip cache
        x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:va, 4: theta, 5:psi, 6:x1, 7:x2
        u = sdpvar(nu, gridSz);%1:m_ctr, 2:de1(theta_dot), 3:dr1(psi_Dot)

        %set windspeed and mass
        p(10) = wind_spd_vec(j);
        p(3) = m_ac_vec(i);

        % Contraints & Objective
        x0 = [];xf = []; u0 = [];
        if solve_reelin
            traction_name = strcat(save_path, 'soln_', string(m_ac_vec(i)),'kg_', string(wind_spd_vec(j)), 'mps');
            if isempty(dir(fullfile(strcat(traction_name, '_states.mat'))))
                break;%skip if infeasible
            end
            load(strcat(traction_name, '_states.mat'));
            load(strcat(traction_name, '_ctrs.mat'));
            x0 = states(:, end);
            xf = states(:, 1);
            x0(1) = x0(1)*num_loops;
            xf(5) = x0(5);
            u0 = ctrs(:, 1);
        end
        [Constraints,Objective] = getConstObj_single(gridSz, dt, p, ctr_obj_gain, nx, nu, x, u, solve_reelin, x0, xf, u0);
        
        % Assign warmstart and/or options for YALMIP and solver
        if use_guess
            
            if solve_reelin % solve reel_in
                if isempty(dir(fullfile(strcat(load_name, '_reelin_states.mat'))))
                    break; %if previous wind speed optimization failed break the wind speed loop
                end
                load(strcat(load_name, '_reelin_states.mat'))
                load(strcat(load_name, '_reelin_ctrs.mat'))
            else% solve traction
                if isempty(dir(fullfile(strcat(load_name, '_states.mat'))))
                    break; %if previous wind speed optimization failed break the wind speed loop
                end
                load(strcat(load_name, '_states.mat'))
                load(strcat(load_name, '_ctrs.mat'))
            end
            assign(x, states);
            assign(u, ctrs);
            options = sdpsettings('solver','ipopt', 'usex0', 1, 'ipopt.max_cpu_time', max_time, 'ipopt.tol', 1e-4, 'ipopt.dual_inf_tol', 1e-4, 'ipopt.constr_viol_tol', 1e-4);
        else
            options = sdpsettings('solver','ipopt', 'ipopt.max_cpu_time', max_time, 'ipopt.tol', 1e-4, 'ipopt.dual_inf_tol', 1e-4, 'ipopt.constr_viol_tol', 1e-4);
        end
        
        % Solve the problem
        sol = optimize(Constraints,Objective,options);
        
        % Plot Solutions/Analyze error flags
        if sol.problem == 0
            states = value(x);
            ctrs = value(u);
%             plot_aerogen_single(states,ctrs,p,timeVec);

            pwr_mesh(j, i) = mean(-u(1, :).*20000.*x(2, :)/1000);
            close(pwr_figure);
            pwr_figure = figure;
            surf(m_ac_mesh, wind_spd_mesh, pwr_mesh);
            title('Single Power Curve')
            xlabel('Mass [kg]')
            ylabel('Wind Speed [mps]')
            zlabel('Average Power [kW]')
            view([45, 45]);
            drawnow;

            if save_soln
                save_name = strcat(save_path, 'soln_', string(m_ac_vec(i)),'kg_', string(wind_spd_vec(j)), 'mps');
                if solve_reelin
                    save(strcat(save_name, '_reelin_states.mat'), 'states')
                    save(strcat(save_name, '_reelin_ctrs.mat'), 'ctrs')
                else
                    save(strcat(save_name, '_states.mat'), 'states')
                    save(strcat(save_name, '_ctrs.mat'), 'ctrs')
                end
            end
            if pwr_mesh(j, i) < 0 && ~solve_reelin
                break; %if previous wind speed optimization <0 power break the wind speed loop
            end
        else
            disp('Hmm, something went wrong!');
            sol.info
            yalmiperror(sol.problem)
            if j == 1
                break;%stop if failed on first wind speed
            end
        end
    end
end
%% Save sweep results
results_name = strcat('Results/', results_run, '_', string(m_ac_vec(1)), 'to', string(m_ac_vec(end)), 'kg_', string(wind_spd_vec(1)), 'to', string(wind_spd_vec(end)), 'mps');
results.m_ac_mesh = m_ac_mesh;
results.wind_spd_mesh = wind_spd_mesh;
results.pwr_mesh = pwr_mesh;
if solve_reelin
    save(strcat(results_name, '_reelin.mat'), 'results')%save as reel-in solution
else
    save(strcat(results_name, '.mat'), 'results')%save as traction solution
end
