clc
clear
close all

yalmip('clear')
addpath('..\')

save_soln = 1;
use_guess = 1;
use_prev_solution_guess = 0;
results_run = 'res31';
save_path = strcat('../../aerogen_yalmip_results/Twin/', results_run, '/');
load_path = strcat('../../aerogen_yalmip_results/Twin/', 'res31', '/');
load_name = 'Solns/warmstart_200kg_12mps';
load_name = strcat(load_path, 'soln_', string(160),'kg_', string(12), 'mps');

% Define horizon
tf = 10;
gridSz = 60;
max_time = 400;
dt = tf/(gridSz-1);
timeVec = linspace(0, tf, gridSz);
ctr_obj_gain = 2;
p = getParams(); p(19) = tf; p(20) = dt;

%% Define variables/params
nx = 14; 
nu = 5;
pwr_figure = figure;
wind_spd_vec = 12:-2:4;
m_ac_vec = 140:-20:0;
soln_fail = 0;
pwr_mesh = zeros(length(wind_spd_vec), length(m_ac_vec));
[m_ac_mesh, wind_spd_mesh] = meshgrid(m_ac_vec, wind_spd_vec);
% wind_spd_vec = wind_spd_vec(1);%temp
for i = 1:length(m_ac_vec)
    if i > 1
        load_name = strcat(save_path, 'soln_', string(m_ac_vec(i-1)),'kg_', string(wind_spd_vec(1)), 'mps');
    end
    for j = 1:length(wind_spd_vec)
        if j > 1
            load_name = strcat(save_path, 'soln_', string(m_ac_vec(i)),'kg_', string(wind_spd_vec(j-1)), 'mps');
        end
        if use_prev_solution_guess
            load_name = strcat(load_path, 'soln_', string(m_ac_vec(i)),'kg_', string(wind_spd_vec(j)), 'mps');
        end
        yalmip('clear')
        x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:theta1, 4:theta2, 5:va1, 6:va2, 7:psi1, 8:psi2, 9:x1, 10:y1, 11:x2, 12:y2
        u = sdpvar(nu, gridSz);%1:m_ctr, 2:de1(theta1_dot), 3:de2(theta2_dot), 4:dr1(psi1_dot), 4:dr2(psi2_dot)

        p(10) = wind_spd_vec(j);
        p(3) = m_ac_vec(i);
        %% Contraints & Objective
        [Constraints,Objective] = getConstObj_twin(gridSz, dt, p, ctr_obj_gain, nx, nu, x, u);
        
        %% Set some options for YALMIP and solver
        if use_guess
            if isempty(dir(fullfile(strcat(load_name, '_states.mat'))))
                break;
            end
            load(strcat(load_name, '_states.mat'));
            load(strcat(load_name, '_ctrs.mat'));
            assign(x, states);
            assign(u, ctrs);
            options = sdpsettings('solver','ipopt', 'usex0', 1, 'ipopt.max_cpu_time', max_time, 'ipopt.tol', 1e-4, 'ipopt.dual_inf_tol', 1e-4, 'ipopt.constr_viol_tol', 1e-4);
        else
            options = sdpsettings('solver','ipopt', 'ipopt.max_cpu_time', max_time, 'ipopt.tol', 1e-4, 'ipopt.dual_inf_tol', 1e-4, 'ipopt.constr_viol_tol', 1e-4);
        end
        
        %% Solve the problem
        sol = optimize(Constraints,Objective,options);
        soln_fail = sol.problem;
        %% Analyze error flags
        if soln_fail == 0
            % Extract and display value
            states = value(x);
            ctrs = value(u);
%             plot_aerogen_twin(states,ctrs,p,timeVec);

            pwr_mesh(j, i) = mean(-u(1, :).*30000.*x(2, :)/1000);
            close(pwr_figure);
            pwr_figure = figure;
            surf(m_ac_mesh, wind_spd_mesh, pwr_mesh);
            title('Twin Power Curve')
            xlabel('Mass [kg]')
            ylabel('Wind Speed [mps]')
            zlabel('Average Power [kw]')
            view([45, 45]);
            drawnow;

            if save_soln
                save_name = strcat(save_path, 'soln_', string(m_ac_vec(i)),'kg_', string(wind_spd_vec(j)), 'mps');
                save(strcat(save_name, '_states.mat'), 'states')
                save(strcat(save_name, '_ctrs.mat'), 'ctrs')
            end
        else
            disp('Hmm, something went wrong!');
            sol.info
            yalmiperror(sol.problem)
            if j == 1
                break;
            end
        end
    end
end
results_name = strcat('Results/', results_run, '_', string(m_ac_vec(1)), 'to', string(m_ac_vec(end)), 'kg_', string(wind_spd_vec(1)), 'to', string(wind_spd_vec(end)), 'mps');
results.m_ac_mesh = m_ac_mesh;
results.wind_spd_mesh = wind_spd_mesh;
results.pwr_mesh = pwr_mesh;
save(strcat(results_name, '.mat'), 'results')
