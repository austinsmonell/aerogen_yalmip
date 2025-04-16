clc
clear
close all

yalmip('clear')
addpath('..\')

save_soln = 1;
use_guess = 1;
results_run = 'res2';
save_path = strcat('../../aerogen_yalmip_results/Single/', results_run, '/');
load_name = 'Solns/soln_0kg_12mps';
% Define horizon
tf = 10;
gridSz = 60;
dt = tf/(gridSz-1);
timeVec = linspace(0, tf, gridSz);
ctr_obj_gain = 1;
alpha_lim = 18*pi/180;
p = getParams(); p(31) = tf; p(32) = dt;

%% Define variables/params
nx = 7; 
nu = 3; 

pwr_figure = figure;
wind_spd_vec = 12:-2:2;
m_ac_vec = 0:200:1000;
pwr_mesh = zeros(length(wind_spd_vec), length(m_ac_vec));
[m_ac_mesh, wind_spd_mesh] = meshgrid(m_ac_vec, wind_spd_vec);
for i = 1:length(m_ac_vec)
    if i > 1
        load_name = strcat(save_path, 'soln_', string(m_ac_vec(i-1)),'kg_', string(wind_spd_vec(1)), 'mps');
    end
    for j = 1:length(wind_spd_vec)
        yalmip('clear')
        x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:va, 4: theta, 5:psi, 6:x1, 7:x2
        u = sdpvar(nu, gridSz);%1:m_ctr, 2:de1(theta_dot), 3:dr1(psi_Dot)
        p(13) = wind_spd_vec(j);
        p(6) = m_ac_vec(i);
        %% Contraints & Objective
        [Constraints,Objective] = getConstObj_single(gridSz, dt, p, alpha_lim, ctr_obj_gain, nx, nu, x, u);
        
        %% Set some options for YALMIP and solver
        if use_guess
            load(strcat(load_name, '_states.mat'));
            load(strcat(load_name, '_ctrs.mat'));
            assign(x, states);
            assign(u, ctrs);
            options = sdpsettings('solver','ipopt', 'usex0', 1);
        else
            options = sdpsettings('solver','ipopt');
        end
        
        %% Solve the problem
        sol = optimize(Constraints,Objective,options);
        
        %% Analyze error flags
        if sol.problem == 0
            % Extract and display value
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
            zlabel('Average Power [kw]')
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
        end
    end
end
results_name = strcat('Results/', results_run, '_', string(m_ac_vec(1)), 'to', string(m_ac_vec(end)), 'kg_', string(wind_spd_vec(1)), 'to', string(wind_spd_vec(end)), 'mps');
results.m_ac_mesh = m_ac_mesh;
results.wind_spd_mesh = wind_spd_mesh;
results.pwr_mesh = pwr_mesh;
save(strcat(results_name, '.mat'), 'results')
