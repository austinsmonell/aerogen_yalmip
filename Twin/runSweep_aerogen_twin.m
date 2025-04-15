clc
clear
close all

yalmip('clear')
addpath('..\')

save_soln = 0;
use_guess = 0;
save_name = 'Solns/soln1';
load_name = 'Solns/soln1';
results_name = 'Results/res2';
% Define horizon
tf = 10;
gridSz = 60;
dt = tf/(gridSz-1);
timeVec = linspace(0, tf, gridSz);
ctr_obj_gain = 10;
alpha_up = 18*pi/180;
alpha_low = -2*pi/180;
p = getParams(); p(31) = tf; p(32) = dt;

%% Define variables/params
nx = 12; 
nu = 5;
x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:theta1, 4:theta2, 5:va1, 6:va2, 7:psi1, 8:psi2, 9:x1, 10:y1, 11:x2, 12:y2
u = sdpvar(nu, gridSz);%1:m_ctr, 2:de1(theta1_dot), 3:de2(theta2_dot), 4:dr1(psi1_dot), 4:dr2(psi2_dot)

pwr_figure = figure;
wind_spd_vec = 12:-4:8;
m_ac_vec = 0:50:500;
pwr_mesh = zeros(length(wind_spd_vec), length(m_ac_vec));
[m_ac_mesh, wind_spd_mesh] = meshgrid(m_ac_vec, wind_spd_vec);
for i = 1:length(m_ac_vec)
    use_guess = 1;
    save_soln = 1;
    for j = 1:length(wind_spd_vec)
        p(13) = wind_spd_vec(j);
        p(6) = m_ac_vec(i);
        %% Contraints & Objective
        [Constraints,Objective] = getConstObj_twin(gridSz, dt, p, alpha_low, alpha_up, ctr_obj_gain, nx, nu, x, u);
        
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
            plot_aerogen_twin(states,ctrs,p,timeVec);

            pwr_mesh(j, i) = mean(-u(1, :).*22000.*x(2, :)/1000);
            close(pwr_figure);
            pwr_figure = figure;
            surf(m_ac_mesh, wind_spd_mesh, pwr_mesh);
            title('Twin Power Curve')
            xlabel('Mass [kg]')
            ylabel('Wind Speed [mps]')
            zlabel('Average Power [kw]')
            drawnow;

            if save_soln
                save(strcat(save_name, '_states.mat'), 'states')
                save(strcat(save_name, '_ctrs.mat'), 'ctrs')
            end
        else
            disp('Hmm, something went wrong!');
            sol.info
            yalmiperror(sol.problem)
        end
        use_guess = 1;
        save_soln = 0;
    end
end
results.m_ac_mesh = m_ac_mesh;
results.wind_spd_mesh = wind_spd_mesh;
results.pwr_mesh = pwr_mesh;
save(strcat(results_name, '.mat'), 'results')
