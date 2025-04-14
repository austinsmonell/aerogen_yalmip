clc
clear
close all

yalmip('clear')
addpath('..\')

save_soln = 0;
use_guess = 1;
save_name = 'Solns/soln1';
load_name = 'Solns/soln1';
% Define horizon
tf = 10;
gridSz = 60;
dt = tf/(gridSz-1);
timeVec = linspace(0, tf, gridSz);
ctr_obj_gain = 10;
alpha_lim = 18*pi/180;
p = getParams(); p(31) = tf; p(32) = dt;

%% Define variables/params
nx = 7; 
nu = 3; 
x = sdpvar(nx,gridSz);%1:sigma 2:sigma_dot, 3:va, 4: theta, 5:psi, 6:x1, 7:x2
u = sdpvar(nu, gridSz);%1:m_ctr, 2:de1(theta_dot), 3:dr1(psi_Dot)

pwr_figure = figure;
wind_spd_vec = 12:-4:4;
m_ac_vec = 0:10:50;
pwr_arr = zeros(length(wind_spd_vec), length(m_ac_vec));
[x_plt, y_plt] = meshgrid(m_ac_vec, wind_spd_vec);
for i = 1:length(wind_spd_vec)
    for j = 1:length(m_ac_vec)
        p(13) = wind_spd_vec(i);
        p(6) = m_ac_vec(j);
        %% Contraints & Objective
        [Constraints,Objective] = getConstObj_single(gridSz, dt, p, alpha_lim, ctr_obj_gain, nx, nu, x, u);
        
        %% Set some options for YALMIP and solver
%         load_name = save_name;
%         save_name = strcat('soln_', wind_spd, 'mps_', m_ac, 'kg');
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
            plot_aerogen_single(states,ctrs,p,timeVec);

            pwr_arr(i, j) = mean(-u(1, :).*20000.*x(2, :)/1000);
            close(pwr_figure);
            pwr_figure = figure;
            surf(x_plt, y_plt, pwr_arr);

            if save_soln
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
