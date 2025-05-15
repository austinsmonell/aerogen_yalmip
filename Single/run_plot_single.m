clc
clear
% close all

%%
plot_states = 0;
plot_set = 1;

wind_spd = 12;
m_ac = 0;
result_set = 'res10';
if plot_states
    solnPath = strcat('../../aerogen_yalmip_results/Single/', result_set, '/soln_', string(m_ac), 'kg_', string(wind_spd), 'mps');
    load(strcat(solnPath, '_states.mat'))
    load(strcat(solnPath, '_ctrs.mat'))
    
    tf = 10;
    gridSz = 60;
    dt = tf/(gridSz-1);
    timeVec = linspace(0, tf, gridSz);
    p = getParams(); p(31) = tf; p(32) = dt;
    p(13) = wind_spd; p(6) = m_ac;
    plot_aerogen_single(states,ctrs,p,timeVec);
end
%% Plot Power Curve
if plot_set
    resPath = strcat('Results/', dir(fullfile('Results/',strcat(result_set, '*'))).name);
    load(resPath);
    
    figure
    surf(results.m_ac_mesh, results.wind_spd_mesh, results.pwr_mesh);
    title('Single Power Curve')
    xlabel('Mass [kg]')
    ylabel('Wind Speed [mps]')
    zlabel('Average Power [kw]')
end