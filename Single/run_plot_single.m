clc
clear
close all

%%
wind_spd = 6;
m_ac = 800;
result_set = 'res2';

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