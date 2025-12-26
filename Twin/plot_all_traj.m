clc
clear
close all

%%
addpath('../')
plot_states = 1;
plot_set = 1;
create_set = 1;
plot_curve = 0;

wind_spd = 12;
m_ac = 120;
result_set = 'res31';

if plot_states
    for m_ac = 180:20:380
        for wind_spd = 12:2:12
            try
                solnPath = strcat('../../aerogen_yalmip_results/Twin/', result_set, '/soln_', string(m_ac), 'kg_', string(wind_spd), 'mps');
                load(strcat(solnPath, '_states.mat'))
                load(strcat(solnPath, '_ctrs.mat'))
                
                tf = 10;
                gridSz = 60;
                dt = tf/(gridSz-1);
                timeVec = linspace(0, tf, gridSz);
                p = getParams(); p(19) = tf; p(20) = dt;
                p(10) = wind_spd; p(3) = m_ac;
                plot_aerogen_twin(states,ctrs,p,timeVec);
            end
        end
    end
    figure(2)
    xlim([-150 250])
    ylim([-70 70])
    zlim([-200 250])
end