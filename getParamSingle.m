function p = getParamSingle()
    %% System Parameters
    r_gen = 0.5;
    moi_g = 1.0;
    c = 40000.0;%unused
    k_line = 10.0;%unused
    d = 300.0;%unused
    m_ac = 100;%6000
    rho = 1.23;
    S = 30.0;%500
    CL0 = 0.3;
    CLa = 5.0;
    CD0 = 0.01;
    e = 0.02;
    vw = 10.0;
    moi_ac_m = 100;%1000000
    d_c = 2.0;%3
    CMa = 0.0;%-1.63;
    CMq = 0.0;%-0.05;
    CMde = 1.6;
    CNb = 0.08;
    CNr = -0.05;
    CNdr = 0.0516;
    CYb = -0.36;
    moi_ac_n = 150;%5000000
    CDb = 0.0;
    g = 9.81;
    m_teth = 120;%6500
    CD_eff_teth = 0.042;
    alpha_min = 0.0;
    CDds = 1.0;
    omega = 0.0;%0

    p = [r_gen; moi_g; c; k_line; d; m_ac; rho; S; CL0; CLa; CD0; e; vw; moi_ac_m; d_c; CMa; CMq; CMde; CNb; CNr; CNdr; CYb; moi_ac_n; CDb; g; m_teth; CD_eff_teth; alpha_min; CDds; omega];
end