================================================================================
                         AEROGEN YALMIP - README
================================================================================

Airborne Wind Energy (AWE) Trajectory Optimization Framework

This MATLAB project optimizes kite trajectories for power generation using
YALMIP with the IPOPT nonlinear solver. It supports two configurations:
  - Single: One kite performing crosswind maneuvers
  - Twin:   Two coordinated kites on a shared generator spool

================================================================================
                              REQUIREMENTS
================================================================================

1. MATLAB (R2019b or later recommended)
2. YALMIP - Optimization modeling framework
   https://yalmip.github.io
3. IPOPT - Nonlinear programming solver
   Must be configured to work with YALMIP

Add to your MATLAB path:
   addpath(genpath('path/to/YALMIP'))

================================================================================
                           DIRECTORY STRUCTURE
================================================================================

aerogen_yalmip/
  getParams.m              - Global parameter definitions
  run_plot_compare.m       - Compare Single vs Twin power output

  Single/
    run_aerogen_single.m       - Main solver (single solution)
    runSweep_aerogen_single.m  - Parameter sweep (mass x wind speed)
    run_plot_single.m          - Plot results and power surfaces
    combine_reelin.m           - Combine traction + reelin phases
    dynamic_single.m           - Dynamics equations
    getConstObj_single.m       - Constraints & objective function
    plot_aerogen_single.m      - State/trajectory plotting
    Solns/                     - Warm-start solution files
    Results/                   - Output result matrices

  Twin/
    run_aerogen_twin.m         - Main solver (single solution)
    runSweep_aerogen_twin.m    - Parameter sweep
    run_plot_twin.m            - Plot results and power surfaces
    dynamic_twin.m             - Dynamics equations
    getConstObj_twin.m         - Constraints & objective function
    plot_aerogen_twin.m        - State/trajectory plotting
    plot_all_traj.m            - Multi-trajectory plotting
    Solns/                     - Warm-start solution files
    Results/                   - Output result matrices

================================================================================
                      SYSTEM PARAMETERS (getParams.m)
================================================================================

Current parameter values:

  GENERATOR:
    r_gen = 0.05           % Drum radius (m)
    moi_g = 0.01           % Moment of inertia (kg*m^2)

  AIRCRAFT:
    m_ac = 1               % Default kite mass (kg) - overridden by scripts
    S = 5.0                % Wing reference area (m^2)

  AERODYNAMICS:
    rho = 1.23             % Air density (kg/m^3)
    CL0 = 0.3              % Lift coefficient at zero alpha
    CLa = 5.0              % Lift curve slope (1/rad)
    CD0 = 0.02             % Drag coefficient baseline
    CYb = -6.3             % Side force coefficient
    e = 0.02               % Oswald efficiency factor
    CDds = 1.0             % Drag coefficient multiplier

  ENVIRONMENT:
    vw = 12.0              % Default wind speed (m/s) - overridden by scripts
    g = 9.81               % Gravity (m/s^2)
    omega = 0.0            % Rotation rate

  TETHER:
    m_teth = 20            % Tether mass (kg)
    CD_eff_teth = 0.12     % Effective tether drag coefficient

  ANGLE LIMITS:
    alpha_min = -10        % Min angle of attack (deg)
    alpha_max = 18         % Max angle of attack (deg)
    beta_lim = 15          % Sideslip angle limit (deg)

  TIMING (set by scripts):
    tf = 0                 % Time horizon (s) - set by run scripts
    dt = 0                 % Time step (s) - set by run scripts

================================================================================
                    RUNNING run_aerogen_single.m
================================================================================

This script solves a single optimization for one kite at specified conditions.

PARAMETERS TO CONFIGURE (lines 8-21):

  save_soln = 1;           % 1 = save solution, 0 = don't save
  use_guess = 0;           % 1 = load warm-start, 0 = cold start
  solve_reelin = 1;        % 1 = solve reelin phase, 0 = solve traction
  num_loops = 6;           % Number of traction loops before reelin

  save_path = 'Solns/';    % Where to save new solutions
  save_name = 'Solns/warmstart_0kg_12mps';  % Warm-start file to load

  tf = 10;                 % Time horizon (seconds)
  gridSz = 60;             % Number of discretization points
  ctr_obj_gain = 10;       % Control smoothness weight
  wind_spd = 12;           % Wind speed (m/s)
  m_ac = 0;                % Kite mass (kg)

IMPORTANT - ALIGNING PATHS:
  - save_name must point to an existing solution when use_guess = 1
  - For reelin phase (solve_reelin = 1), the script automatically loads
    the traction solution from save_path matching m_ac and wind_spd
  - Ensure the traction solution exists before running reelin

WORKFLOW:
  1. First run with solve_reelin = 0 to get traction solution
  2. Then run with solve_reelin = 1 to get reelin solution
  3. Use combine_reelin.m to merge into full cycle

================================================================================
                      RUNNING run_aerogen_twin.m
================================================================================

This script solves a single optimization for the twin-kite configuration.

PARAMETERS TO CONFIGURE (lines 8-23):

  save_soln = 1;           % 1 = save solution, 0 = don't save
  use_guess = 1;           % 1 = load warm-start, 0 = cold start

  save_name = 'Solns/warmstart_200kg_12mps';   % Where to save solution
  load_name = 'Solns/warmstart_200kg_12mps';   % Warm-start file to load

  tf = 10;                 % Time horizon (seconds)
  gridSz = 60;             % Number of discretization points
  max_time = 500;          % Max solver time (seconds)
  ctr_obj_gain = 2;        % Control smoothness weight
  wind_spd = 12;           % Wind speed (m/s)
  m_ac = 200;              % Kite mass (kg)

IMPORTANT - ALIGNING PATHS:
  - save_name and load_name must match your intended mass/wind values
  - When use_guess = 1, load_name must point to existing solution
  - Update both save_name and load_name when changing m_ac or wind_spd

================================================================================
                   RUNNING runSweep_aerogen_single.m
================================================================================

This script runs a parameter sweep over mass and wind speed ranges.

PARAMETERS TO CONFIGURE (lines 8-16, 32-33):

  save_soln = 1;           % 1 = save each solution
  use_guess = 1;           % 1 = use warm-starting between solves
  solve_reelin = 1;        % 1 = sweep reelin, 0 = sweep traction
  num_loops = 6;           % Traction loops (for reelin phase)

  results_run = 'res27';   % Result set identifier (e.g., 'res27')
  save_path = strcat('../../aerogen_yalmip_results/Single/', results_run, '/');
  load_path = strcat('../../aerogen_yalmip_results/Single/', 'res27', '/');
  load_name = strcat(load_path, 'soln_', string(80),'kg_', string(12), 'mps');

  max_time = 200;          % Max solver time per solution
  ctr_obj_gain = 10;       % Control smoothness weight

  wind_spd_vec = 12:-2:4;  % Wind speeds to sweep (high to low)
  m_ac_vec = 80:20:400;    % Masses to sweep (low to high)

IMPORTANT - ALIGNING PATHS:
  - results_run: Use a unique identifier for each sweep run
  - save_path: Where new solutions are saved (create folder first!)
  - load_path: Where to find warm-start solutions (can be same as save_path
    or a previous result set)
  - load_name: Initial warm-start solution (first solve in sweep)
  - For reelin sweep, traction solutions must exist at save_path

WORKFLOW:
  1. Create output folder: aerogen_yalmip_results/Single/res##/
  2. Run traction sweep first (solve_reelin = 0)
  3. Run reelin sweep second (solve_reelin = 1)
  4. Results saved to Results/res##_...mat

================================================================================
                    RUNNING runSweep_aerogen_twin.m
================================================================================

This script runs a parameter sweep for the twin-kite configuration.

PARAMETERS TO CONFIGURE (lines 8-15, 30-31):

  save_soln = 1;           % 1 = save each solution
  use_guess = 1;           % 1 = use warm-starting
  use_prev_solution_guess = 0;  % 1 = use exact same params from load_path

  results_run = 'res32';   % Result set identifier
  save_path = strcat('../../aerogen_yalmip_results/Twin/', results_run, '/');
  load_path = strcat('../../aerogen_yalmip_results/Twin/', 'res32', '/');
  load_name = strcat(load_path, 'soln_', string(240),'kg_', string(12), 'mps');

  max_time = 400;          % Max solver time per solution
  ctr_obj_gain = 2;        % Control smoothness weight

  wind_spd_vec = 12:-2:4;  % Wind speeds to sweep
  m_ac_vec = 220:-20:0;    % Masses to sweep (high to low for twin)

IMPORTANT - ALIGNING PATHS:
  - results_run: Unique identifier for this sweep
  - save_path: Create this folder before running!
  - load_path: Can reference a previous result set for warm-starting
  - load_name: Must point to valid solution for first iteration

================================================================================
                      RUNNING run_plot_single.m
================================================================================

This script visualizes solutions and creates power surface plots.

PARAMETERS TO CONFIGURE (lines 6-17):

  plot_states = 0;         % 1 = plot individual trajectory
  plot_set = 1;            % 1 = plot 3D power surface
  create_set = 1;          % 1 = create result matrix from solution files
  plot_curve = 0;          % 1 = plot power curve at fixed mass
  use_full_cyl = 1;        % 1 = use full cycle data, 0 = traction only
  plot_mode = 0;           % 0 = traction, 1 = reelin, 2 = full cycle

  wind_spd = 12;           % Wind speed for individual plot
  m_ac = 340;              % Mass for individual plot / power curve
  result_set = 'res27';    % Result set to load/plot

  tf = 10;                 % Time horizon
  gridSz = 60;             % Grid size

IMPORTANT - ALIGNING PATHS:
  - result_set: Must match the results_run used in runSweep script
  - Solution files are loaded from:
    ../../aerogen_yalmip_results/Single/{result_set}/
  - Result matrices are saved to: Results/

  When plot_states = 1:
    - Set wind_spd and m_ac to match an existing solution
    - Set plot_mode based on available data (0/1/2)

  When create_set = 1:
    - Ensure wind_spd_vec and m_ac_vec (lines 43-44) match your sweep range
    - Set use_full_cyl = 1 if you have full cycle data

WORKFLOW:
  1. Set result_set to match your sweep
  2. Set create_set = 1, plot_set = 1 to generate and view power surface
  3. Set plot_states = 1 with specific m_ac/wind_spd to view trajectory

================================================================================
                       RUNNING run_plot_twin.m
================================================================================

This script visualizes twin-kite solutions and power surfaces.

PARAMETERS TO CONFIGURE (lines 7-14):

  plot_states = 1;         % 1 = plot individual trajectory
  plot_set = 0;            % 1 = plot 3D power surface
  create_set = 0;          % 1 = create result matrix from solution files
  plot_curve = 0;          % 1 = plot power curve at fixed mass

  wind_spd = 12;           % Wind speed for individual plot
  m_ac = 200;              % Mass for individual plot / power curve
  result_set = 'res31';    % Result set to load/plot

IMPORTANT - ALIGNING PATHS:
  - result_set: Must match the results_run used in runSweep_aerogen_twin
  - Solution files are loaded from:
    ../../aerogen_yalmip_results/Twin/{result_set}/
  - When create_set = 1, update wind_spd_vec and m_ac_vec (lines 31-32)
    to match your sweep range

WORKFLOW:
  1. Set result_set to match your sweep
  2. Adjust wind_spd_vec/m_ac_vec if your sweep used different ranges
  3. Use create_set = 1 first to build the result matrix
  4. Use plot_set = 1 to visualize the power surface

================================================================================
                         STATE AND CONTROL VARIABLES
================================================================================

SINGLE KITE (8 states, 3 controls):

  States:
    1. sigma      - Generator spool position (rad)
    2. sigma_dot  - Generator angular velocity (rad/s)
    3. va_u       - Kite airspeed along tether (m/s)
    4. theta      - Pitch angle (rad)
    5. psi        - Heading angle (rad)
    6. x1         - Horizontal position (m)
    7. y1         - Vertical position (m)
    8. va_v       - Lateral velocity component (m/s)

  Controls:
    1. m_ctr      - Motor torque (scaled -1 to +1 -> +/-20,000 Nm)
    2. de         - Pitch rate command (rad/s)
    3. dr         - Yaw rate command (rad/s)

TWIN KITE (14 states, 5 controls):

  States:
    1. sigma      - Shared generator position (rad)
    2. sigma_dot  - Shared generator velocity (rad/s)
    3. theta1     - Kite 1 pitch angle (rad)
    4. theta2     - Kite 2 pitch angle (rad)
    5. va1_u      - Kite 1 airspeed along tether (m/s)
    6. va2_u      - Kite 2 airspeed along tether (m/s)
    7. psi1       - Kite 1 heading angle (rad)
    8. psi2       - Kite 2 heading angle (rad)
    9. x1         - Kite 1 horizontal position (m)
    10. y1        - Kite 1 vertical position (m)
    11. x2        - Kite 2 horizontal position (m)
    12. y2        - Kite 2 vertical position (m)
    13. va1_v     - Kite 1 lateral velocity (m/s)
    14. va2_v     - Kite 2 lateral velocity (m/s)

  Controls:
    1. m_ctr      - Shared motor torque (scaled -1 to +1 -> +/-30,000 Nm)
    2. de1        - Kite 1 pitch rate (rad/s)
    3. de2        - Kite 2 pitch rate (rad/s)
    4. dr1        - Kite 1 yaw rate (rad/s)
    5. dr2        - Kite 2 yaw rate (rad/s)

================================================================================
                              OUTPUT FILES
================================================================================

SOLUTION FILES:
  Single/Solns/ or aerogen_yalmip_results/Single/{result_set}/
    soln_###kg_##mps_states.mat      - Traction state trajectory
    soln_###kg_##mps_ctrs.mat        - Traction control trajectory
    soln_###kg_##mps_reelin_states.mat - Reelin state trajectory
    soln_###kg_##mps_reelin_ctrs.mat   - Reelin control trajectory
    soln_###kg_##mps_full_states.mat   - Combined full cycle states
    soln_###kg_##mps_full_ctrs.mat     - Combined full cycle controls
    soln_###kg_##mps_full_time.mat     - Combined time vector

  Twin/Solns/ or aerogen_yalmip_results/Twin/{result_set}/
    soln_###kg_##mps_states.mat      - State trajectory
    soln_###kg_##mps_ctrs.mat        - Control trajectory

RESULT MATRICES:
  Single/Results/ or Twin/Results/
    res##_###to###kg_##to##mps.mat
    res##full_###to###kg_##to##mps.mat  (Single full cycle)

  Contains:
    results.m_ac_mesh      - Mass grid
    results.wind_spd_mesh  - Wind speed grid
    results.pwr_mesh       - Average power (kW)

================================================================================
                            TROUBLESHOOTING
================================================================================

"File not found" errors:
  - Check that save_path/load_path folders exist
  - Verify result_set matches your sweep identifier
  - Ensure mass and wind speed values match existing solutions

Solver fails to converge:
  - Increase max_time
  - Use warm-start from nearby solution (use_guess = 1)
  - Try reducing gridSz for faster (less accurate) solve
  - Check wind speed is in feasible range (4-12 m/s typical)

Empty power surface regions:
  - Gray areas indicate infeasible parameter combinations
  - Usually at very low wind speeds or extreme masses

IPOPT not found:
  - Verify IPOPT installation and MATLAB path
  - Test with: yalmiptest

================================================================================
                               QUICK START
================================================================================

1. SINGLE KITE - Solve one case:
   >> cd Single
   >> % Edit run_aerogen_single.m: set wind_spd, m_ac, use_guess=0
   >> run_aerogen_single

2. TWIN KITE - Solve one case:
   >> cd Twin
   >> % Edit run_aerogen_twin.m: set wind_spd, m_ac, load_name
   >> run_aerogen_twin

3. RUN A SWEEP:
   >> cd Single  % or Twin
   >> % Create output folder first
   >> % Edit runSweep script: set paths, mass/wind ranges
   >> runSweep_aerogen_single  % or runSweep_aerogen_twin

4. PLOT RESULTS:
   >> cd Single  % or Twin
   >> % Edit run_plot script: set result_set, create_set=1, plot_set=1
   >> run_plot_single  % or run_plot_twin

================================================================================
