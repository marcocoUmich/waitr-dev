%% description
% This script iterates through a list of presaved random worlds and runs
% the ARMOUR planner on them. It then saves information on how well each the
% planner performed in each trial.
%
% Authors: Bohao Zhang (adapted from Patrick Holmes code)
% Created 25 November 2019
% Edited 16 January 2020
% Edited: 02 June 2022 to work with updated UARMTD code
% Edited: 25 September 2022 to work with updated UARMTD code for kinova
% Edited: 10 November 2022 clean up

initialize_script_path = matlab.desktop.editor.getActiveFilename;
cd(initialize_script_path(1:end-25));

close all; clear; clc; dbstop if error

delete(gcp('nocreate'))
% parpool('threads')

%% user parameters
goal_type = 'configuration'; % pick 'end_effector_location' or 'configuration' or 'fk_func'
goal_radius = pi/20;
dimension = 3 ;
verbosity = 10;

DURATION = 1.0;

u_s = 0.6; 
surf_rad =  0.058 / 2;

%%% for planner
traj_type = 'bernstein'; % pick 'orig' (ARMTD) or 'bernstein' (ARMOUR)
use_cuda_flag = true;
grasp_constraints_flag = true;
input_constraints_flag = true;

%%% for agent
agent_urdf = 'Kinova_Grasp_w_Tray.urdf';

add_uncertainty_to = 'none'; % choose 'all', 'link', or 'none'
links_with_uncertainty = {}; % if add_uncertainty_to = 'link', specify links here.
uncertain_mass_range = [0.97, 1.03];

agent_move_mode = 'integrator' ; % pick 'direct' or 'integrator'
use_CAD_flag = true; % plot robot with CAD or bounding boxes

%%% for LLC
use_robust_input = true;
use_true_params_for_robust = false;
if_use_mex_controller = true;
LLC_V_max = 2e-2;
alpha_constant = 10;
Kr = 4.0;

%%%File Saving
save_file_header = 'trial_' ;
file_location = '/media/marco/Backup disk/KRange_experiments' ;
if ~exist(file_location, 'dir')
    mkdir(file_location);
end

%%% for HLP
% default is a straight-line planner
if_use_RRT = false; % use an RRT HLP
if_use_graph_HLP = false; % use a graph HLP
HLP_grow_tree_mode = 'new' ; % pick 'new' or 'keep'
plot_waypoint_flag = true ;
plot_waypoint_arm_flag  = true ;
lookahead_distance = 0.1 ;

% plotting
plot_while_running = false ;

% simulation
max_sim_time = 172800 ; % 48 hours
max_sim_iter = 100 ;
stop_threshold = 3 ; % number of failed iterations before exiting

%%% for world
% start = [-1; -1; -1; -1; -1; -1; -1]; % start configuration
% goal = [1; 1; 1; 1; 1; 1; 1]; % goal configuration

% simple rotation
% goal = [0;-pi/2;0;0;0;0;0];
% start = [pi/4;-pi/2;0;0;0;0;0];

% start = [-pi/6;-pi/2;-pi/2;pi/2;0;pi/2;pi/2];
% goal = [pi/6;-pi/2;pi/2;pi/2;pi;-pi/2;pi/2];

% start = [3.9270, -1.0472, 0, -2.0944, 0, 1.5708, 0]';
% start = [3.9270, -1.0472, 0, -2.0944, 0, 1.5708, 0]';
% goal = [2.5,-0.5236,0,-2.0944,0,1.0472,0]';

% start = [0.4933;
%     0.9728;
%     0.6090;
%    -0.3981;
%     0.4258;
%    -1.5576;
%     0.7482];
% goal = [0.3808;
%     1.8013;
%     0.8670;
%    -0.7256;
%     0.7211;
%    -2.0458;
%     1.4523];


% swing
% start = [0;-pi/2;0;0;0;0;0];
% goal = [pi;-pi/2;pi;0;0;0;0];

% random that struggles to reach goal
% use to debug gradients as well
% start = [0.9534;-1.4310;0.1330;0.6418;-0.9534;-0.9534;0.0637];
% goal = [1.62310000000000;-1.59990000000000;-0.137000000000000;0.493080000000000;-3.26490000000000;-2.23000000000000;-0.246620000000000];

obstacles{1} = box_obstacle_zonotope('center', [10; 3; 3],...
                                     'side_lengths', [0.1; 0.1; 0.1]) ;
% obstacles{2} = box_obstacle_zonotope('center', [0.3; 0; 0.4],...
%                                      'side_lengths', [0.1; 0.8; 0.05]) ;

%% robot params:
robot = importrobot(agent_urdf);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
params = load_robot_params(robot, ...
                           'add_uncertainty_to', add_uncertainty_to, ...
                           'links_with_uncertainty', links_with_uncertainty,...
                           'uncertain_mass_range', uncertain_mass_range);
joint_speed_limits = [-1.3963, -1.3963, -1.3963, -1.3963, -1.2218, -1.2218, -1.2218;
                       1.3963,  1.3963,  1.3963,  1.3963,  1.2218,  1.2218,  1.2218]; % matlab doesn't import these from urdf so hard code into class
joint_input_limits = [-56.7, -56.7, -56.7, -56.7, -29.4, -29.4, -29.4;
                       56.7,  56.7,  56.7,  56.7,  29.4,  29.4,  29.4]; % matlab doesn't import these from urdf so hard code into class
transmision_inertia = [8.02999999999999936 11.99620246153036440 9.00254278617515169 11.58064393167063599 8.46650409179141228 8.85370693737424297 8.85873036646853151]; % matlab doesn't import these from urdf so hard code into class
M_min_eigenvalue = 8.0386472; % 8.29938; % ; matlab doesn't import these from urdf so hard code into class

figure(101)
show(robot)
%% automated from here
% run loop
if plot_while_running
    figure(1); clf; view(3); grid on;
end



%Setup params for looping different K ranges
k_range_mins = [pi/96; pi/96; pi/96; pi/96; pi/96; pi/96; pi/96];
k_range_maxes =  [pi/24; pi/24; pi/24; pi/24; pi/24; pi/24; pi/24];
k_range_iters =  200;

%Calculate individual k values. Keeps ratios the same for now
iter_arr = 1:1:k_range_iters;
iter_arr = iter_arr - 1;
k_diffs = k_range_maxes - k_range_mins;
k_range_table = (iter_arr.*k_diffs/(k_range_iters-1)) + k_range_mins;


tic
for idx = 1:1:k_range_iters
    clc; 
    k_range = k_range_table(:, idx);
    k_for_idx = k_range(1);
    
    W = kinova_grasp_world_static('create_random_obstacles_flag', false, 'goal_radius', goal_radius, 'dimension',dimension,'workspace_goal_check', 0, 'verbose',verbosity, 'start', start, 'goal', goal,  'goal_type', goal_type, 'grasp_constraint_flag', grasp_constraints_flag,'ik_start_goal_flag', true,'u_s', u_s, 'surf_rad', surf_rad, 'robot_params', params) ;
    % create arm agent
    A = uarmtd_agent(robot, params,...
                     'verbose', verbosity,...
                     'animation_set_axes_flag', 0,... 
                     'animation_set_view_flag', 0,...
                     'move_mode', agent_move_mode,...
                     'use_CAD_flag', use_CAD_flag,...
                     'joint_speed_limits', joint_speed_limits, ...
                     'joint_input_limits', joint_input_limits, ...
                     'add_measurement_noise_', false, ...
                     'measurement_noise_size_', 0,...
                     'M_min_eigenvalue', M_min_eigenvalue, ...
                     'transmision_inertia', transmision_inertia,...
                     't_total', DURATION);

    % LLC
    if use_robust_input
        A.LLC = uarmtd_robust_CBF_LLC('verbose', verbosity, ...
                                      'use_true_params_for_robust', use_true_params_for_robust, ...
                                      'V_max', LLC_V_max, ...
                                      'alpha_constant', alpha_constant, ...
                                      'Kr', Kr, ...
                                      'if_use_mex_controller', if_use_mex_controller);
    else
        A.LLC = uarmtd_nominal_passivity_LLC('verbose', verbosity);
    end

    A.LLC.setup(A);
    P = uarmtd_planner('verbose', verbosity, ...
                   'first_iter_pause_flag', false, ...
                   'use_q_plan_for_cost', true, ...
                   'input_constraints_flag', input_constraints_flag, ...
                   'grasp_constraints_flag', grasp_constraints_flag,...
                   'use_robust_input', use_robust_input, ...
                   'traj_type', traj_type, ...
                   'use_cuda', use_cuda_flag,...
                   'k_range', k_range, ...
                   'plot_HLP_flag', true, ...
                   'lookahead_distance', 0.4, ...
                   'u_s', u_s,...
                   'surf_rad', surf_rad,...
                   'DURATION', DURATION) ; % 't_move_temp', t_move't_plan', t_plan,...'t_stop', t_stop % _wrapper

    if if_use_RRT
        P.HLP = arm_end_effector_RRT_star_HLP('plot_waypoint_flag',plot_waypoint_flag,...
                                              'plot_waypoint_arm_flag',plot_waypoint_arm_flag,...
                                              'grow_tree_mode',HLP_grow_tree_mode,...
                                              'buffer',0.1) ;
    end
    
    % set up world using arm
    I = A.get_agent_info ;
    W.setup(I) ;
    W.bounds = [-1 1 -1 1 0 2];
    
    % place arm at starting configuration
    A.state(A.joint_state_indices) = W.start ;
    
    % create simulator
    S = simulator_armtd(A,W,P, ...
                    'verbose', verbosity, ...
                    'stop_threshold', stop_threshold, ...
                    'plot_while_running', plot_while_running,...
                    'allow_replan_errors',true,...
                    'max_sim_time',max_sim_time,...
                    'max_sim_iterations',max_sim_iter,...
                    'stop_sim_when_ultimate_bound_exceeded', false) ; 
    
    % %% plotting
    if plot_while_running
        figure(1) ; clf ; axis equal ; xlim([-1 1]); ylim([-1 1]); zlim([0 2]); grid on; hold on ;

        if dimension == 3
            view(3);
        end
        
        plot(A);
        plot(W);
    end
    
    % run simulation
    summary = S.run() ;
    
    %% save summary
    filename = append(file_location,'/',save_file_header,num2str(k_for_idx),'.mat');
    save(filename, 'summary', 'A', 'P', 'W', 'S')
end