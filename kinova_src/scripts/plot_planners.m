% Setup params for looping different K ranges
k_range_mins = [pi/96; pi/96; pi/96; pi/96; pi/96; pi/96; pi/96];
k_range_maxes = [pi/8; pi/8; pi/8; pi/8; pi/8; pi/8; pi/8];
k_range_iters = 50;

% Calculate individual k values. Keeps ratios the same for now
iter_arr = 1:1:k_range_iters;
iter_arr = iter_arr - 1;
k_diffs = k_range_maxes - k_range_mins;
k_range_table = (iter_arr.*k_diffs/(k_range_iters-1)) + k_range_mins;

file_location = '/media/marco/D/WAITR_experiments_redo2/KR';
save_file_header = 'KR_iteration_';

% Initialize arrays to store values
k_range_idx_values = zeros(k_range_iters, 1);
goal_check_values = zeros(k_range_iters, 1);
total_real_time_values = zeros(k_range_iters, 1);
max_planning_time_values = zeros(k_range_iters, 1);
num_brakes_values = zeros(k_range_iters, 1);
joint_speed_means = zeros(7, k_range_iters); 
tilt_values = zeros(k_range_iters, 1);
num_trajectories = zeros(k_range_iters, 1);

for idx = 1:k_range_iters
    k_range = k_range_table(:, idx);
    k_for_idx = k_range(1);
    filename = fullfile(file_location, strcat(save_file_header, num2str(idx), '.mat'));
    
    % Parsing file
    if isfile(filename)
        loaded_data = load(filename);
        k_range_idx_values(idx) = loaded_data.k_range(1);
        goal_check_values(idx) = loaded_data.summary.goal_check;
        total_real_time_values(idx) = loaded_data.summary.total_real_time;
        max_planning_time_values(idx) = max(loaded_data.summary.planning_time);
        num_brakes_values(idx) = sum(loaded_data.summary.stop_check);
        joint_speed_means(:, idx) = abs(mean(loaded_data.A.state(loaded_data.A.joint_speed_indices, :)')');
        [max_tilt_angle, max_tilt_angle_index] = max(abs(loaded_data.A.state(13,:)));
        if max_tilt_angle>0.1
            disp(idx)
            disp(loaded_data.k_for_idx)
        end
        tilt_values(idx) = rad2deg(max_tilt_angle);
%         traj_size = size(loaded_data.iterations);
%         num_trajectories(idx) = traj_size(2);
        
    else
        disp(['File not found: ', filename]);
    end
end

% Plotting
figure;

% Subplot 1: k_range_idx values with bar colors based on goal_check
subplot(4,1,1);
for idx = 1:k_range_iters
    if goal_check_values(idx) == 1
        bar(idx, k_range_idx_values(idx), 'g'); % Green for goal_check == 1
    else
        bar(idx, k_range_idx_values(idx), 'r'); % Red for goal_check == 0
    end
    hold on; 
end
xlabel('k range iteration');
ylabel('k range value');
title('k range idx Values with Goal Check');
hold off;

% Subplot 2: total_real_time and max_planning_time
subplot(4,1,2);
plot(1:k_range_iters, total_real_time_values, 'b-o');
xlabel('k range iteration');
ylabel('Time (s)');
legend('Total Real Time');
title('Total Time');


% Subplot 3: Mean joint speeds


subplot(4,1,3);
plot(joint_speed_means');
xlabel('k range iteration');
ylabel('Mean Joint Speed');
title('Mean Joint Speeds');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7');

% Subplot 2: total_real_time and max_planning_time
subplot(4,1,4);
plot(1:k_range_iters, max_planning_time_values, 'r-*');
xlabel('k range iteration');
ylabel('Time (s)');
legend('Max Planning Time');
title('Max Planning Time');
figure;


% Subplot 1: k_range_idx values with bar colors based on goal_check
subplot(3,1,1);
for idx = 1:k_range_iters
    if goal_check_values(idx) == 1
        bar(idx, k_range_idx_values(idx), 'g'); % Green for goal_check == 1
    else
        bar(idx, k_range_idx_values(idx), 'r'); % Red for goal_check == 0
    end
    hold on; 
end
xlabel('k range iteration');
ylabel('k range value');
title('k range idx Values with Goal Check');
hold off;

% Subplot 4: tilt_angle
subplot(3,1,2);
bar(tilt_values);
xlabel('k range iteration');
ylabel('Max tilt angle (degrees)');
title('Max tilt angle');
% 
% % Subplot 3: num_trajectories
% subplot(4,1,3);
% bar(num_trajectories);
% xlabel('k range iteration');
% ylabel('Num Trajectories');
% title('Number of trajectories');

% Subplot 4: num_brakes
subplot(3,1,3);
bar(num_brakes_values);
xlabel('k range iteration');
ylabel('Num Brakes');
title('Number of Braking Maneuvers');
