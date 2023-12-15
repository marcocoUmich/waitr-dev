%%Initialize vars and call the app

gen_FileList = true; % Set this to true or false as needed

% Define the directory path
directory = '/media/marco/D/WAITR_experiments_redo2/TS'; % Replace with the actual directory path

% Call the function if gen_FileList is true
if gen_FileList
    [FileList, type] = generateFileList(directory);
    
end


args = {};
if exist('A', 'var')
    args{end+1} = 'A';
    args{end+1} = A;
end
if exist('P', 'var')
    args{end+1} = 'P';
    args{end+1} = P;
end
if exist('S', 'var')
    args{end+1} = 'S';
    args{end+1} = S;
end
if exist('W', 'var')
    args{end+1} = 'W';
    args{end+1} = W;
end
if exist('FileList', 'var')  
    args{end+1} = 'Filelist';  
    args{end+1} = FileList;
    args{end+1} = 'Type';
    args{end+1} = type;
end


plotting_app(args{:});





function plotting_app(varargin)

    traj_len = 100;
    braking_frac = 0.5;


    p = inputParser;

    % Add parameters with default values as empty
    addParameter(p, 'A', [], @(x) isempty(x) || isa(x, 'uarmtd_agent'));
    addParameter(p, 'P', [], @(x) isempty(x) || isa(x, 'uarmtd_planner'));
    addParameter(p, 'S', [], @(x) isempty(x) || isa(x, 'simulator_armtd'));
    addParameter(p, 'W', [], @(x) isempty(x) || isa(x, 'kinova_grasp_world_static') || isa(x, 'kinova_world_static'));



    addParameter(p, 'Filelist', [], @(x) isempty(x) || (iscell(x) && all(cellfun(@ischar, x))));
    addParameter(p, 'Type', [], @(x) ismember(x, {'KR', 'ST', 'TS'}));


    % Parse the inputs
    parse(p, varargin{:});

    % Extract the results
    A = p.Results.A;
    P = p.Results.P;
    S = p.Results.S;
    W = p.Results.W;
    Filelist = p.Results.Filelist;
    Type = p.Results.Type;

    % Initialize a global variable to hold the save path
    save_path = pwd; 
    iterations = 10;
    single_iteration_types = {'position', 'velocity', 'acceleration', 'torques', 'force_cone'};
    comparison_types_x = {'k_range', 'n_time_steps', 'simplify_threshold'};
    comparison_types_y = {'mean_v', 'max_plan_t', 'mean_plan_t', 'n_brakes'};

    [single_iteration_types, single_iteration_selectable] = determineSingleIterationTypes();
    [comparison_types_x, comparison_types_x_selectable] = determineComparisonTypesX();
    [comparison_types_y, comparison_types_y_selectable] = determineComparisonTypesY();

    P1comparison_type_y_idx = 1;
    P1comparison_type_x_idx = 1;
    P1single_iteration_types_idx= 1;

    P2comparison_type_y_idx = 1;
    P2comparison_type_x_idx = 1;
    P2single_iteration_types_idx= 1;
    
    P3comparison_type_y_idx = 1;
    P3comparison_type_x_idx = 1;
    P3single_iteration_types_idx = 1;




    activePlot = 1;


    % Create the main figure

    main_fig = figure('Name', 'armour-plotting', 'NumberTitle', 'off', 'MenuBar', 'none');
    set(main_fig, 'Position', [100, 100, 800, 600]); % Set the position and size of the figure

    % Add menus' 
    menu_save_path = uimenu(main_fig, 'Text', 'Set Save Path', 'Callback', @set_save_path);
    menu1 = uimenu(main_fig, 'Text', 'Save Plot 1', 'Callback', @save_plot1);
    menu2 = uimenu(main_fig, 'Text', 'Save Plot 2', 'Callback', @save_plot2);
    menu_large_plot = uimenu(main_fig, 'Text', 'Save Large Plot', 'Callback', @save_large_plot);
    menu_help = uimenu(main_fig, 'Text', 'Help', 'Callback', @display_help);

    % Add binary selector (dropdown)
    lbl_switch = uicontrol('Style', 'text', 'Position', [10 550 100 20], 'String', 'Plot Selector');
    binary_selector = uicontrol('Style', 'popupmenu', 'Position', [20 530 100 20], 'String', {'Plot 1', 'Plot 2', 'Large Plot'}, 'Callback', @binary_selector_callback);

    % Add spinner
    lbl_spinner = uicontrol('Style', 'text', 'Position', [10 450 150 20], 'String', 'Joint/Iter Selection');
    spinner = uicontrol('Style', 'edit', 'Position', [20 430 100 20], 'String', '1', 'Callback', @spinner_callback);

%     lbl_spinner2 = uicontrol('Style', 'text', 'Position', [10 450 150 20], 'String', 'Iteration Selection');
%     spinner = uicontrol('Style', 'edit', 'Position', [20 430 100 20], 'String', '1', 'Callback', @spinner_callback);

    spinner_value = str2double(get(spinner, 'String'));
    

    % Add drop down 
    lbl_dropdown1 = uicontrol('Style', 'text', 'Position', [10 470 120 20], 'String', 'Single Iteration:', 'FontWeight', 'bold');
    dropdown1 = uicontrol('Style', 'popupmenu', 'Position', [20 410 100 20], 'String', single_iteration_types, 'Callback', @dropdown1_callback);

    % Add drop down 2
    lbl_dropdown2 = uicontrol('Style', 'text', 'Position', [10 250 150 20], 'String', 'Comparison Plots:', 'FontWeight', 'bold');
    lbl_dropdown2x = uicontrol('Style', 'text', 'Position', [10 230 20 20], 'String', 'x:', 'FontWeight', 'bold');
    lbl_dropdown2y = uicontrol('Style', 'text', 'Position', [10 210 20 20], 'String', 'y:', 'FontWeight', 'bold');
    dropdown2x = uicontrol('Style', 'popupmenu', 'Position', [30 230 100 20], 'String', comparison_types_x, 'Callback', @dropdown2x_callback);
    dropdown2y = uicontrol('Style', 'popupmenu', 'Position', [30 210 100 20], 'String', comparison_types_y, 'Callback', @dropdown2y_callback);

    % Add plots
    ax1 = axes('Position', [.3 .55 .65 .35]); % Adjusted position
    ax2 = axes('Position', [.3 .1 .65 .35]); % Adjusted position
    ax3 = axes('Position', [.25 .1 .7 .8], 'Visible', 'off');
    % Add a button to toggle zoom
    toggleZoomBtn = uicontrol('Style', 'pushbutton', 'Position', [20 360 100 30], 'String', 'Enable Zoom', 'Callback', @toggleZoom);
    
    % Add a button to toggle pan
    togglePanBtn = uicontrol('Style', 'pushbutton', 'Position', [20 320 100 30], 'String', 'Enable Pan', 'Callback', @togglePan);
    % Add a button to reset zoom and pan
    resetViewBtn = uicontrol('Style', 'pushbutton', 'Position', [20 280 100 30], 'String', 'Reset View', 'Callback', @resetView);

    % Set titles for the plots
    title(ax1, 'Plot 1');
    title(ax2, 'Plot 2');

    % Initialize plots visibility
    set(ax1, 'Visible', 'on');
    set(ax2, 'Visible', 'on');

    lbl_save_path = uicontrol('Style', 'text', 'Position', [10 570 80 20], 'String', 'Save Path:');
    save_path_display = uicontrol('Style', 'text', 'Position', [90 570 500 20], 'String', save_path);


    % Callback function to reset zoom and pan
    function resetView(~, ~)
        % Reset the axes to their default limits
        axis(ax1, 'auto');
        axis(ax2, 'auto');
        axis(ax3, 'auto');
    
        % Turn off zoom and pan
        zoom(main_fig, 'off');
        pan(main_fig, 'off');
    
        % Reset the labels of the zoom and pan buttons
        set(toggleZoomBtn, 'String', 'Enable Zoom');
        set(togglePanBtn, 'String', 'Enable Pan');
    end

    % Callback function to toggle zoom
    function toggleZoom(src, ~)
        if strcmp(get(zoom(main_fig), 'Enable'), 'off')
            zoom(main_fig, 'on');
            pan(main_fig, 'off'); % Disable pan when enabling zoom
            set(src, 'String', 'Disable Zoom');
            set(togglePanBtn, 'String', 'Enable Pan');
        else
            zoom(main_fig, 'off');
            set(src, 'String', 'Enable Zoom');
        end
    end
    
    % Callback function to toggle pan
    function togglePan(src, ~)
        if strcmp(get(pan(main_fig), 'Enable'), 'off')
            pan(main_fig, 'on');
            zoom(main_fig, 'off'); % Disable zoom when enabling pan
            set(src, 'String', 'Disable Pan');
            set(toggleZoomBtn, 'String', 'Enable Zoom');
        else
            pan(main_fig, 'off');
            set(src, 'String', 'Enable Pan');
        end
    end

    % Define the binary selector callback function
    function binary_selector_callback(src, ~)
        switch src.Value
            case 1 % Plot 1 is selected
                activePlot = 1;
                set(ax1, 'Visible', 'on');
                set(ax3, 'Visible', 'off');
                set(ax2, 'Visible', 'on');
                cla(ax3);
                set(dropdown1, 'String', single_iteration_types, 'Value', P1single_iteration_types_idx);
                set(dropdown2x, 'String', comparison_types_x, 'Value', P1comparison_type_x_idx);
                set(dropdown2y, 'String', comparison_types_y, 'Value', P1comparison_type_y_idx);
            case 2 % Plot 2 is selected
                activePlot = 2;
                set(ax3, 'Visible', 'off');
                set(ax1, 'Visible', 'on');
                set(ax2, 'Visible', 'on');
                cla(ax3);
                set(dropdown1, 'String', single_iteration_types, 'Value', P2single_iteration_types_idx);
                set(dropdown2x, 'String', comparison_types_x, 'Value', P2comparison_type_x_idx);
                set(dropdown2y, 'String', comparison_types_y, 'Value', P2comparison_type_y_idx);
            case 3 % Large Plot is selected
                activePlot = 3;
                if ~isvalid(ax3)
                        ax3 = axes('Position', [.25 .1 .7 .8], 'Visible', 'off');
                end
                set(ax1, 'Visible', 'off');
                set(ax2, 'Visible', 'off');
                set(ax3, 'Visible', 'on');
                cla(ax2);
                cla(ax1);
                set(dropdown1, 'String', single_iteration_types, 'Value', P3single_iteration_types_idx);
                set(dropdown2x, 'String', comparison_types_x, 'Value', P3comparison_type_x_idx);
                set(dropdown2y, 'String', comparison_types_y, 'Value', P3comparison_type_y_idx);
        end
    end


    function set_save_path(src, event)
        folder_name = uigetdir;
        if folder_name ~= 0 
            save_path = folder_name;
            set(save_path_display, 'String', save_path);
        end

       
    end
    function save_plot1(src, event)
        % Use the save_path from the display
        save_path = get(save_path_display, 'String');
        

        file_path = fullfile(save_path, 'Plot1.png');
        

        set(ax1, 'Visible', 'on');
        exportgraphics(ax1, file_path);
    end

    function save_plot2(src, event)
        % Use the save_path from the display
        save_path = get(save_path_display, 'String');
        
        file_path = fullfile(save_path, 'Plot2.png');
        
        set(ax2, 'Visible', 'on');
        exportgraphics(ax2, file_path);
    end

    function save_large_plot(src, event)
        save_path = get(save_path_display, 'String');
        
        file_path = fullfile(save_path, 'LargePlot.png');
        
        exportgraphics(ax3, file_path);
    end

  function display_help(src, event)
    % Help message text with HTML tags for bold
    help_message = ['<html>Plot Availability in the Application<br><br>', ...
        '<b>Single Iteration Plots</b><br>', ...
        '<ul><li><b>Available Plots:</b><br>', ...
        '  - <b>Position, Velocity, Acceleration, Accel_ref:</b><br>', ...
        '    Available if A is a non-empty instance of uarmtd_agent,<br>', ...
        '    and it has properties state, time, and these properties are not empty.<br>', ...
        '  - <b>Torque:</b><br>', ...
        '    Available if A has properties input, time, and input is not empty.<br>', ...
        '  - <b>Pos_braking, Vel_braking, Accel_braking:</b><br>', ...
        '    Available if A has a non-empty full_state,<br>', ...
        '    and it has properties full_state, full_time.<br>', ...
        '  - <b>Torq_bounds:</b><br>', ...
        '    Available if A has a non-empty full_u and properties input_constraints, input_radii.<br><br>', ...
        '</li><li><b>Unavailable Plots (marked with "NA"):</b><br>', ...
        '  - If the conditions for the available plots are not met, the corresponding plots are marked as "NA" (Not Available).<br><br>', ...
        '</li></ul><b>Comparison Types (Y-axis)</b><br>', ...
        '<ul><li><b>Available Plots:</b><br>', ...
        '  - mean_v, max_plan_t, total_real_t, n_brakes, goal_check: Available if Filelist is not empty.<br>', ...
        '  - max_tilt: Additionally available if W is an instance of kinova_grasp_world_static and Filelist is not empty.<br><br>', ...
        '</li><li><b>Unavailable Plots (marked with "NA"):</b><br>', ...
        '  - If Filelist is empty, all comparison types are marked as "NA".<br><br>', ...
        '</li></ul><b>Comparison Types (X-axis)</b><br>', ...
        '<ul><li><b>Available Plots (depends on Type):</b><br>', ...
        '  - k_range: Available if Type is ''KR''.<br>', ...
        '  - n_timesteps: Available if Type is ''ST''.<br>', ...
        '  - s_thresh: Available if Type is ''TS''.<br><br>', ...
        '</li><li><b>Unavailable Plots (marked with "NA"):</b><br>', ...
        '  - If Filelist is empty or if the conditions for a specific Type are not met, ', ...
        '    the corresponding plots are marked as "NA".<br>', ...
        '</li></ul></html>'];


    fig_width = 600;
    fig_height = 500;
    f = figure('Name', 'Help', 'NumberTitle', 'off', 'MenuBar', 'none', 'Position', [100 100 fig_width fig_height], 'Resize', 'off');
    jTextPane = javax.swing.JTextPane();
    jTextPane.setContentType('text/html');
    jTextPane.setText(help_message);
    jTextPane.setEditable(false);

    jScrollPane = javax.swing.JScrollPane(jTextPane);


    jScrollPane.setVerticalScrollBarPolicy(javax.swing.ScrollPaneConstants.VERTICAL_SCROLLBAR_AS_NEEDED);

   
    [~, container] = javacomponent(jScrollPane, [0, 0, fig_width, fig_height], f);
    set(container, 'Units', 'normalized', 'Position', [0 0 1 1]);
    end





    function spinner_callback(src, ~)
        % Get the current value from the spinner
        value = str2double(get(src, 'String'));

        % Ensure the value is within the allowed range
        value = min(max(0, value), iterations);

        if (value == 0)
            set(src, 'String', "All");
        else
            set(src, 'String', num2str(value));
        end
        

        set(main_fig, 'UserData', value);
    end
    function dropdown1_callback(src, ~)
        if ~single_iteration_selectable(src.Value)
            % Handle unselectable option
            disp('Selected option is not available. Check help to see what is needed!');
            src.Value = find(single_iteration_selectable, 1); % Find the first selectable option
        else
            spinner_value = str2double(get(spinner, 'String'));
            switch activePlot
                case 1
                    P1single_iteration_types_idx = src.Value;
                    P1comparison_type_x_idx = 1; % Reset to 'Off'
                    P1comparison_type_y_idx = 1; % Reset to 'Off'
                    plot_single_iter(ax1, P1single_iteration_types_idx, spinner_value);
                case 2
                    P2single_iteration_types_idx = src.Value;
                    P2comparison_type_x_idx = 1; % Reset to 'Off'
                    P2comparison_type_y_idx = 1; % Reset to 'Off'
                    plot_single_iter(ax2, P2single_iteration_types_idx, spinner_value);
                case 3
                    P3single_iteration_types_idx = src.Value;
                    P3comparison_type_x_idx = 1; % Reset to 'Off'
                    P3comparison_type_y_idx = 1; % Reset to 'Off'
                    plot_single_iter(ax3, P3single_iteration_types_idx, spinner_value);
            end
            set(dropdown2x, 'Value', 1);
            set(dropdown2y, 'Value', 1);
        end
    end

    
    function dropdown2x_callback(src, ~)
    if ~comparison_types_x_selectable(src.Value)
        % Handle unselectable option
        disp('Selected option is not available. Check help to see what is needed!');
        src.Value = find(comparison_types_x_selectable, 1);
    else
        switch activePlot
            case 1
                P1comparison_type_x_idx = src.Value;
                P1single_iteration_types_idx = 1; % Reset to 'Off'
                try
                    plot_comparison(ax1, P1comparison_type_x_idx, P1comparison_type_y_idx);
                catch exeption
                end
            case 2
                P2comparison_type_x_idx = src.Value;
                P2single_iteration_types_idx = 1; % Reset to 'Off'
                try
                    plot_comparison(ax2, P2comparison_type_x_idx, P2comparison_type_y_idx);
                catch exeption
                end
            case 3
                P3comparison_type_x_idx = src.Value;
                P3single_iteration_types_idx = 1; % Reset to 'Off'
                try
                    plot_comparison(ax3, P3comparison_type_x_idx, P3comparison_type_y_idx);
                catch exeption
                end
        end
        set(dropdown1, 'Value', 1);
    end
    end

    function dropdown2y_callback(src, ~)
    if ~comparison_types_y_selectable(src.Value)
        % Handle unselectable option
        disp('Selected option is not available. Check help to see what is needed!');
        src.Value = find(comparison_types_y_selectable, 1);
    else
        switch activePlot
            case 1
                P1comparison_type_y_idx = src.Value;
                P1single_iteration_types_idx = 1; % Reset to 'Off'
                try
                    plot_comparison(ax1, P1comparison_type_x_idx, P1comparison_type_y_idx);
                catch exeption
                end
            case 2
                P2comparison_type_y_idx = src.Value;
                P2single_iteration_types_idx = 1; % Reset to 'Off'
                try
                    plot_comparison(ax2, P2comparison_type_x_idx, P2comparison_type_y_idx);
                catch exeption
                end
            case 3
                P3comparison_type_y_idx = src.Value;
                P3single_iteration_types_idx = 1; % Reset to 'Off'
                try
                    plot_comparison(ax3, P3comparison_type_x_idx, P3comparison_type_y_idx);
                catch exeption
                end
        end
        set(dropdown1, 'Value', 1);
    end
    end

    function [single_iter, single_select] = determineSingleIterationTypes()
    single_iter = {"Off"};
    single_select = [1,0,0,0, 0,0,0,0,0,0,0];
    if ~isempty(A) && isa(A, 'uarmtd_agent')
    if all(arrayfun(@(x) isprop(x, 'state'), A)) && all(arrayfun(@(x) isprop(x, 'time'), A)) && ~isempty(A.state)
        single_iter{end+1} = "Position";
        single_iter{end+1} = "Velocity";
        single_iter{end+1} = "Acceleration";
        single_iter{end+1} = "Accel_ref";
        single_select = [1,1,1,1,1,0,0,0,0,0,0];
    else
        single_iter{end+1} = "Position (NA)";
        single_iter{end+1} = "Velocity (NA)";
        single_iter{end+1} = "Acceleration (NA)";
        single_iter{end+1} = "Accel_ref (NA)";
        single_select = [1,0,0,0,0,0,0,0,0,0,0];
    end
    if isprop(A, 'input') && isprop(A, 'time') && ~isempty(A.input)
        single_iter{end+1} = "Torque";
        single_select(6) = 1;
    else
        single_iter{end+1} = "Torque (NA)";
        single_select(6) = 0;
    end
    if isprop(A, 'full_state') && ~isempty(A.full_state) && isprop(A, 'full_time')
        single_iter{end+1} = "Pos_braking";
        single_iter{end+1} = "Vel_braking";
        single_iter{end+1} = "Accel_braking";
        single_select(7:9) = 1;
    else
        single_iter{end+1} = "Pos_brake (NA)";
        single_iter{end+1} = "Vel_brake (NA)";
        single_iter{end+1} = "Accel_brake (NA)";
        single_select(7:9) = 0;
    end
    if isprop(A, 'full_u') && ~isempty(A.full_u) && isprop(A, 'input_constraints') && isprop(A, 'input_radii')
        single_iter{end+1} = "Torq_bounds";
        single_select(10) = 1;
    else
        single_iter{end+1} = "Torq_bound (NA)";
        single_select(10) = 0;
    end
    if isprop(P, 'u_s') && ~isempty(P.u_s)
        single_iter{end+1} = "Force Cone";
        single_select(11) = 1;
    else
        single_iter{end+1} = "Force Cone (NA)";
        single_select(11) = 0;
    end
    else
        single_iter{end+1} = "Position (NA)";
        single_iter{end+1} = "Velocity (NA)";
        single_iter{end+1} = "Acceleration (NA)";
        single_iter{end+1} = "Accel_ref (NA)";
        single_iter{end+1} = "Torque (NA)";
        single_iter{end+1} = "Pos_brake (NA)";
        single_iter{end+1} = "Vel_brake (NA)";
        single_iter{end+1} = "Accel_brake (NA)";
        single_iter{end+1} = "Torq_bound (NA)";
        single_iter{end+1} = "Force Cone (NA)";
    end
    end
    function [compar_y, compar_y_select] = determineComparisonTypesY()
    if ~isempty(Filelist)
        compar_y = {'Off', 'mean_v', 'max_plan_t', 'total_real_t', 'n_brakes', 'max_tilt (NA)', 'goal_check'};
        compar_y_select = [1,1,1,1,1, 0, 1];
    else
        compar_y = {'Off', 'mean_v (NA)', 'max_plan_t (NA)', 'mean_plan_t (NA)', 'n_brakes (NA)', 'max_tilt (NA)', 'goal_check (NA)'};
        compar_y_select = [1,0,0,0,0, 0, 0];

    end
    if isa(W, 'kinova_grasp_world_static') && ~isempty(Filelist)
        compar_y = {'Off', 'mean_v', 'max_plan_t', 'total_real_t', 'n_brakes', 'max_tilt', 'goal_check'};
        compar_y_select = [1,1,1,1,1, 1, 1];
    end
    end
    function [compar_x, compar_x_select] = determineComparisonTypesX()
    if ~isempty(Filelist)
        
        if Type=='KR'
            compar_x_select = [1,1,0,0];
            compar_x = {'Off','k_range', 'n_timesteps (NA)', "s_thresh (NA)"};
        elseif Type=='TS'
            compar_x_select = [1,0,1,0];
            compar_x = {'Off','k_range (NA)', 'n_timesteps', "s_thresh (NA)"};
        else
            compar_x_select = [1,0,0,1];
            compar_x = {'Off','k_range (NA)', 'n_timesteps (NA)', "s_thresh"};
        end
        
    else
        compar_x = {'Off','k_range (NA)', 'n_timesteps (NA)', "s_thresh (NA)"};
        compar_x_select = [1,0,0,0];
    end
    end
    function plot_single_iter(plt, idx, val)
        cla(plt);
        if (idx==1)
            title(plt, 'Plot')
        elseif (idx==2)
            %plot(plt, A.time,A.state(A.joint_state_indices,:))
%             fig = get(plt, 'Parent');
%             subplot(2, 1, 2, 'Parent', fig);
%             plot(A.time, A.state(A.joint_speed_indices, :));
            plot(plt, A.time,A.state(A.joint_state_indices,:))
            title(plt, 'Joint Positions')
            ylabel(plt, 'Joint Angle (rad)')
            xlabel(plt, 'Time(s)')

        elseif (idx==3)
            plot(plt, A.time,A.state(A.joint_speed_indices,:))
            title(plt, 'Joint Velocities')
            ylabel(plt, 'Vel (m/s)')
            xlabel(plt, 'Time(s)')
        elseif (idx==4)
            joint_angles = A.state(A.joint_state_indices,:);
            joint_angular_velocity = A.state(A.joint_speed_indices,:);
            
            qdd_post = zeros(7,length(A.time));
            % calculating the acceleration in post to compare with what is stored
            for i = 2:length(A.time)
                [M, C, g] = A.calculate_dynamics(joint_angles(:,i), joint_angular_velocity(:,i), A.params.true);
            
                for j = 1:A.n_inputs
                    M(j,j) = M(j,j) + A.transmision_inertia(j);
                end
                % can I call u=A.LLC.get_control_inputs() here with the P.info?
                
                qdd_post(:,i) = M\(A.input(:,i)-C*joint_angular_velocity(:,i)-g);
            %         qdd_post(:,i) = M\(A.input(:,i)-C*joint_angular_velocity(:,i)-g);
            end
            plot(plt, A.time,qdd_post)
            title(plt, "Calculated Joint Accelerations")
            ylabel(plt, 'Accel (m/s^2)')
            xlabel(plt, 'Time(s)')
        elseif (idx==5)
            plot(plt, A.time,A.reference_acceleration)
            title(plt, "Reference Joint Accelerations")
            ylabel(plt, 'Accel ref (m/s^2)')
            xlabel(plt, 'Time(s)')
        elseif (idx==6)
            plot(plt, A.time,A.input)
            title(plt, 'Joint Torques')
            ylabel(plt, 'Torque')
            xlabel(plt, 'Time(s)')
        elseif (idx==7)
            Ys = A.full_state(A.joint_state_indices, :);
            T = A.full_time;
            joint = str2double(get(spinner, 'String'));
            plot_bounded_trajectories(plt, A, T ,Ys, -1, traj_len, braking_frac, joint);
        elseif (idx==8)
            Yv = A.full_state(A.joint_speed_indices, :);
            T = A.full_time;
            joint = str2double(get(spinner, 'String'));
            plot_bounded_trajectories(plt, A, T ,Yv, -1, traj_len, braking_frac, joint);
        elseif (idx==9)
            joint = str2double(get(spinner, 'String'));
            joint_angles = A.full_state(A.joint_state_indices,:);
            joint_angular_velocity = A.full_state(A.joint_speed_indices,:);
            
            qdd_post = zeros(7,length(A.full_time));
            % calculating the acceleration in post to compare with what is stored
            for i = 2:length(A.full_time)
                [M, C, g] = A.calculate_dynamics(joint_angles(:,i), joint_angular_velocity(:,i), A.params.true);
            
                for j = 1:A.n_inputs
                    M(j,j) = M(j,j) + A.transmision_inertia(j);
                end
                % can I call u=A.LLC.get_control_inputs() here with the P.info?
                
                qdd_post(:,i) = M\(A.full_u(:,i)-C*joint_angular_velocity(:,i)-g);
            %         qdd_post(:,i) = M\(A.input(:,i)-C*joint_angular_velocity(:,i)-g);
            
            end
            Ya = qdd_post;
            T = A.full_time;

            plot_bounded_trajectories(plt, A, T ,Ya, -1, traj_len, braking_frac, joint);
        elseif (idx==10)
            iteration = val;
            plot_torques(A.joint_input_limits, A.full_u, A.input_constraints, A.input_radii, A.full_time, iteration, 0)
        elseif (idx==11)
            iteration = val;
            plot_force(iteration, A.fs, A.ns, A.f_rs_c, A.f_rs_r, A.n_rs_c, A.n_rs_r, A.f_const);
        end
    end
    function plot_comparison(plt, xidx, yidx)
        Filelist;
        disp("PLOTTING COMPARISON, this takes a second...")
        y_data = [];
        x_data = [];
        x_label = '';
        y_label = '';
        for idx = 1:length(Filelist)
            loaded_data = load(Filelist{idx});
            disp(loaded_data)
            % Assign x_data and labels based on xidx
            switch xidx
                case 2

                    x_data(idx) = loaded_data.k_range(1);
                    x_label = 'k range';
                case 3

                    x_data(idx) = loaded_data.n_t;
                    x_label = 'ntsteps';
                case 4

                    x_data(idx) = loaded_data.s_thresh;
                    x_label = 'sthresh';
            end
    
            % Assign y_data and labels based on yidx
            switch yidx
                case 2

                    y_data(:, idx) = abs(mean(loaded_data.A.state(loaded_data.A.joint_speed_indices, :)'))';
                    y_label = 'mean v';
                case 3

                    y_data(idx) = max(loaded_data.summary.planning_time);
                    y_label = 'max plan t';
                case 4
                  
                    y_data(idx) = loaded_data.summary.total_real_time;
                    y_label = 'total real t';
                case 5

                    y_data(idx) = sum(loaded_data.summary.stop_check);
                    y_label = 'n brakes';
                case 6

                    [max_tilt_angle, ~] = max(abs(loaded_data.A.state(13,:)));
                    y_data(idx) = rad2deg(max_tilt_angle);
                    y_label = 'max tilt angle';
                case 7

                    y_data(idx) = loaded_data.summary.goal_check;
                    y_label = 'goal check';
            end
        end
        disp(x_data)
        % Plotting based on y_data type
        switch yidx
            case 2 % mean_v
                disp("Plotting_V")
                plot(plt, x_data, y_data'); % Plot the ith row of y against x
                ylabel(plt, 'Mean Joint Speed');
                title(plt, 'Mean Joint Speeds');
                legend(plt, 'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7');
            case {3, 4, 5, 6} % max_plan_t, total_real_t, n_brakes, max_tilt_angle
                bar(plt, x_data, y_data);
                ylabel(plt, y_label);
                title(plt, y_label);
            case 7 % goal_check
                % Color based on goal_check
                for idx = 1:length(x_data)
                    barColor = 'r'; % Default to red
                    if y_data(idx) == 1
                        barColor = 'g'; % Green for goal_check == 1
                    end
                    bar(plt, idx, x_data(idx), barColor);
                    hold(plt, 'on');
                end
                xlabel(plt, x_label);
                ylabel(plt, 'Goal Check');
                title(plt, 'Goal Check with ' + x_label);
                hold(plt, 'off');
        end
    
        xlabel(plt, x_label);
    end

end
%% Create a fileList if needed:
function [FileList, type] = generateFileList(directory)
    FileList = {};
    if contains(directory, 'KR')
        filePattern = 'KR_iteration_*.mat';
        type = 'KR';
    elseif contains(directory, 'ST')
        filePattern = 'ST_iteration_*.mat';
        type = 'ST';
    elseif contains(directory, 'TS')
        filePattern = 'TS_iteration_*.mat';
        type = 'TS';
    else
        error('Directory does not contain KR, ST, or TS in its path.');
    end
    files = dir(fullfile(directory, filePattern));

    fileNames = {files.name};

    iterationNumbers = regexp(fileNames, '\d+', 'match');
    iterationNumbers = str2double([iterationNumbers{:}]);

    [~, sortIdx] = sort(iterationNumbers);
    sortedFileNames = fileNames(sortIdx);

    FileList = fullfile(directory, sortedFileNames);
end




