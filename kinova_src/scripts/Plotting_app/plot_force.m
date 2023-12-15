function plot_force(iteration, f_nom, n_nom, f_rs_c, f_rs_r, n_rs_c, n_rs_r, force_constraint_values)
    % Check if specific iteration is requested
    if iteration ~= -1
        f_nom = f_nom(iteration, :);
        n_nom = n_nom(iteration, :);
        f_rs_c = f_rs_c(iteration, :);
        f_rs_r = f_rs_r(iteration, :);
        n_rs_c = n_rs_c(iteration, :);
        n_rs_r = n_rs_r(iteration, :);
        force_constraint_values = force_constraint_values(iteration, :);
        ts = iteration;
    end

    % Calculate upper and lower bounds of the force and moment reach sets
    f_ub = f_rs_c + f_rs_r;
    f_lb = f_rs_c - f_rs_r;
    n_ub = n_rs_c + n_rs_r;
    n_lb = n_rs_c - n_rs_r;
    disp(ts)

    % Plot Force and Moment Reach Sets
    fig_num = figure; 
    hold on;

    plot_label = {'X-axis', 'Y-axis', 'Z-axis'};
    for i = 1:3
        subplot(3,2,i*2-1)
        hold on
        plot(ts, f_nom(:,i), 'LineWidth', 2, 'Color', [0.8500, 0.3250, 0.0980]);
        plot(ts, f_ub(:,i), '-b')
        plot(ts, f_lb(:,i), '-b')
        title([plot_label{i}, ' Force'])
        xlabel('Time (sec)')
        ylabel('Force (Newton)')
    end
    for i = 1:3
        subplot(3,2,i*2)
        hold on
        plot(ts, n_nom(:,i), 'LineWidth', 2, 'Color', [0.8500, 0.3250, 0.0980]);
        plot(ts, n_ub(:,i), '-b')
        plot(ts, n_lb(:,i), '-b')
        title([plot_label{i}, ' Moment'])
        xlabel('Time (sec)')
        ylabel('Moment (Newton*meter)')
    end

    % Plot Constraints
%     sgtitle('Force and Moment Reach Sets and Constraints')
%     constraint_label = {'Separation Constraint', 'Slipping Constraint', 'Tipping Constraint'};
%     for i = 1:3
%         subplot(3,1,i)
%         hold on
%         plot(ts, force_constraint_values((1+(i-1)*100):(100+(i-1)*100), 1), 'b-')
%         plot(ts, force_constraint_values((1+(i-1)*100):(100+(i-1)*100), 2), 'b-')
%         title(constraint_label{i})
%         xlabel('Time (sec)')
%         ylabel('Constraint Value')
%     end
end
