clc;
clear;
close all;

%% =========================================================
% DATA-ASSISTED DISSIPATIVITY VS DATA-DRIVEN DISSIPATIVITY
%
% Proposition 4:
%   Data-Assisted Dissipativity Controller
%   - Recovers A_est and B_est from data
%   - Designs the dissipativity controller using the recovered model
%
% Proposition 5:
%   Data-Driven Dissipativity Controller
%   - Uses measured data directly
%   - Does not recover A_est or B_est for controller synthesis
%
% This script only loads saved simulation results.
% It does not redesign either controller or rerun either simulation.
%
% State vector:
%   x = [position; velocity; body angle; angular velocity]
%% =========================================================

fprintf('====================================================\n');
fprintf('DATA-ASSISTED VS DATA-DRIVEN DISSIPATIVITY\n');
fprintf('====================================================\n');

%% =========================================================
% RESULT FILES
% =========================================================

% Use the folder containing this comparison script.
script_path = mfilename('fullpath');
if isempty(script_path)
    script_folder = pwd;
else
    script_folder = fileparts(script_path);
end

data_assisted_file = fullfile(script_folder, 'Proposition 4\SBR_DataAssisted_Dissipativity_Controller_Result.mat');

data_driven_file = fullfile(script_folder, 'Proposition 5\SBR_DataDriven_Dissipativity_Controller_Result.mat');

if exist(data_assisted_file, 'file') ~= 2
    error('Could not find the data-assisted result file:\n%s', data_assisted_file);
end

if exist(data_driven_file, 'file') ~= 2
    error('Could not find the data-driven result file:\n%s', data_driven_file);
end

DA = load(data_assisted_file);
DD = load(data_driven_file);

fprintf('Loaded result files successfully.\n');

%% =========================================================
% VERIFY REQUIRED VARIABLES
% =========================================================

required_fields = {'x_test','u_test','Ts'};

for i = 1:numel(required_fields)
    field_name = required_fields{i};

    if ~isfield(DA, field_name)
        error('Data-assisted result is missing variable: %s', field_name);
    end

    if ~isfield(DD, field_name)
        error('Data-driven result is missing variable: %s', field_name);
    end
end

%% =========================================================
% EXTRACT SAVED RESULTS
% =========================================================

x_da = DA.x_test;
u_da = DA.u_test;

x_dd = DD.x_test;
u_dd = DD.u_test;

% Unsaturated controller command, when available
if isfield(DA, 'u_cmd_test')
    u_cmd_da = DA.u_cmd_test;
else
    u_cmd_da = u_da;
end

if isfield(DD, 'u_cmd_test')
    u_cmd_dd = DD.u_cmd_test;
else
    u_cmd_dd = u_dd;
end

Ts_da = DA.Ts;
Ts_dd = DD.Ts;

if abs(Ts_da - Ts_dd) > 1e-12
    error(['The result files use different sampling times.\n' ...
           'Data-assisted Ts = %.12g s\n' ...
           'Data-driven Ts   = %.12g s'], Ts_da, Ts_dd);
end

Ts = Ts_da;

%% =========================================================
% MATCH RESULT LENGTHS
% =========================================================

N_da = size(x_da, 2) - 1;
N_dd = size(x_dd, 2) - 1;
Ntest = min(N_da, N_dd);

x_da = x_da(:, 1:Ntest+1);
x_dd = x_dd(:, 1:Ntest+1);

u_da = u_da(:, 1:Ntest);
u_dd = u_dd(:, 1:Ntest);

u_cmd_da = u_cmd_da(:, 1:Ntest);
u_cmd_dd = u_cmd_dd(:, 1:Ntest);

t_state = (0:Ntest) * Ts;
t_input = (0:Ntest-1) * Ts;

%% =========================================================
% DISTURBANCE INFORMATION
% =========================================================

if isfield(DA, 'disturbance_time')
    disturbance_time = DA.disturbance_time;
elseif isfield(DD, 'disturbance_time')
    disturbance_time = DD.disturbance_time;
else
    disturbance_time = 10;
end

if isfield(DA, 'dist_step')
    dist_step = round(DA.dist_step);
elseif isfield(DD, 'dist_step')
    dist_step = round(DD.dist_step);
else
    dist_step = round(disturbance_time/Ts) + 1;
end

if isfield(DA, 'angle_threshold_deg')
    angle_threshold_deg = DA.angle_threshold_deg;
elseif isfield(DD, 'angle_threshold_deg')
    angle_threshold_deg = DD.angle_threshold_deg;
else
    angle_threshold_deg = 1;
end

% In the controller scripts, the disturbance is added to x(:,k+1)
% when k == dist_step. Therefore, the first actually disturbed state is
% MATLAB column dist_step+1.
disturbed_state_index = min(dist_step + 1, Ntest + 1);

% Require the body angle to remain inside the threshold for this duration.
settling_hold_time = 0.5;
settling_hold_samples = max(1, round(settling_hold_time / Ts));

%% =========================================================
% CONTROLLER INFORMATION
% =========================================================

if isfield(DA, 'gamma_val')
    gamma_da = DA.gamma_val;
else
    gamma_da = NaN;
end

if isfield(DD, 'gamma_val')
    gamma_dd = DD.gamma_val;
else
    gamma_dd = NaN;
end

if isfield(DA, 'K_diss')
    K_da = DA.K_diss;
else
    K_da = NaN(1, size(x_da,1));
end

if isfield(DD, 'K_data')
    K_dd = DD.K_data;
else
    K_dd = NaN(1, size(x_dd,1));
end

%% =========================================================
% CALCULATE COMPARISON METRICS
% =========================================================

metrics_da = calculate_metrics( ...
    x_da, u_da, u_cmd_da, Ts, disturbed_state_index, ...
    angle_threshold_deg, settling_hold_samples);

metrics_dd = calculate_metrics( ...
    x_dd, u_dd, u_cmd_dd, Ts, disturbed_state_index, ...
    angle_threshold_deg, settling_hold_samples);

%% =========================================================
% RESULT INFORMATION
% =========================================================

fprintf('\n====================================================\n');
fprintf('RESULT INFORMATION\n');
fprintf('====================================================\n');
fprintf('Sampling time                    = %.4f s\n', Ts);
fprintf('Compared samples                 = %d\n', Ntest);
fprintf('Compared simulation duration     = %.2f s\n', Ntest*Ts);
fprintf('Nominal disturbance time         = %.2f s\n', disturbance_time);
fprintf('First disturbed state time       = %.2f s\n', ...
    t_state(disturbed_state_index));
fprintf('Recovery threshold               = +/- %.2f deg\n', ...
    angle_threshold_deg);
fprintf('Required threshold hold time     = %.2f s\n', ...
    settling_hold_time);

fprintf('\nData-Assisted gain K_diss:\n');
disp(K_da);
fprintf('Data-Driven gain K_data:\n');
disp(K_dd);

%% =========================================================
% SUMMARY TABLE
% =========================================================

Metric = { ...
    'Dissipativity gamma';
    'Initial angle (deg)';
    'Peak angle after disturbance (deg)';
    'Recovery time, first entry (s)';
    'Recovery time, sustained (s)';
    'Angle IAE after disturbance (deg*s)';
    'Final angle (deg)';
    'Final position (m)';
    'Final velocity (m/s)';
    'Final angular velocity (deg/s)';
    'Maximum absolute position (m)';
    'Maximum absolute velocity (m/s)';
    'Maximum absolute angular velocity (deg/s)';
    'Maximum saturated control input';
    'RMS saturated control input';
    'Maximum commanded control input'};

Data_Assisted = [ ...
    gamma_da;
    metrics_da.initial_angle_deg;
    metrics_da.peak_angle_post_dist_deg;
    metrics_da.recovery_first_entry_s;
    metrics_da.recovery_sustained_s;
    metrics_da.angle_iae_post_dist_deg_s;
    metrics_da.final_angle_deg;
    metrics_da.final_position_m;
    metrics_da.final_velocity_m_s;
    metrics_da.final_ang_velocity_deg_s;
    metrics_da.max_position_m;
    metrics_da.max_velocity_m_s;
    metrics_da.max_ang_velocity_deg_s;
    metrics_da.max_abs_u;
    metrics_da.rms_u;
    metrics_da.max_abs_u_cmd];

Data_Driven = [ ...
    gamma_dd;
    metrics_dd.initial_angle_deg;
    metrics_dd.peak_angle_post_dist_deg;
    metrics_dd.recovery_first_entry_s;
    metrics_dd.recovery_sustained_s;
    metrics_dd.angle_iae_post_dist_deg_s;
    metrics_dd.final_angle_deg;
    metrics_dd.final_position_m;
    metrics_dd.final_velocity_m_s;
    metrics_dd.final_ang_velocity_deg_s;
    metrics_dd.max_position_m;
    metrics_dd.max_velocity_m_s;
    metrics_dd.max_ang_velocity_deg_s;
    metrics_dd.max_abs_u;
    metrics_dd.rms_u;
    metrics_dd.max_abs_u_cmd];

comparison_table = table(Metric, Data_Assisted, Data_Driven);

fprintf('\n====================================================\n');
fprintf('COMPARISON SUMMARY\n');
fprintf('====================================================\n');
disp(comparison_table);

%% =========================================================
% CREATE OUTPUT FOLDER
% =========================================================

output_folder = fullfile(script_folder, ...
    'comparison_graphs_data_assisted_vs_data_driven_dissipativity');

if exist(output_folder, 'dir') ~= 7
    mkdir(output_folder);
end

lw = 2;

%% =========================================================
% BODY ANGLE COMPARISON
% =========================================================

figure('Name','Body Angle Comparison');
plot(t_state, rad2deg(x_da(3,:)), 'LineWidth', lw);
hold on;
plot(t_state, rad2deg(x_dd(3,:)), '--', 'LineWidth', lw);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
yline(angle_threshold_deg, ':', '+1 deg', 'LineWidth', 1.0);
yline(-angle_threshold_deg, ':', '-1 deg', 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('\phi (deg)');
title('Body Angle: Data-Assisted vs Data-Driven Dissipativity');
legend('Data-Assisted Dissipativity', ...
       'Data-Driven Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'body_angle_comparison.png'));

%% =========================================================
% CONTROL INPUT COMPARISON
% =========================================================

figure('Name','Control Input Comparison');
plot(t_input, u_da(1,:), 'LineWidth', lw);
hold on;
plot(t_input, u_dd(1,:), '--', 'LineWidth', lw);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Control input u');
title('Control Input: Data-Assisted vs Data-Driven Dissipativity');
legend('Data-Assisted Dissipativity', ...
       'Data-Driven Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'control_input_comparison.png'));

%% =========================================================
% ROBOT POSITION COMPARISON
% =========================================================

figure('Name','Robot Position Comparison');
plot(t_state, x_da(1,:), 'LineWidth', lw);
hold on;
plot(t_state, x_dd(1,:), '--', 'LineWidth', lw);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Position x (m)');
title('Robot Position: Data-Assisted vs Data-Driven Dissipativity');
legend('Data-Assisted Dissipativity', ...
       'Data-Driven Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'robot_position_comparison.png'));

%% =========================================================
% ROBOT VELOCITY COMPARISON
% =========================================================

figure('Name','Robot Velocity Comparison');
plot(t_state, x_da(2,:), 'LineWidth', lw);
hold on;
plot(t_state, x_dd(2,:), '--', 'LineWidth', lw);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Velocity xdot (m/s)');
title('Robot Velocity: Data-Assisted vs Data-Driven Dissipativity');
legend('Data-Assisted Dissipativity', ...
       'Data-Driven Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'robot_velocity_comparison.png'));

%% =========================================================
% ANGULAR VELOCITY COMPARISON
% =========================================================

figure('Name','Angular Velocity Comparison');
plot(t_state, rad2deg(x_da(4,:)), 'LineWidth', lw);
hold on;
plot(t_state, rad2deg(x_dd(4,:)), '--', 'LineWidth', lw);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('\phidot (deg/s)');
title('Angular Velocity: Data-Assisted vs Data-Driven Dissipativity');
legend('Data-Assisted Dissipativity', ...
       'Data-Driven Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'angular_velocity_comparison.png'));

%% =========================================================
% FULL COMPARISON FIGURE
% =========================================================

figure('Name','Full Dissipativity Comparison', ...
       'Position', [100 100 950 900]);

subplot(5,1,1);
plot(t_state, rad2deg(x_da(3,:)), 'LineWidth', 1.6);
hold on;
plot(t_state, rad2deg(x_dd(3,:)), '--', 'LineWidth', 1.6);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
yline(angle_threshold_deg, ':', 'LineWidth', 0.8);
yline(-angle_threshold_deg, ':', 'LineWidth', 0.8);
grid on;
ylabel('\phi (deg)');
title('Data-Assisted vs Data-Driven Dissipativity Controller Comparison');
legend('Data-Assisted', 'Data-Driven', 'Location', 'best');

subplot(5,1,2);
plot(t_input, u_da(1,:), 'LineWidth', 1.6);
hold on;
plot(t_input, u_dd(1,:), '--', 'LineWidth', 1.6);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
grid on;
ylabel('u');

subplot(5,1,3);
plot(t_state, x_da(1,:), 'LineWidth', 1.6);
hold on;
plot(t_state, x_dd(1,:), '--', 'LineWidth', 1.6);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
grid on;
ylabel('x (m)');

subplot(5,1,4);
plot(t_state, x_da(2,:), 'LineWidth', 1.6);
hold on;
plot(t_state, x_dd(2,:), '--', 'LineWidth', 1.6);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
grid on;
ylabel('xdot (m/s)');

subplot(5,1,5);
plot(t_state, rad2deg(x_da(4,:)), 'LineWidth', 1.6);
hold on;
plot(t_state, rad2deg(x_dd(4,:)), '--', 'LineWidth', 1.6);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('\phidot (deg/s)');

saveas(gcf, fullfile(output_folder, 'full_comparison.png'));

%% =========================================================
% SAVE COMPARISON DATA
% =========================================================

comparison_file = fullfile(output_folder, ...
    'SBR_Comparison_DataAssisted_vs_DataDriven_Dissipativity.mat');

save(comparison_file, ...
    'x_da', 'x_dd', ...
    'u_da', 'u_dd', ...
    'u_cmd_da', 'u_cmd_dd', ...
    'K_da', 'K_dd', ...
    'gamma_da', 'gamma_dd', ...
    'metrics_da', 'metrics_dd', ...
    'comparison_table', ...
    'Ts', 'Ntest', ...
    't_state', 't_input', ...
    'disturbance_time', 'dist_step', ...
    'disturbed_state_index', ...
    'angle_threshold_deg', ...
    'settling_hold_time');

fprintf('\n====================================================\n');
fprintf('COMPARISON COMPLETED SUCCESSFULLY\n');
fprintf('====================================================\n');
fprintf('Graphs saved in:\n%s\n', output_folder);
fprintf('Comparison data saved as:\n%s\n', comparison_file);

%% =========================================================
% LOCAL FUNCTION: PERFORMANCE METRICS
% =========================================================

function metrics = calculate_metrics( ...
    x, u, u_cmd, Ts, disturbed_state_index, ...
    angle_threshold_deg, settling_hold_samples)

    angle_deg = rad2deg(x(3,:));
    angular_velocity_deg_s = rad2deg(x(4,:));

    post_angle_abs = abs(angle_deg(disturbed_state_index:end));

    % First threshold entry after the disturbed state
    first_entry_relative = find( ...
        post_angle_abs <= angle_threshold_deg, 1, 'first');

    if isempty(first_entry_relative)
        recovery_first_entry_s = NaN;
    else
        recovery_first_entry_s = ...
            (first_entry_relative - 1) * Ts;
    end

    % Sustained threshold entry: angle must remain inside the threshold
    % for the specified number of samples.
    recovery_sustained_s = NaN;

    last_start = numel(post_angle_abs) - settling_hold_samples + 1;

    for k = 1:max(0, last_start)
        window = post_angle_abs(k:k+settling_hold_samples-1);

        if all(window <= angle_threshold_deg)
            recovery_sustained_s = (k - 1) * Ts;
            break;
        end
    end

    metrics.initial_angle_deg = angle_deg(1);
    metrics.peak_angle_post_dist_deg = max(post_angle_abs);
    metrics.recovery_first_entry_s = recovery_first_entry_s;
    metrics.recovery_sustained_s = recovery_sustained_s;
    metrics.angle_iae_post_dist_deg_s = sum(post_angle_abs) * Ts;

    metrics.final_angle_deg = angle_deg(end);
    metrics.final_position_m = x(1,end);
    metrics.final_velocity_m_s = x(2,end);
    metrics.final_ang_velocity_deg_s = angular_velocity_deg_s(end);

    metrics.max_position_m = max(abs(x(1,:)));
    metrics.max_velocity_m_s = max(abs(x(2,:)));
    metrics.max_ang_velocity_deg_s = max(abs(angular_velocity_deg_s));

    metrics.max_abs_u = max(abs(u(:)));
    metrics.rms_u = sqrt(mean(u(:).^2));
    metrics.max_abs_u_cmd = max(abs(u_cmd(:)));
end
