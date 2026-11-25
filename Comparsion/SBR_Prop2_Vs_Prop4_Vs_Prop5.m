clc;
clear;
close all;

%% =========================================================
% THREE-WAY COMPARISON FROM SAVED RESULT FILES
%
% Controller 1:
%   Pure Data-Driven Controller
%   SBR_DataDriven_Controller_Disturbance_Result.mat
%
% Controller 2:
%   Data-Assisted Dissipativity Controller
%   SBR_DataAssisted_Dissipativity_Controller_Result.mat
%
% Controller 3:
%   Data-Driven Dissipativity Controller
%   SBR_DataDriven_Dissipativity_Controller_Result.mat
%
% This script only loads saved results.
% It does NOT redesign controllers.
% It does NOT rerun simulations.
%
% State:
%   x = [position; velocity; body angle; angular velocity]
%
% Plots:
%   1) Body angle
%   2) Control input
%   3) Robot position
%   4) Robot velocity
%   5) Angular velocity
%   6) Full combined subplot
%% =========================================================

disp('====================================================');
disp('THREE-WAY DATA CONTROLLER COMPARISON');
disp('====================================================');

%% =========================================================
% RESULT FILES
% =========================================================

data_driven_file = 'C:\Users\shris\Desktop\CPE 800 Project\Proposition 2\SBR_DataDriven_Controller_Disturbance_Result.mat';
data_assisted_diss_file = 'C:\Users\shris\Desktop\CPE 800 Project\Proposition 4\SBR_DataAssisted_Dissipativity_Controller_Result.mat';
data_driven_diss_file = 'C:\Users\shris\Desktop\CPE 800 Project\Proposition 5\SBR_DataDriven_Dissipativity_Controller_Result.mat';

if exist(data_driven_file, 'file') ~= 2
    error('Could not find %s in the current folder.', data_driven_file);
end

if exist(data_assisted_diss_file, 'file') ~= 2
    error('Could not find %s in the current folder.', data_assisted_diss_file);
end

if exist(data_driven_diss_file, 'file') ~= 2
    error('Could not find %s in the current folder.', data_driven_diss_file);
end

DD  = load(data_driven_file);
DAD = load(data_assisted_diss_file);
DDD = load(data_driven_diss_file);

disp('Loaded all three result files successfully.');

%% =========================================================
% EXTRACT STATE AND CONTROL HISTORIES
% =========================================================

x_dd  = DD.x_test;
x_dad = DAD.x_test;
x_ddd = DDD.x_test;

u_dd  = DD.u_test;
u_dad = DAD.u_test;
u_ddd = DDD.u_test;

%% Optional raw command histories
if isfield(DD, 'u_cmd_test')
    u_cmd_dd = DD.u_cmd_test;
else
    u_cmd_dd = u_dd;
end

if isfield(DAD, 'u_cmd_test')
    u_cmd_dad = DAD.u_cmd_test;
else
    u_cmd_dad = u_dad;
end

if isfield(DDD, 'u_cmd_test')
    u_cmd_ddd = DDD.u_cmd_test;
else
    u_cmd_ddd = u_ddd;
end

%% =========================================================
% SAMPLING TIME
% =========================================================

if isfield(DD, 'Ts')
    Ts = DD.Ts;
elseif isfield(DAD, 'Ts')
    Ts = DAD.Ts;
elseif isfield(DDD, 'Ts')
    Ts = DDD.Ts;
else
    error('Ts was not found in any result file.');
end

%% =========================================================
% MATCH LENGTHS
% =========================================================

N_dd  = size(x_dd, 2) - 1;
N_dad = size(x_dad, 2) - 1;
N_ddd = size(x_ddd, 2) - 1;

Ntest = min([N_dd, N_dad, N_ddd]);

x_dd  = x_dd(:, 1:Ntest+1);
x_dad = x_dad(:, 1:Ntest+1);
x_ddd = x_ddd(:, 1:Ntest+1);

u_dd  = u_dd(:, 1:Ntest);
u_dad = u_dad(:, 1:Ntest);
u_ddd = u_ddd(:, 1:Ntest);

u_cmd_dd  = u_cmd_dd(:, 1:Ntest);
u_cmd_dad = u_cmd_dad(:, 1:Ntest);
u_cmd_ddd = u_cmd_ddd(:, 1:Ntest);

t_state = 0:Ts:Ntest*Ts;
t_input = 0:Ts:(Ntest-1)*Ts;

%% =========================================================
% DISTURBANCE TIME
% =========================================================

if isfield(DD, 'disturbance_time')
    disturbance_time = DD.disturbance_time;
elseif isfield(DD, 'dist_time')
    disturbance_time = DD.dist_time;
elseif isfield(DAD, 'disturbance_time')
    disturbance_time = DAD.disturbance_time;
elseif isfield(DDD, 'disturbance_time')
    disturbance_time = DDD.disturbance_time;
else
    disturbance_time = 10;
end

%% Disturbance values if available
if isfield(DD, 'dist_angle_deg')
    dist_angle_deg = DD.dist_angle_deg;
elseif isfield(DAD, 'dist_angle_deg')
    dist_angle_deg = DAD.dist_angle_deg;
elseif isfield(DDD, 'dist_angle_deg')
    dist_angle_deg = DDD.dist_angle_deg;
else
    dist_angle_deg = NaN;
end

if isfield(DD, 'dist_angvel_deg')
    dist_angvel_deg = DD.dist_angvel_deg;
elseif isfield(DAD, 'dist_angvel_deg')
    dist_angvel_deg = DAD.dist_angvel_deg;
elseif isfield(DDD, 'dist_angvel_deg')
    dist_angvel_deg = DDD.dist_angvel_deg;
else
    dist_angvel_deg = NaN;
end

disp('====================================================');
disp('RESULT INFORMATION');
disp('====================================================');

fprintf('Ts                 = %.4f s\n', Ts);
fprintf('Ntest              = %d samples\n', Ntest);
fprintf('Total time         = %.2f s\n', Ntest*Ts);
fprintf('Disturbance time   = %.2f s\n', disturbance_time);

if ~isnan(dist_angle_deg)
    fprintf('Angle disturbance  = %.2f deg\n', dist_angle_deg);
end

if ~isnan(dist_angvel_deg)
    fprintf('Angular velocity disturbance = %.2f deg/s\n', dist_angvel_deg);
end

%% =========================================================
% CONTROLLER GAINS IF AVAILABLE
% =========================================================

disp(' ');
disp('====================================================');
disp('CONTROLLER GAINS');
disp('====================================================');

if isfield(DD, 'K_data')
    disp('Pure Data-Driven gain K_data:');
    disp(DD.K_data);
end

if isfield(DAD, 'K_diss')
    disp('Data-Assisted Dissipativity gain K_diss:');
    disp(DAD.K_diss);
end

if isfield(DDD, 'K_data')
    disp('Data-Driven Dissipativity gain K_data:');
    disp(DDD.K_data);
end

%% =========================================================
% CLOSED-LOOP EIGENVALUE SUMMARY
% =========================================================

disp(' ');
disp('====================================================');
disp('CLOSED-LOOP EIGENVALUE SUMMARY');
disp('====================================================');

if isfield(DD, 'cl_eigs')
    eig_dd = DD.cl_eigs;
    fprintf('Pure Data-Driven max |eig|              = %.6f\n', max(abs(eig_dd)));
    fprintf('Pure Data-Driven stable?                = %d\n', all(abs(eig_dd) < 1));
end

if isfield(DAD, 'cl_eigs')
    eig_dad = DAD.cl_eigs;
    fprintf('Data-Assisted Dissipativity max |eig|   = %.6f\n', max(abs(eig_dad)));
    fprintf('Data-Assisted Dissipativity stable?     = %d\n', all(abs(eig_dad) < 1));
end

if isfield(DDD, 'cl_eigs')
    eig_ddd = DDD.cl_eigs;
    fprintf('Data-Driven Dissipativity max |eig|     = %.6f\n', max(abs(eig_ddd)));
    fprintf('Data-Driven Dissipativity stable?       = %d\n', all(abs(eig_ddd) < 1));
end

%% =========================================================
% SUMMARY METRICS
% =========================================================

disp(' ');
disp('====================================================');
disp('THREE-WAY COMPARISON SUMMARY');
disp('====================================================');

print_metrics('Pure Data-Driven', x_dd, u_dd, u_cmd_dd, DD);
print_metrics('Data-Assisted Dissipativity', x_dad, u_dad, u_cmd_dad, DAD);
print_metrics('Data-Driven Dissipativity', x_ddd, u_ddd, u_cmd_ddd, DDD);

%% =========================================================
% CREATE OUTPUT FOLDER
% =========================================================

output_folder = 'comparison_graphs_three_data_controllers';

if exist(output_folder, 'dir') ~= 7
    mkdir(output_folder);
end

lw = 2;

%% =========================================================
% 1) BODY ANGLE COMPARISON
% =========================================================

figure('Name','Body Angle Comparison');
plot(t_state, rad2deg(x_dd(3,:)), 'LineWidth', lw);
hold on;
plot(t_state, rad2deg(x_dad(3,:)), '--', 'LineWidth', lw);
plot(t_state, rad2deg(x_ddd(3,:)), ':', 'LineWidth', lw + 0.5);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('\phi (deg)');
title('Body Angle Comparison');
legend('Pure Data-Driven', ...
       'Data-Assisted Dissipativity', ...
       'Data-Driven Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'body_angle_comparison.png'));

%% =========================================================
% 2) CONTROL INPUT COMPARISON
% =========================================================

figure('Name','Control Input Comparison');
plot(t_input, u_dd(1,:), 'LineWidth', lw);
hold on;
plot(t_input, u_dad(1,:), '--', 'LineWidth', lw);
plot(t_input, u_ddd(1,:), ':', 'LineWidth', lw + 0.5);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Control input u');
title('Control Input Comparison');
legend('Pure Data-Driven', ...
       'Data-Assisted Dissipativity', ...
       'Data-Driven Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'control_input_comparison.png'));

%% =========================================================
% 3) ROBOT POSITION COMPARISON
% =========================================================

figure('Name','Robot Position Comparison');
plot(t_state, x_dd(1,:), 'LineWidth', lw);
hold on;
plot(t_state, x_dad(1,:), '--', 'LineWidth', lw);
plot(t_state, x_ddd(1,:), ':', 'LineWidth', lw + 0.5);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Position x (m)');
title('Robot Position Comparison');
legend('Pure Data-Driven', ...
       'Data-Assisted Dissipativity', ...
       'Data-Driven Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'robot_position_comparison.png'));

%% =========================================================
% 4) ROBOT VELOCITY COMPARISON
% =========================================================

figure('Name','Robot Velocity Comparison');
plot(t_state, x_dd(2,:), 'LineWidth', lw);
hold on;
plot(t_state, x_dad(2,:), '--', 'LineWidth', lw);
plot(t_state, x_ddd(2,:), ':', 'LineWidth', lw + 0.5);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Velocity xdot (m/s)');
title('Robot Velocity Comparison');
legend('Pure Data-Driven', ...
       'Data-Assisted Dissipativity', ...
       'Data-Driven Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'robot_velocity_comparison.png'));

%% =========================================================
% 5) ANGULAR VELOCITY COMPARISON
% =========================================================

figure('Name','Angular Velocity Comparison');
plot(t_state, rad2deg(x_dd(4,:)), 'LineWidth', lw);
hold on;
plot(t_state, rad2deg(x_dad(4,:)), '--', 'LineWidth', lw);
plot(t_state, rad2deg(x_ddd(4,:)), ':', 'LineWidth', lw + 0.5);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('\phidot (deg/s)');
title('Angular Velocity Comparison');
legend('Pure Data-Driven', ...
       'Data-Assisted Dissipativity', ...
       'Data-Driven Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'angular_velocity_comparison.png'));

%% =========================================================
% 6) FULL COMPARISON FIGURE
% =========================================================

figure('Name','Full Three-Way Comparison', 'Position', [100 100 950 950]);

subplot(5,1,1);
plot(t_state, rad2deg(x_dd(3,:)), 'LineWidth', 1.6);
hold on;
plot(t_state, rad2deg(x_dad(3,:)), '--', 'LineWidth', 1.6);
plot(t_state, rad2deg(x_ddd(3,:)), ':', 'LineWidth', 2.0);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
grid on;
ylabel('\phi (deg)');
title('Three-Way Data Controller Comparison Under Disturbance');
legend('Pure Data-Driven', 'Data-Assisted Dissipativity', ...
       'Data-Driven Dissipativity', 'Location', 'best');

subplot(5,1,2);
plot(t_input, u_dd(1,:), 'LineWidth', 1.6);
hold on;
plot(t_input, u_dad(1,:), '--', 'LineWidth', 1.6);
plot(t_input, u_ddd(1,:), ':', 'LineWidth', 2.0);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
grid on;
ylabel('u');

subplot(5,1,3);
plot(t_state, x_dd(1,:), 'LineWidth', 1.6);
hold on;
plot(t_state, x_dad(1,:), '--', 'LineWidth', 1.6);
plot(t_state, x_ddd(1,:), ':', 'LineWidth', 2.0);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
grid on;
ylabel('x (m)');

subplot(5,1,4);
plot(t_state, x_dd(2,:), 'LineWidth', 1.6);
hold on;
plot(t_state, x_dad(2,:), '--', 'LineWidth', 1.6);
plot(t_state, x_ddd(2,:), ':', 'LineWidth', 2.0);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
grid on;
ylabel('xdot (m/s)');

subplot(5,1,5);
plot(t_state, rad2deg(x_dd(4,:)), 'LineWidth', 1.6);
hold on;
plot(t_state, rad2deg(x_dad(4,:)), '--', 'LineWidth', 1.6);
plot(t_state, rad2deg(x_ddd(4,:)), ':', 'LineWidth', 2.0);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('\phidot (deg/s)');

saveas(gcf, fullfile(output_folder, 'full_comparison.png'));

%% =========================================================
% SAVE COMPARISON DATA
% =========================================================

save(fullfile(output_folder, 'SBR_Comparison_Three_Data_Controllers.mat'), ...
    'x_dd', 'x_dad', 'x_ddd', ...
    'u_dd', 'u_dad', 'u_ddd', ...
    'u_cmd_dd', 'u_cmd_dad', 'u_cmd_ddd', ...
    'Ts', 'Ntest', ...
    't_state', 't_input', ...
    'disturbance_time');

disp(' ');
disp('====================================================');
disp('THREE-WAY COMPARISON GRAPHS GENERATED SUCCESSFULLY');
disp('====================================================');
fprintf('Saved graphs in folder: %s\n', output_folder);
disp('Saved comparison data as SBR_Comparison_Three_Data_Controllers.mat');

%% =========================================================
% LOCAL FUNCTION: PRINT METRICS
% =========================================================

function print_metrics(name, x, u, u_cmd, result_struct)

    fprintf('\n--- %s ---\n', name);

    fprintf('Initial body angle      = %.4f deg\n', rad2deg(x(3,1)));
    fprintf('Final body angle        = %.6f deg\n', rad2deg(x(3,end)));
    fprintf('Final position          = %.6f m\n', x(1,end));
    fprintf('Final velocity          = %.6f m/s\n', x(2,end));
    fprintf('Final angular velocity  = %.6f deg/s\n', rad2deg(x(4,end)));

    fprintf('Max |body angle|        = %.6f deg\n', max(abs(rad2deg(x(3,:)))));
    fprintf('Max |position|          = %.6f m\n', max(abs(x(1,:))));
    fprintf('Max |velocity|          = %.6f m/s\n', max(abs(x(2,:))));
    fprintf('Max |angular velocity|  = %.6f deg/s\n', max(abs(rad2deg(x(4,:)))));

    fprintf('Max |control input|     = %.6f\n', max(abs(u(:))));
    fprintf('Max |raw command|       = %.6f\n', max(abs(u_cmd(:))));

    if isfield(result_struct, 'recovery_time')
        fprintf('Saved recovery time     = %.6f s\n', result_struct.recovery_time);
    end

    if isfield(result_struct, 'gamma_val')
        fprintf('Gamma value             = %.6f\n', result_struct.gamma_val);
    end

    if isfield(result_struct, 'rank_data') && isfield(result_struct, 'required_rank')
        fprintf('Data rank               = %d / %d\n', ...
            result_struct.rank_data, result_struct.required_rank);
    end

end