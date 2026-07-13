clc;
clear;
close all;

%% =========================================================
% GENERATE COMPARISON GRAPHS FROM SAVED RESULT FILES
%
% Controller 1:
%   Model-Based / Proposition 1
%   SBR_Data_Gen_Controller_Disturbance_Result.mat
%
% Controller 2:
%   Model-Based Dissipativity
%   SBR_ModelBased_Dissipativity_Controller_Disturbance_Result.mat
%
% This script only loads saved x_test and u_test results.
% It does not redesign controllers or rerun simulations.
%
% State:
%   x = [position; velocity; body angle; angular velocity]
%% =========================================================

disp('====================================================');
disp('LOADING SAVED DISTURBANCE RESULTS');
disp('====================================================');

%% =========================================================
% RESULT FILES
% =========================================================

model_based_file = 'C:\Users\shris\Desktop\CPE 800 Project\Proposition 1\SBR_Data_Gen_Controller_Disturbance_Result.mat';
dissipativity_file = 'C:\Users\shris\Desktop\CPE 800 Project\Proposition 3\SBR_ModelBased_Dissipativity_Controller_Disturbance_Result.mat';

if exist(model_based_file, 'file') ~= 2
    error('Could not find %s in the current folder.', model_based_file);
end

if exist(dissipativity_file, 'file') ~= 2
    error('Could not find %s in the current folder.', dissipativity_file);
end

MB = load(model_based_file);
DIS = load(dissipativity_file);

disp('Loaded files successfully.');

%% =========================================================
% EXTRACT SAVED RESULTS
% =========================================================

x_mb = MB.x_test;
u_mb = MB.u_test;

x_dis = DIS.x_test;
u_dis = DIS.u_test;

%% Sampling time
if isfield(MB, 'Ts')
    Ts = MB.Ts;
elseif isfield(DIS, 'Ts')
    Ts = DIS.Ts;
else
    error('Ts was not found in either result file.');
end

%% Match length
N_mb = size(x_mb, 2) - 1;
N_dis = size(x_dis, 2) - 1;

Ntest = min(N_mb, N_dis);

x_mb = x_mb(:, 1:Ntest+1);
x_dis = x_dis(:, 1:Ntest+1);

u_mb = u_mb(:, 1:Ntest);
u_dis = u_dis(:, 1:Ntest);

t_state = 0:Ts:Ntest*Ts;
t_input = 0:Ts:(Ntest-1)*Ts;

%% Disturbance time
if isfield(MB, 'disturbance_time')
    disturbance_time = MB.disturbance_time;
elseif isfield(DIS, 'disturbance_time')
    disturbance_time = DIS.disturbance_time;
else
    disturbance_time = 10;
end

disp('====================================================');
disp('RESULT INFORMATION');
disp('====================================================');

fprintf('Ts                 = %.4f s\n', Ts);
fprintf('Ntest              = %d samples\n', Ntest);
fprintf('Total time         = %.2f s\n', Ntest*Ts);
fprintf('Disturbance time   = %.2f s\n', disturbance_time);

%% =========================================================
% SUMMARY METRICS
% =========================================================

disp(' ');
disp('====================================================');
disp('COMPARISON SUMMARY');
disp('====================================================');

fprintf('\n--- Model-Based / Proposition 1 ---\n');
fprintf('Initial body angle      = %.4f deg\n', rad2deg(x_mb(3,1)));
fprintf('Final body angle        = %.6f deg\n', rad2deg(x_mb(3,end)));
fprintf('Final position          = %.6f m\n', x_mb(1,end));
fprintf('Final velocity          = %.6f m/s\n', x_mb(2,end));
fprintf('Final angular velocity  = %.6f deg/s\n', rad2deg(x_mb(4,end)));
fprintf('Max |body angle|        = %.6f deg\n', max(abs(rad2deg(x_mb(3,:)))));
fprintf('Max |position|          = %.6f m\n', max(abs(x_mb(1,:))));
fprintf('Max |velocity|          = %.6f m/s\n', max(abs(x_mb(2,:))));
fprintf('Max |angular velocity|  = %.6f deg/s\n', max(abs(rad2deg(x_mb(4,:)))));
fprintf('Max |control input|     = %.6f\n', max(abs(u_mb(:))));

fprintf('\n--- Model-Based Dissipativity ---\n');
fprintf('Initial body angle      = %.4f deg\n', rad2deg(x_dis(3,1)));
fprintf('Final body angle        = %.6f deg\n', rad2deg(x_dis(3,end)));
fprintf('Final position          = %.6f m\n', x_dis(1,end));
fprintf('Final velocity          = %.6f m/s\n', x_dis(2,end));
fprintf('Final angular velocity  = %.6f deg/s\n', rad2deg(x_dis(4,end)));
fprintf('Max |body angle|        = %.6f deg\n', max(abs(rad2deg(x_dis(3,:)))));
fprintf('Max |position|          = %.6f m\n', max(abs(x_dis(1,:))));
fprintf('Max |velocity|          = %.6f m/s\n', max(abs(x_dis(2,:))));
fprintf('Max |angular velocity|  = %.6f deg/s\n', max(abs(rad2deg(x_dis(4,:)))));
fprintf('Max |control input|     = %.6f\n', max(abs(u_dis(:))));

%% =========================================================
% CREATE OUTPUT FOLDER
% =========================================================

output_folder = 'comparison_graphs_modelbased_vs_dissipativity';

if exist(output_folder, 'dir') ~= 7
    mkdir(output_folder);
end

lw = 2;

%% =========================================================
% BODY ANGLE COMPARISON
% =========================================================

figure('Name','Body Angle Comparison');
plot(t_state, rad2deg(x_mb(3,:)), 'LineWidth', lw);
hold on;
plot(t_state, rad2deg(x_dis(3,:)), '--', 'LineWidth', lw);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('\phi (deg)');
title('Body Angle Comparison');
legend('Model-Based - Proposition 1', ...
       'Model-Based Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'body_angle_comparison.png'));

%% =========================================================
% CONTROL INPUT COMPARISON
% =========================================================

figure('Name','Control Input Comparison');
plot(t_input, u_mb(1,:), 'LineWidth', lw);
hold on;
plot(t_input, u_dis(1,:), '--', 'LineWidth', lw);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Control input u');
title('Control Input Comparison');
legend('Model-Based - Proposition 1', ...
       'Model-Based Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'control_input_comparison.png'));

%% =========================================================
% ROBOT POSITION COMPARISON
% =========================================================

figure('Name','Robot Position Comparison');
plot(t_state, x_mb(1,:), 'LineWidth', lw);
hold on;
plot(t_state, x_dis(1,:), '--', 'LineWidth', lw);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Position x (m)');
title('Robot Position Comparison');
legend('Model-Based - Proposition 1', ...
       'Model-Based Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'robot_position_comparison.png'));

%% =========================================================
% ROBOT VELOCITY COMPARISON
% =========================================================

figure('Name','Robot Velocity Comparison');
plot(t_state, x_mb(2,:), 'LineWidth', lw);
hold on;
plot(t_state, x_dis(2,:), '--', 'LineWidth', lw);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Velocity xdot (m/s)');
title('Robot Velocity Comparison');
legend('Model-Based - Proposition 1', ...
       'Model-Based Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'robot_velocity_comparison.png'));

%% =========================================================
% ANGULAR VELOCITY COMPARISON
% =========================================================

figure('Name','Angular Velocity Comparison');
plot(t_state, rad2deg(x_mb(4,:)), 'LineWidth', lw);
hold on;
plot(t_state, rad2deg(x_dis(4,:)), '--', 'LineWidth', lw);
xline(disturbance_time, 'k--', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('\phidot (deg/s)');
title('Angular Velocity Comparison');
legend('Model-Based - Proposition 1', ...
       'Model-Based Dissipativity', ...
       'Location', 'best');

saveas(gcf, fullfile(output_folder, 'angular_velocity_comparison.png'));

%% =========================================================
% FULL COMPARISON FIGURE
% =========================================================

figure('Name','Full Comparison', 'Position', [100 100 900 900]);

subplot(5,1,1);
plot(t_state, rad2deg(x_mb(3,:)), 'LineWidth', 1.6);
hold on;
plot(t_state, rad2deg(x_dis(3,:)), '--', 'LineWidth', 1.6);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
grid on;
ylabel('\phi (deg)');
title('Model-Based vs Model-Based Dissipativity Controller Comparison');
legend('Model-Based', 'Dissipativity', 'Location', 'best');

subplot(5,1,2);
plot(t_input, u_mb(1,:), 'LineWidth', 1.6);
hold on;
plot(t_input, u_dis(1,:), '--', 'LineWidth', 1.6);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
grid on;
ylabel('u');

subplot(5,1,3);
plot(t_state, x_mb(1,:), 'LineWidth', 1.6);
hold on;
plot(t_state, x_dis(1,:), '--', 'LineWidth', 1.6);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
grid on;
ylabel('x (m)');

subplot(5,1,4);
plot(t_state, x_mb(2,:), 'LineWidth', 1.6);
hold on;
plot(t_state, x_dis(2,:), '--', 'LineWidth', 1.6);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
grid on;
ylabel('xdot (m/s)');

subplot(5,1,5);
plot(t_state, rad2deg(x_mb(4,:)), 'LineWidth', 1.6);
hold on;
plot(t_state, rad2deg(x_dis(4,:)), '--', 'LineWidth', 1.6);
xline(disturbance_time, 'k--', 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('\phidot (deg/s)');

saveas(gcf, fullfile(output_folder, 'full_comparison.png'));

%% =========================================================
% SAVE COMPARISON DATA
% =========================================================

save(fullfile(output_folder, 'SBR_Comparison_ModelBased_vs_Dissipativity.mat'), ...
    'x_mb', 'x_dis', ...
    'u_mb', 'u_dis', ...
    'Ts', 'Ntest', ...
    't_state', 't_input', ...
    'disturbance_time');

disp(' ');
disp('====================================================');
disp('COMPARISON GRAPHS GENERATED SUCCESSFULLY');
disp('====================================================');
fprintf('Saved graphs in folder: %s\n', output_folder);
disp('Saved comparison data as SBR_Comparison_ModelBased_vs_Dissipativity.mat');