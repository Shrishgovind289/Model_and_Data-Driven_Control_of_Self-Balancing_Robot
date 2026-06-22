clc;
clear;
close all;

%% =========================================
% SELF-BALANCING ROBOT
% DATA-ASSISTED DISSIPATIVITY CONTROLLER
% USING INPUT-OUTPUT DATA MATRICES
% =========================================
%
% CATEGORY:
%   Data-Assisted Dissipativity Controller
%
% Meaning:
%   1) Load data matrices: X0, X1, U0, Y0, Y1
%   2) Recover A_est, B_est from data
%   3) Use recovered A_est, B_est to design dissipativity controller
%
% IMPORTANT:
%   This is NOT pure data-driven dissipativity.
%   This is DATA-ASSISTED because it first estimates/reconstructs
%   a model from data, then designs the dissipativity controller.
%
% Convention:
%   u(k) = K_diss*x(k)
%   x(k+1) = A_est*x(k) + B_est*u(k)
%          = (A_est + B_est*K_diss)*x(k)

%% =========================================
% LOAD INPUT-OUTPUT DATA MATRICES
% =========================================

load('C:\Users\shris\Desktop\CPE 800 Project\Proposition 3\SBR_Dissipative_Data_Matrices.mat');

disp('================ LOADED DISSIPATIVE DATA MATRICES ================');

fprintf('size(X0) = [%d %d]\n', size(X0,1), size(X0,2));
fprintf('size(X1) = [%d %d]\n', size(X1,1), size(X1,2));
fprintf('size(U0) = [%d %d]\n', size(U0,1), size(U0,2));

if exist('Y0','var')
    fprintf('size(Y0) = [%d %d]\n', size(Y0,1), size(Y0,2));
end

if exist('Y1','var')
    fprintf('size(Y1) = [%d %d]\n', size(Y1,1), size(Y1,2));
end

%% =========================================
% BASIC DIMENSIONS
% =========================================

n = size(X0,1);      % number of states
m = size(U0,1);      % number of inputs
Tdata = size(X0,2);  % number of samples

% Sampling time fallback
if ~exist('Ts','var')
    Ts = 0.01;
end

%% =========================================
% DATA RANK CHECK
% =========================================

rank_data = rank([U0; X0]);
required_rank = n + m;

disp('================ DATA RANK CHECK =================');
fprintf('rank([U0; X0]) = %d\n', rank_data);
fprintf('required rank  = %d\n', required_rank);

if rank_data < required_rank
    error('Data is NOT rich enough for data-assisted dissipativity control.');
else
    disp('Data is rich enough for data-assisted dissipativity control.');
end

%% =========================================
% DATA-ASSISTED MODEL RECOVERY
% =========================================
%
% Data equation:
%   X1 = A_est*X0 + B_est*U0
%
% Rearranged:
%   X1 = [B_est A_est] * [U0; X0]
%
% Therefore:
%   [B_est A_est] = X1 * pinv([U0; X0])

AB_est = X1 * pinv([U0; X0]);

B_est = AB_est(:,1:m);
A_est = AB_est(:,m+1:m+n);

disp('================ RECOVERED MODEL FROM DATA =================');

disp('Recovered A_est:');
disp(A_est);

disp('Recovered B_est:');
disp(B_est);

%% =========================================
% OPTIONAL MODEL COMPARISON IF TRUE MODEL EXISTS
% =========================================

if exist('Ad','var') && exist('Bd','var')
    fprintf('||A_est - Ad|| = %.6e\n', norm(A_est - Ad));
    fprintf('||B_est - Bd|| = %.6e\n', norm(B_est - Bd));
else
    disp('True Ad, Bd not found in data file. Skipping model comparison.');
end

%% =========================================
% DATA-ASSISTED DISSIPATIVITY CONTROLLER DESIGN
% =========================================

disp('====== DATA-ASSISTED DISSIPATIVITY CONTROLLER ======');

if ~exist('sdpvar','file')
    error('YALMIP not found. Install YALMIP and SDPT3/SeDuMi/MOSEK.');
end

% Disturbance input matrix
Ew = 0.05 * eye(n);
nw = size(Ew,2);

% Performance output weighting
% Main focus: body angle phi
Cz = diag([0.2 0.05 1 0.2]);
nz = size(Cz,1);

% YALMIP decision variables
X = sdpvar(n,n,'symmetric');
L = sdpvar(m,n,'full');
gamma = sdpvar(1,1);

% Closed-loop term for u = Kx
% DATA-ASSISTED: uses recovered A_est, B_est
AclX = A_est*X + B_est*L;

% Discrete-time bounded-real / dissipativity-style LMI
BRL = [ X,            AclX,         Ew,               zeros(n,nz);
        AclX',        X,            zeros(nw,n),      X*Cz';
        Ew',          zeros(nw,n),  gamma*eye(nw),    zeros(nw,nz);
        zeros(nz,n),  Cz*X,         zeros(nz,nw),     gamma*eye(nz) ];

constraints = [];
constraints = [constraints, X >= 1e-5*eye(n)];
constraints = [constraints, gamma >= 1e-4];
constraints = [constraints, BRL >= 1e-5*eye(size(BRL,1))];

% Keep controller size reasonable
Lmax = 5;
constraints = [constraints, -Lmax <= L <= Lmax];

objective = gamma;

ops = sdpsettings('solver','sdpt3','verbose',1);
sol = optimize(constraints, objective, ops);

if sol.problem ~= 0
    error('SDPT3 failed: %s', sol.info);
end

X_val = value(X);
L_val = value(L);
gamma_val = value(gamma);

K_diss = L_val / X_val;

disp('Data-assisted dissipativity gain K_diss:');
disp(K_diss);

fprintf('Optimal gamma = %.6f\n', gamma_val);

%% =========================================
% CLOSED-LOOP STABILITY CHECK
% =========================================

Acl = A_est + B_est*K_diss;

disp('============ CLOSED-LOOP STABILITY CHECK =========');
disp('Acl = A_est + B_est*K_diss:');
disp(Acl);

cl_eigs = eig(Acl);

disp('Closed-loop eigenvalues:');
disp(cl_eigs);

if all(abs(cl_eigs) < 1)
    disp('Closed-loop data-assisted dissipativity system is stable.');
else
    disp('Closed-loop data-assisted dissipativity system is NOT stable.');
end

%% =========================================
% DISTURBANCE RESPONSE TEST
% =========================================

Ntest = 2000;              % 20 seconds
u_sat_limit = 20;

disturbance_time = 10;     % seconds
dist_step = round(disturbance_time/Ts) + 1;

dist_angle_deg = -20;      % angle disturbance
dist_angvel_deg = -80;     % angular velocity disturbance

angle_threshold_deg = 1;   % recovery threshold

x_test = zeros(n, Ntest+1);
u_test = zeros(m, Ntest);
u_cmd_test = zeros(m, Ntest);

% Initial condition
x_test(:,1) = [0;
               0;
               deg2rad(45);
               0];

for k = 1:Ntest

    % u = Kx
    u_cmd = K_diss * x_test(:,k);

    % Saturation
    u_sat = max(min(u_cmd, u_sat_limit), -u_sat_limit);

    % Store
    u_cmd_test(:,k) = u_cmd;
    u_test(:,k) = u_sat;

    % Data-assisted simulation using recovered model
    x_test(:,k+1) = A_est*x_test(:,k) + B_est*u_sat;

    % Inject disturbance at 10 seconds
    if k == dist_step
        x_test(3,k+1) = x_test(3,k+1) + deg2rad(dist_angle_deg);
        x_test(4,k+1) = x_test(4,k+1) + deg2rad(dist_angvel_deg);

        fprintf('\n*** Disturbance injected at t = %.2f s ***\n', disturbance_time);
        fprintf('Added angle disturbance = %.2f deg\n', dist_angle_deg);
        fprintf('Added angular velocity disturbance = %.2f deg/s\n', dist_angvel_deg);
    end
end

%% =========================================
% RECOVERY TIME CALCULATION
% =========================================

angle_deg = rad2deg(x_test(3,:));

post_dist_idx = dist_step:length(angle_deg);
recovery_idx_rel = find(abs(angle_deg(post_dist_idx)) <= angle_threshold_deg, 1, 'first');

if isempty(recovery_idx_rel)
    recovery_time = NaN;
    disp('Recovery threshold was not reached within simulation window.');
else
    recovery_idx = post_dist_idx(recovery_idx_rel);
    recovery_time = (recovery_idx-1)*Ts - disturbance_time;
    fprintf('Recovery time after disturbance = %.4f s\n', recovery_time);
end

disp('================ DISTURBANCE TEST RESPONSE ===================');
fprintf('Initial angle (deg)                 = %.4f\n', rad2deg(x_test(3,1)));
fprintf('Disturbance angle added (deg)        = %.4f\n', dist_angle_deg);
fprintf('Disturbance angular velocity (deg/s) = %.4f\n', dist_angvel_deg);
fprintf('Final angle (deg)                   = %.6f\n', rad2deg(x_test(3,end)));
fprintf('Final state norm                    = %.6e\n', norm(x_test(:,end)));
fprintf('Max |u_cmd|                         = %.6f\n', max(abs(u_cmd_test(:))));
fprintf('Max |u_sat|                         = %.6f\n', max(abs(u_test(:))));

%% =========================================
% PLOTS
% =========================================

t = 0:Ts:Ntest*Ts;
t_input = 0:Ts:(Ntest-1)*Ts;

figure('Name','Data-Assisted Dissipative Cart Position');
plot(t, x_test(1,:), 'LineWidth', 2);
xline(disturbance_time, '--r', 'Disturbance');
grid on;
xlabel('Time (s)');
ylabel('x (m)');
title('Data-Assisted Dissipativity Controller: Cart Position');

figure('Name','Data-Assisted Dissipative Linear Velocity');
plot(t, x_test(2,:), 'LineWidth', 2);
xline(disturbance_time, '--r', 'Disturbance');
grid on;
xlabel('Time (s)');
ylabel('xdot (m/s)');
title('Data-Assisted Dissipativity Controller: Linear Velocity');

figure('Name','Data-Assisted Dissipative Body Angle');
plot(t, rad2deg(x_test(3,:)), 'LineWidth', 2);
hold on;
xline(disturbance_time, '--r', 'Disturbance');
yline(angle_threshold_deg, '--k', '+1 deg');
yline(-angle_threshold_deg, '--k', '-1 deg');
grid on;
xlabel('Time (s)');
ylabel('\phi (deg)');
title('Data-Assisted Dissipativity Controller: Body Angle with Disturbance');

figure('Name','Data-Assisted Dissipative Angular Velocity');
plot(t, rad2deg(x_test(4,:)), 'LineWidth', 2);
xline(disturbance_time, '--r', 'Disturbance');
grid on;
xlabel('Time (s)');
ylabel('\phidot (deg/s)');
title('Data-Assisted Dissipativity Controller: Angular Velocity');

figure('Name','Data-Assisted Dissipative Control Input');
plot(t_input, u_test, 'LineWidth', 2);
xline(disturbance_time, '--r', 'Disturbance');
grid on;
xlabel('Time (s)');
ylabel('Control input u');
title('Data-Assisted Dissipativity Controller: Control Input');

%% =========================================
% EXPORT ANIMATION VIDEO ONLY
% =========================================

disp('========== EXPORTING DATA-ASSISTED DISSIPATIVITY VIDEO ==========');

video_name = 'data_assisted_dissipativity_robot_animation.mp4';

v = VideoWriter(video_name, 'MPEG-4');
v.FrameRate = 20;
open(v);

fig_anim = figure('Name','Data-Assisted Dissipativity Robot Animation Video', ...
                  'NumberTitle','off', ...
                  'WindowStyle','normal');

% Fixed frame size
set(fig_anim, 'Position', [100 100 960 540]);

wheel_radius = 0.05;
body_length  = 0.5;

for k = 1:10:length(x_test)

    clf(fig_anim);
    hold on;
    grid on;
    axis equal;

    % Fixed view
    xlim([-1.5 1.5]);
    ylim([-0.1 1.0]);

    % States
    x_pos = x_test(1,k);
    phi   = x_test(3,k);

    % Geometry
    wheel_x = x_pos;
    wheel_y = wheel_radius;

    body_x = wheel_x + body_length*sin(phi);
    body_y = wheel_y + body_length*cos(phi);

    % Ground
    plot([-5 5], [0 0], 'k', 'LineWidth', 2);

    % Wheel
    theta = linspace(0, 2*pi, 80);
    plot(wheel_x + wheel_radius*cos(theta), ...
         wheel_y + wheel_radius*sin(theta), ...
         'b', 'LineWidth', 2);

    % Body
    plot([wheel_x body_x], [wheel_y body_y], ...
         'r', 'LineWidth', 4);

    % Center of mass
    plot(body_x, body_y, 'ko', 'MarkerFaceColor', 'k');

    % Disturbance marker
    if (k-1)*Ts >= disturbance_time
        text(-1.35, 0.9, 'Disturbance applied', ...
             'Color', 'r', 'FontWeight', 'bold');
    end

    title(sprintf('Data-Assisted Dissipativity Animation | Time = %.2f s | Angle = %.2f deg', ...
          (k-1)*Ts, rad2deg(phi)));

    xlabel('Position x (m)');
    ylabel('Height (m)');

    drawnow;

    frame = getframe(fig_anim);
    writeVideo(v, frame);
end

close(v);

disp('Video saved successfully:');
disp(fullfile(pwd, video_name));

%% =========================================
% PLAY SAVED VIDEO
% =========================================

disp('Playing saved video...');

video_path = fullfile(pwd, video_name);

if exist(video_path, 'file')
    try
        implay(video_path);
    catch
        open(video_path);
    end
else
    warning('Video file was not found.');
end

%% =========================================
% SAVE RESULTS
% =========================================

save('SBR_DataAssisted_Dissipativity_Controller_Result.mat', ...
    'X0','X1','U0','Y0','Y1', ...
    'A_est','B_est', ...
    'K_diss', ...
    'Acl','cl_eigs', ...
    'x_test','u_test','u_cmd_test', ...
    'disturbance_time','dist_step', ...
    'dist_angle_deg','dist_angvel_deg', ...
    'angle_threshold_deg','recovery_time', ...
    'Ts','Ew','Cz','gamma_val','u_sat_limit', ...
    'rank_data','required_rank','video_name');

disp('================ FINISHED ========================');
disp('Saved results to SBR_DataAssisted_Dissipativity_Controller_Result.mat');
disp('Data-assisted dissipativity controller completed.');