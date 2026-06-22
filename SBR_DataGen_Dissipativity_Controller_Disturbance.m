clc;
clear;
close all;

%% =========================================
% SELF-BALANCING ROBOT
% MODEL-BASED DISSIPATIVITY CONTROLLER
% + DATA GENERATION FOR DATA-DRIVEN / DATA-ASSISTED CONTROLLERS
% + DISTURBANCE VALIDATION (-20 deg, -80 deg/s at 10 s)
% =========================================
%
% CATEGORY:
%   Model-Based Controller with Dissipativity
%
% IMPORTANT:
%   This script GENERATES and SAVES data matrices for later scripts.
%   This script does NOT use those data matrices for its own controller design.
%   This script does NOT recover A_est or B_est for controller design.
%   The dissipativity controller is designed directly from Ad and Bd.
%
% REQUIREMENTS:
%   - YALMIP installed
%   - SDP solver installed (SDPT3 / SeDuMi / MOSEK)
%
% Closed-loop convention:
%   u(k) = K_diss * x(k)
%   x(k+1) = Ad*x(k) + Bd*u(k)
%          = (Ad + Bd*K_diss)*x(k)

%% =========================================
% PHYSICAL PARAMETERS
% =========================================
M = 0.5;         % chassis mass
m_body = 0.2;    % body mass
l = 0.3;         % COM distance
I = 0.006;       % body inertia
b = 0.1;         % viscous friction 0.8
g = 9.81;        % gravity

%% =========================================
% MODEL CONSTANTS
% =========================================
a = M + m_body;
c = m_body * l;
d = I + m_body * l^2;
Delta = a*d + c^2;

%% =========================================
% CONTINUOUS-TIME MODEL
% x = [x; xdot; phi; phidot]
% =========================================
A = [0, 1, 0, 0;
     0, -(d*b)/Delta, -(c*m_body*g*l)/Delta, 0;
     0, 0, 0, 1;
     0, -(c*b)/Delta, (a*m_body*g*l)/Delta, 0];

B = [0;
     d/Delta;
     0;
     c/Delta];

C = eye(4);
D = zeros(4,1);

disp('================ CONTINUOUS MODEL ================');
disp('A matrix:');
disp(A);
disp('B matrix:');
disp(B);

%% =========================================
% DISCRETIZATION
% =========================================
Ts = 0.01;

n = size(A,1);
m = size(B,2);
p = size(C,1);

Maug = [A B;
        zeros(m, n+m)];

Md = expm(Maug * Ts);

Ad = Md(1:n,1:n);
Bd = Md(1:n,n+1:n+m);

Cd = C;
Dd = D;

disp('================ DISCRETE MODEL ==================');
disp('Ad matrix:');
disp(Ad);
disp('Bd matrix:');
disp(Bd);

%% =========================================
% DATA GENERATION FOR DATA-DRIVEN / DATA-ASSISTED CONTROLLERS
% =========================================
% Purpose:
%   Generate rich input-state data matrices:
%       X0 = [x(0) ... x(T-1)]
%       X1 = [x(1) ... x(T)]
%       U0 = [u(0) ... u(T-1)]
%       Y0 = [y(0) ... y(T-1)]
%       Y1 = [y(1) ... y(T)]
%
% Important:
%   These matrices are saved for other scripts.
%   They are NOT used below to design K_diss.

Tdata = 1000;

x0_data = [0;
           0;
           deg2rad(45);
           0];

% Temporary stabilizing feedback only for safe data collection
% This is NOT the final dissipativity controller.
% Convention here is u = Ktemp*x + excitation.
Ktemp = [1.0  2.0  -18.0  -3.5];
Acl_temp = Ad + Bd*Ktemp;

disp('========= DATA COLLECTION CLOSED-LOOP EIGS ========');
disp(eig(Acl_temp));

% Persistently exciting input
u_amp = 0.15;
hold_steps = 8;
num_blocks = ceil(Tdata/hold_steps);
u_blocks = u_amp * sign(randn(1,num_blocks));
u_exc = repelem(u_blocks, hold_steps);
u_exc = u_exc(1:Tdata);

% Storage
x_data = zeros(n, Tdata+1);
y_data = zeros(p, Tdata);
u_data = zeros(m, Tdata);

x_data(:,1) = x0_data;

% Simulate data collection using the known model Ad, Bd
for k = 1:Tdata
    u_data(:,k) = Ktemp * x_data(:,k) + u_exc(k);
    y_data(:,k) = Cd * x_data(:,k) + Dd * u_data(:,k);
    x_data(:,k+1) = Ad * x_data(:,k) + Bd * u_data(:,k);
end

% Build data matrices
X0 = x_data(:,1:Tdata);
X1 = x_data(:,2:Tdata+1);
U0 = u_data(:,1:Tdata);
Y0 = y_data(:,1:Tdata);

Y1 = zeros(p,Tdata);
Y1(:,1:Tdata-1) = y_data(:,2:Tdata);
Y1(:,Tdata) = Cd*x_data(:,Tdata+1) + Dd*u_data(:,Tdata);

disp('================ DATA MATRIX SIZES ===============');
fprintf('size(X0) = [%d %d]\n', size(X0,1), size(X0,2));
fprintf('size(X1) = [%d %d]\n', size(X1,1), size(X1,2));
fprintf('size(U0) = [%d %d]\n', size(U0,1), size(U0,2));
fprintf('size(Y0) = [%d %d]\n', size(Y0,1), size(Y0,2));
fprintf('size(Y1) = [%d %d]\n', size(Y1,1), size(Y1,2));

rank_data = rank([U0; X0]);
required_rank = n + m;

disp('================ DATA RANK CHECK =================');
fprintf('rank([U0; X0]) = %d\n', rank_data);
fprintf('required rank  = %d\n', required_rank);

if rank_data < required_rank
    warning('Generated data is NOT rich enough. Increase Tdata/u_amp or change excitation.');
else
    disp('Generated data is rich enough for data-driven / data-assisted control.');
end

% Save data for both future approaches:
%   Data-driven: uses X0, X1, U0 directly.
%   Data-assisted: can recover A_est, B_est from X1 = [B A]*[U0; X0].
save('SBR_Dissipative_Data_Matrices.mat', ...
    'X0','X1','U0','Y0','Y1', ...
    'x_data','y_data','u_data','u_exc','Ktemp','Tdata','Ts', ...
    'Ad','Bd','Cd','Dd','A','B','C','D', ...
    'M','m_body','l','I','b','g','a','c','d','Delta');

disp('Saved generated data to SBR_Data_Matrices.mat');

%% =========================================
% MODEL-BASED DISSIPATIVITY CONTROLLER DESIGN
% =========================================
disp('====== MODEL-BASED DISSIPATIVITY CONTROLLER ======');

if ~exist('sdpvar','file')
    error('YALMIP not found. Install YALMIP and SDPT3/SeDuMi/MOSEK.');
end

% Disturbance matrix
Ew = 0.05 * eye(n);
nw = size(Ew,2);

% Performance output: focus mainly on body angle
Cz = diag([0.2 0.05 1 0.2]);
nz = size(Cz,1);

% YALMIP variables
X = sdpvar(n,n,'symmetric');
L = sdpvar(m,n,'full');
gamma = sdpvar(1,1);

% Closed-loop term for u = Kx
% MODEL-BASED: uses Ad and Bd directly
AclX = Ad*X + Bd*L;

% Discrete-time bounded-real / dissipativity-style LMI
BRL = [ X,            AclX,         Ew,               zeros(n,nz);
        AclX',        X,            zeros(nw,n),      X*Cz';
        Ew',          zeros(nw,n),  gamma*eye(nw),    zeros(nw,nz);
        zeros(nz,n),  Cz*X,         zeros(nz,nw),     gamma*eye(nz) ];

constraints = [];
constraints = [constraints, X >= 1e-5*eye(n)];
constraints = [constraints, gamma >= 1e-4];
constraints = [constraints, BRL >= 1e-5*eye(size(BRL,1))];

% Simple linear bounds on L instead of norm(L,'fro')
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

disp('Model-based dissipativity gain K_diss:');
disp(K_diss);
fprintf('Optimal gamma = %.6f\n', gamma_val);

%% =========================================
% CLOSED-LOOP STABILITY CHECK
% =========================================
Acl = Ad + Bd*K_diss;

disp('============ CLOSED-LOOP STABILITY CHECK =========');
disp('Acl = Ad + Bd*K_diss:');
disp(Acl);

cl_eigs = eig(Acl);

disp('Closed-loop eigenvalues:');
disp(cl_eigs);

if all(abs(cl_eigs) < 1)
    disp('Closed-loop discrete-time model-based system is stable.');
else
    disp('Closed-loop discrete-time model-based system is NOT stable.');
end

%% =========================================
% ROBUSTNESS TEST (±10% variation in b and I)
% =========================================
disp('================ ROBUSTNESS TEST =================');

b_values = [0.9*b, 1.1*b];
I_values = [0.9*I, 1.1*I];

u_sat_limit = 20;   % actuator saturation for realistic testing

for i = 1:length(b_values)
    for j = 1:length(I_values)

        b_test = b_values(i);
        I_test = I_values(j);

        a_test = M + m_body;
        c_test = m_body * l;
        d_test = I_test + m_body * l^2;
        Delta_test = a_test*d_test + c_test^2;

        A_test = [0, 1, 0, 0;
                  0, -(d_test*b_test)/Delta_test, -(c_test*m_body*g*l)/Delta_test, 0;
                  0, 0, 0, 1;
                  0, -(c_test*b_test)/Delta_test, (a_test*m_body*g*l)/Delta_test, 0];

        B_test = [0;
                  d_test/Delta_test;
                  0;
                  c_test/Delta_test];

        Maug_test = [A_test B_test;
                     zeros(m, n+m)];

        Md_test = expm(Maug_test * Ts);

        Ad_test = Md_test(1:n,1:n);
        Bd_test = Md_test(1:n,n+1:n+m);

        Acl_test = Ad_test + Bd_test*K_diss;
        eig_test = eig(Acl_test);

        fprintf('\n--- Test Case ---\n');
        fprintf('b = %.4f, I = %.6f\n', b_test, I_test);
        fprintf('Linear closed-loop stable? %d\n', all(abs(eig_test) < 1));
        fprintf('Max eigenvalue magnitude = %.4f\n', max(abs(eig_test)));

        x_sim = zeros(n, 1001);
        x_sim(:,1) = [0;0;deg2rad(45);0];

        u_sim_hist = zeros(m,1000);

        for k = 1:300
            u_cmd = K_diss * x_sim(:,k);
            u_sat = max(min(u_cmd, u_sat_limit), -u_sat_limit);
            u_sim_hist(:,k) = u_sat;
            x_sim(:,k+1) = Ad_test * x_sim(:,k) + Bd_test * u_sat;
        end

        fprintf('Final angle (deg) = %.4f\n', rad2deg(x_sim(3,end)));
        fprintf('Max |u| with saturation = %.4f\n', max(abs(u_sim_hist(:))));
    end
end

%% =========================================
% DISTURBANCE RESPONSE CHECK
% =========================================
% Disturbance is only injected during validation.

Ntest = 2000;              % 20 seconds, so recovery after 10 s is visible
u_sat_limit = 20;

disturbance_time = 10;     % seconds
dist_step = round(disturbance_time/Ts) + 1;
dist_angle_deg = -20;      % body angle disturbance
dist_angvel_deg = -80;     % angular velocity disturbance
angle_threshold_deg = 1;   % recovery threshold for body angle

x_test = zeros(n, Ntest+1);
u_test = zeros(m, Ntest);
u_cmd_test = zeros(m, Ntest);

x_test(:,1) = [0;
               0;
               deg2rad(45);
               0];

for k = 1:Ntest
    u_cmd = K_diss * x_test(:,k);
    u_sat = max(min(u_cmd, u_sat_limit), -u_sat_limit);

    u_cmd_test(:,k) = u_cmd;
    u_test(:,k) = u_sat;

    % MODEL-BASED simulation uses Ad and Bd directly
    x_test(:,k+1) = Ad * x_test(:,k) + Bd * u_sat;

    % Apply impulse-like state disturbance at 10 seconds
    if k == dist_step
        x_test(3,k+1) = x_test(3,k+1) + deg2rad(dist_angle_deg);
        x_test(4,k+1) = x_test(4,k+1) + deg2rad(dist_angvel_deg);

        fprintf('\n*** Disturbance injected at t = %.2f s ***\n', disturbance_time);
        fprintf('Added angle disturbance = %.2f deg\n', dist_angle_deg);
        fprintf('Added angular velocity disturbance = %.2f deg/s\n', dist_angvel_deg);
    end
end

% Recovery time calculation: first time after disturbance when |phi| <= threshold
angle_deg = rad2deg(x_test(3,:));
post_dist_idx = dist_step:length(angle_deg);
recovery_idx_rel = find(abs(angle_deg(post_dist_idx)) <= angle_threshold_deg, 1, 'first');

if isempty(recovery_idx_rel)
    recovery_time = NaN;
    disp('Recovery threshold was not reached within the simulation window.');
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

figure('Name','Model-Based Dissipative Robot Position');
plot(t, x_test(1,:), 'LineWidth', 2);
xline(disturbance_time, '--r', 'Disturbance');
grid on;
xlabel('Time (s)');
ylabel('x (m)');
title('Model-Based Dissipativity Controller: Robot Position');

figure('Name','Model-Based Dissipative Linear Velocity');
plot(t, x_test(2,:), 'LineWidth', 2);
xline(disturbance_time, '--r', 'Disturbance');
grid on;
xlabel('Time (s)');
ylabel('xdot (m/s)');
title('Model-Based Dissipativity Controller: Linear Velocity');

figure('Name','Model-Based Dissipative Body Angle');
plot(t, rad2deg(x_test(3,:)), 'LineWidth', 2);
hold on;
xline(disturbance_time, '--r', 'Disturbance');
yline(angle_threshold_deg, '--k', '+1 deg');
yline(-angle_threshold_deg, '--k', '-1 deg');
grid on;
xlabel('Time (s)');
ylabel('\phi (deg)');
title('Model-Based Dissipativity Controller: Body Angle with Disturbance');

figure('Name','Model-Based Dissipative Angular Velocity');
plot(t, rad2deg(x_test(4,:)), 'LineWidth', 2);
xline(disturbance_time, '--r', 'Disturbance');
grid on;
xlabel('Time (s)');
ylabel('\phidot (deg/s)');
title('Model-Based Dissipativity Controller: Angular Velocity');

figure('Name','Model-Based Dissipative Control Input');
plot(t_input, u_test, 'LineWidth', 2);
xline(disturbance_time, '--r', 'Disturbance');
grid on;
xlabel('Time (s)');
ylabel('Control input u');
title('Model-Based Dissipativity Controller: Control Input');

%% =========================================
% SAVE EVERYTHING
% =========================================
save('SBR_ModelBased_Dissipativity_Controller_Disturbance_Result.mat', ...
    'A','B','C','D', ...
    'Ad','Bd','Cd','Dd', ...
    'X0','X1','U0','Y0','Y1','x_data','y_data','u_data','u_exc','Ktemp','Tdata','rank_data','required_rank', ...
    'K_diss', ...
    'Acl','cl_eigs', ...
    'x_test','u_test','u_cmd_test', ...
    'disturbance_time','dist_step','dist_angle_deg','dist_angvel_deg', ...
    'angle_threshold_deg','recovery_time', ...
    'Ts','M','m_body','l','I','b','g','a','c','d','Delta', ...
    'Ew','Cz','gamma_val','u_sat_limit');

disp('================ FINISHED ========================');
disp('Saved results to SBR_ModelBased_Dissipativity_Controller_Disturbance_Result.mat');
disp('Plot export and animation/video generation are disabled.');

%% =========================================
% SHOW + SAVE DATA-DRIVEN ROBOT ANIMATION
% =========================================
disp('========== GENERATING AND SAVING ANIMATION ==========');

opengl software

video_name = 'model_based_dissipativity_robot_animation.mp4';

v = VideoWriter(video_name,'MPEG-4');
v.FrameRate = 10;
open(v);

fig = figure('Name','Model-Based Dissipativity Robot Animation', 'Visible','on');

wheel_radius = 0.05;
body_length = 0.5;

frame_count = 0;

for k = 1:10:length(x_test)

    clf(fig);
    hold on;
    grid on;
    axis equal;

    xlim([-1 1]);
    ylim([-0.1 1]);

    x_pos = x_test(1,k);
    phi   = x_test(3,k);

    wheel_x = x_pos;
    wheel_y = wheel_radius;

    body_x = wheel_x + body_length*sin(phi);
    body_y = wheel_y + body_length*cos(phi);

    % Ground
    plot([-5 5],[0 0],'k','LineWidth',2);

    % Wheel
    theta = linspace(0,2*pi,60);
    plot(wheel_x + wheel_radius*cos(theta), ...
         wheel_y + wheel_radius*sin(theta), ...
         'b','LineWidth',2);

    % Body
    plot([wheel_x body_x], [wheel_y body_y], ...
         'r','LineWidth',4);

    % COM
    plot(body_x, body_y, 'ko','MarkerFaceColor','k');

    current_time = (k-1)*Ts;

    if abs(current_time - disturbance_time) < 0.05
        title_text = sprintf('DISTURBANCE APPLIED | Time = %.2f s | Angle = %.2f deg', ...
                             current_time, rad2deg(phi));
    else
        title_text = sprintf('Time = %.2f s | Angle = %.2f deg', ...
                             current_time, rad2deg(phi));
    end

    title(title_text);

    xlabel('Position x (m)');
    ylabel('Height (m)');

    drawnow;

    frame = getframe(fig);

    if ~isempty(frame.cdata)
        writeVideo(v, frame);
        frame_count = frame_count + 1;
    end
end

close(v);

fprintf('Frames written: %d\n', frame_count);

if frame_count > 0
    disp('Animation saved successfully:');
    disp(fullfile(pwd, video_name));
else
    warning('No frames captured.');
end