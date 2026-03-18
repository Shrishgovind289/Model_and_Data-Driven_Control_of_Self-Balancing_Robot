clc;
clear;
close all;

%% ============================================================
% SELF-BALANCING ROBOT:
% RANDOM b AND I, LINEAR VS NONLINEAR COMPARISON
%
% Control law:
%   u = Kx
%
% What this script does:
%   1. Randomly selects friction b and inertia I
%   2. Builds the linearized model
%   3. Designs K using an LMI
%   4. Simulates both linear and nonlinear systems
%   5. Overlays the responses
%% ============================================================

%% 1) Fixed physical parameters
M = 0.5;      % Base / wheel assembly mass (kg)
m = 0.2;      % Body mass (kg)
g = 9.81;     % Gravity (m/s^2)
l = 0.3;      % Distance from axle to COM (m)

%% 2) Randomly choose b and I
rng('shuffle');   % New random seed every run

b_min = 0.05;
b_max = 0.30;
I_min = 0.003;
I_max = 0.020;

b = b_min + (b_max - b_min)*rand;   % Random friction coefficient
I = I_min + (I_max - I_min)*rand;   % Random inertia

fprintf('Randomly selected parameters:\n');
fprintf('b = %.6f\n', b);
fprintf('I = %.6f\n', I);

%% 3) Linearized model
Delta = (M + m)*(I + m*l^2) - (m*l)^2;

A = [ 0, 1, 0, 0;
      0, -(I + m*l^2)*b/Delta,  (m^2*g*l^2)/Delta, 0;
      0, 0, 0, 1;
      0, -(m*l*b)/Delta,        (m*g*l*(M + m))/Delta, 0 ];

B = [ 0;
      (I + m*l^2)/Delta;
      0;
      (m*l)/Delta ];

disp('Open-loop poles of linearized system:');
disp(eig(A));

%% 4) LMI design for u = Kx
n = size(A,1);
m_in = size(B,2);

P = sdpvar(n,n,'symmetric');
Y = sdpvar(m_in,n,'full');

eps_val = 1e-6;

% For u = Kx:
% Closed-loop system is x_dot = (A + B*K)x
% Lyapunov LMI:
% A*P + P*A' + B*Y + Y'*B' < 0
LMI = A*P + P*A' + B*Y + Y'*B';

Constraints = [];
Constraints = [Constraints, P >= eps_val*eye(n)];
Constraints = [Constraints, LMI <= -eps_val*eye(n)];

ops = sdpsettings('solver','sdpt3','verbose',1);
sol = optimize(Constraints, [], ops);

if sol.problem ~= 0
    error('LMI problem not solved successfully.');
end

P_val = value(P);
Y_val = value(Y);

K = Y_val / P_val;

disp('LMI-based feedback gain K = ');
disp(K);

%% 5) Closed-loop poles of linearized system
Acl = A + B*K;

disp('Closed-loop poles of linearized system:');
disp(eig(Acl));

%% 6) Initial condition
x0 = [0;
      0;
      deg2rad(5);
      0];
% Initial condition:
%   position = 0
%   velocity = 0
%   angle = 5 deg
%   angular velocity = 0

%% 7) Simulation interval
tspan = [0 10];

%% 8) Linear simulation
[t_lin, x_lin] = ode45(@(t,x) Acl*x, tspan, x0);

%% 9) Nonlinear simulation
[t_nl, x_nl] = ode45(@(t,x) nonlinear_sbr(t, x, K, M, m, b, I, g, l), tspan, x0);

%% ===== Compute control input for plotting =====
u_lin = zeros(length(t_lin),1);
for k = 1:length(t_lin)
    u_lin(k) = K * x_lin(k,:)';
end

u_nl = zeros(length(t_nl),1);
for k = 1:length(t_nl)
    u_nl(k) = K * x_nl(k,:)';
end

%% ===== Figure 1: Cart Position =====
figure
plot(t_lin, x_lin(:,1), '--','LineWidth',2)
hold on
plot(t_nl, x_nl(:,1), 'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('x (m)')
title('Cart Position')
legend('Linearized','Nonlinear')

%% ===== Figure 2: Cart Velocity =====
figure
plot(t_lin, x_lin(:,2), '--','LineWidth',2)
hold on
plot(t_nl, x_nl(:,2), 'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('\dot{x} (m/s)')
title('Cart Velocity')
legend('Linearized','Nonlinear')

%% ===== Figure 3: Body Angle =====
figure
plot(t_lin, rad2deg(x_lin(:,3)), '--','LineWidth',2)
hold on
plot(t_nl, rad2deg(x_nl(:,3)), 'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('\phi (deg)')
title('Body Angle')
legend('Linearized','Nonlinear')

%% ===== Figure 4: Angular Velocity =====
figure
plot(t_lin, rad2deg(x_lin(:,4)), '--','LineWidth',2)
hold on
plot(t_nl, rad2deg(x_nl(:,4)), 'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('\dot{\phi} (deg/s)')
title('Angular Velocity')
legend('Linearized','Nonlinear')

%% ===== Figure 5: Control Input =====
figure
plot(t_lin, u_lin, '--','LineWidth',2)
hold on
plot(t_nl, u_nl, 'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('u')
title('Control Input')
legend('Linearized','Nonlinear')

%% 11) Overlay plots: angle, position, control input
figure;

subplot(3,1,1);
plot(t_lin, rad2deg(x_lin(:,3)), 'b', 'LineWidth', 2);
hold on;
plot(t_nl, rad2deg(x_nl(:,3)), 'r--', 'LineWidth', 2);
grid on;
ylabel('\phi (deg)');
title(sprintf('Tilt Angle Comparison  |  b = %.4f, I = %.5f', b, I));
legend('Linearized', 'Nonlinear');

subplot(3,1,2);
plot(t_lin, x_lin(:,1), 'b', 'LineWidth', 2);
hold on;
plot(t_nl, x_nl(:,1), 'r--', 'LineWidth', 2);
grid on;
ylabel('x (m)');
title('Position Comparison');
legend('Linearized', 'Nonlinear');

subplot(3,1,3);
plot(t_lin, u_lin, 'b', 'LineWidth', 2);
hold on;
plot(t_nl, u_nl, 'r--', 'LineWidth', 2);
grid on;
ylabel('u');
xlabel('Time (s)');
title('Control Input Comparison');
legend('Linearized', 'Nonlinear');

%% 12) Extra overlay plot for all states
figure;

subplot(4,1,1);
plot(t_lin, x_lin(:,1), 'b', 'LineWidth', 1.8); hold on;
plot(t_nl, x_nl(:,1), 'r--', 'LineWidth', 1.8);
grid on;
ylabel('x (m)');
title('State Comparison: Linearized vs Nonlinear');
legend('Linearized', 'Nonlinear');

subplot(4,1,2);
plot(t_lin, x_lin(:,2), 'b', 'LineWidth', 1.8); hold on;
plot(t_nl, x_nl(:,2), 'r--', 'LineWidth', 1.8);
grid on;
ylabel('x dot (m/s)');

subplot(4,1,3);
plot(t_lin, rad2deg(x_lin(:,3)), 'b', 'LineWidth', 1.8); hold on;
plot(t_nl, rad2deg(x_nl(:,3)), 'r--', 'LineWidth', 1.8);
grid on;
ylabel('\phi (deg)');

subplot(4,1,4);
plot(t_lin, rad2deg(x_lin(:,4)), 'b', 'LineWidth', 1.8); hold on;
plot(t_nl, rad2deg(x_nl(:,4)), 'r--', 'LineWidth', 1.8);
grid on;
ylabel('\phi dot (deg/s)');
xlabel('Time (s)');

%% ============================================================
% Local function: nonlinear dynamics
% ============================================================
function dx = nonlinear_sbr(~, x, K, M, m, b, I, g, l)

    % States
    pos     = x(1); %#ok<NASGU>
    vel     = x(2);
    phi     = x(3);
    phi_dot = x(4);

    % Control law: u = Kx
    u = K * x;

    % Nonlinear dynamics:
    %
    % (M+m)*xddot + b*xdot + m*l*phiddot*cos(phi) - m*l*phi_dot^2*sin(phi) = u
    % (I+m*l^2)*phiddot - m*g*l*sin(phi) = m*l*xddot*cos(phi)
    %
    % Rearranged as:
    %
    % [ M+m            m*l*cos(phi) ] [xddot  ] = [ u - b*vel + m*l*phi_dot^2*sin(phi) ]
    % [ -m*l*cos(phi)  I+m*l^2      ] [phiddot]   [ m*g*l*sin(phi)                      ]

    A_nl = [ M + m,            m*l*cos(phi);
            -m*l*cos(phi),     I + m*l^2 ];

    rhs = [ u - b*vel + m*l*phi_dot^2*sin(phi);
            m*g*l*sin(phi) ];

    acc = A_nl \ rhs;

    xddot   = acc(1);
    phiddot = acc(2);

    dx = zeros(4,1);
    dx(1) = vel;
    dx(2) = xddot;
    dx(3) = phi_dot;
    dx(4) = phiddot;


end