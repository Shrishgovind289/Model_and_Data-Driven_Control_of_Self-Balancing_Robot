clc;
clear;
close all;

%% ============================================================
% NONLINEAR SELF-BALANCING ROBOT SIMULATION
% Controller designed from linearized model using LMI
% Control law: u = Kx
%
% States:
%   x1 = position
%   x2 = velocity
%   x3 = tilt angle phi
%   x4 = angular velocity
%% ============================================================

%% 1) Physical parameters
M = 0.5;      % Base / wheel assembly mass (kg)
m = 0.2;      % Body mass (kg)
b = rand();      % Viscous friction coefficient
I = 0.006;    % Body inertia about center of mass (kg.m^2)
g = 9.81;     % Gravity (m/s^2)
l = 0.3;      % Distance from axle to center of mass (m)

%% 2) Linearized model for LMI-based controller design
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

%% 3) LMI design for u = Kx
n = size(A,1);
m_in = size(B,2);

P = sdpvar(n,n,'symmetric');
Y = sdpvar(m_in,n,'full');

eps_val = 1e-6;

% For u = Kx, closed-loop system is x_dot = (A + BK)x
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

Acl = A + B*K;
disp('Closed-loop poles of linearized system:');
disp(eig(Acl));

%% 4) Initial condition
% Small initial tilt around upright equilibrium
x0 = [0;
      0;
      deg2rad(5);
      0];

%% 5) Nonlinear closed-loop simulation
tspan = [0 10];

% Simulate nonlinear dynamics with ode45
[t, x] = ode45(@(t,x) nonlinear_sbr_dynamics(t, x, K, M, m, b, I, g, l), tspan, x0);

%% 6) Compute control input over time
u = zeros(length(t),1);
for k = 1:length(t)
    u(k) = K * x(k,:)';
end

%% 7) Plot results
figure;

subplot(5,1,1);
plot(t, x(:,1), 'LineWidth', 1.5);
grid on;
ylabel('x (m)');
title('Nonlinear Closed-Loop Response of Self-Balancing Robot');

subplot(5,1,2);
plot(t, x(:,2), 'LineWidth', 1.5);
grid on;
ylabel('x dot (m/s)');

subplot(5,1,3);
plot(t, rad2deg(x(:,3)), 'LineWidth', 1.5);
grid on;
ylabel('\phi (deg)');

subplot(5,1,4);
plot(t, rad2deg(x(:,4)), 'LineWidth', 1.5);
grid on;
ylabel('\phi dot (deg/s)');

subplot(5,1,5);
plot(t, u, 'LineWidth', 1.5);
grid on;
ylabel('u');
xlabel('Time (s)');

%% 8) Separate angle plot
figure;
plot(t, rad2deg(x(:,3)), 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Tilt Angle \phi (deg)');
title('Nonlinear Tilt Angle Response');

%% 9) Separate position plot
figure;
plot(t, x(:,1), 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Position x (m)');
title('Nonlinear Position Response');

%% 10) Separate control input plot
figure;
plot(t, u, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Control Input u');
title('Nonlinear Control Effort');

%% ============================================================
% LOCAL FUNCTION: Nonlinear dynamics
%% ============================================================
function dx = nonlinear_sbr_dynamics(~, x, K, M, m, b, I, g, l)

    % State variables
    pos     = x(1);
    vel     = x(2);
    phi     = x(3);
    phi_dot = x(4);

    % Control law: u = Kx
    u = K * x;

    % Nonlinear coupled equations:
    %
    % (M+m)*xddot + b*xdot + m*l*phiddot*cos(phi) - m*l*phi_dot^2*sin(phi) = u
    %
    % (I+m*l^2)*phiddot - m*g*l*sin(phi) = m*l*xddot*cos(phi)
    %
    % Rearranged in matrix form:
    %
    % [ M+m              m*l*cos(phi) ] [xddot  ] = [ u - b*vel + m*l*phi_dot^2*sin(phi) ]
    % [ -m*l*cos(phi)    I+m*l^2      ] [phiddot]   [ m*g*l*sin(phi)                      ]

    A_nl = [ M + m,            m*l*cos(phi);
            -m*l*cos(phi),     I + m*l^2 ];

    rhs = [ u - b*vel + m*l*phi_dot^2*sin(phi);
            m*g*l*sin(phi) ];

    acc = A_nl \ rhs;

    xddot   = acc(1);
    phiddot = acc(2);

    % State derivative vector
    dx = zeros(4,1);
    dx(1) = vel;
    dx(2) = xddot;
    dx(3) = phi_dot;
    dx(4) = phiddot;
end