% Daniel Torres
% Final Project - Multirotor Aircraft
% MCEN 5228 - Linear Systems
% Fall 2019

clear all; close all; clc;

% Multirotor aircraft state space realizations

A = [0 1 0 0 0 0; 
     0 -0.0104 0 0 0 0; 
     0 0 0 1 0 0; 
     0 0 0 -0.0104 0 0;
     0 0 0 0 0 1;
     0 0 0 0 0 -0.0208];
 
B = [0 0 0 0;
     -0.04167 0 0 0.04167;
     0 0 0 0;
     0 -0.04167 0 0.04167;
     0 0 0 0;
     0.4 0.4 0.4 0.4];
 
C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];
  
D = [0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0];

sys = ss(A,B,C,0); % create state space model object

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

% Part 2: Determine the system poles

% calculate A matrix eigenvalues
poles_ol = eig(A); % poles of open loop system
fprintf('The system poles are: %f, %f, %f, %f, %f and %f.\n', poles_ol);
disp('The system poles are either zero or negative so it''s marginally stable.');

% simulate input to system without controller
t = 0:0.01:5; % time vector
x0 = [1; 0; -2; 0; 0; 0]; % initial state vector
input_gain = 1; % input gain variable to scale input to thrusters (for testing)
u1 = input_gain*ones(1,length(t)); % input to thruster 1
u2 = input_gain*ones(1,length(t)); % input to thruster 2
u3 = input_gain*ones(1,length(t)); % input to thruster 3
u4 = input_gain*ones(1,length(t)); % input to thruster 4
u = [u1; u2; u3; u4]; % combine all thruster inputs into one vector
[y_ol,t_ol,x_ol] = lsim(sys,u,t,x0); % simulate open-loop response

% plot open-loop response x, y, z inertial positions
figure(1); 
plot(t_ol,y_ol(:,1),'LineWidth',2); % plot open loop x-position
hold on; grid on;
plot(t_ol,y_ol(:,2),'LineWidth',2); % plot open loop y-position
plot(t_ol,y_ol(:,3),'LineWidth',2); % plot open loop z-position
title('Open Loop Response');
xlabel('Time [s]');
ylabel('Distance [m]');
legend('x','y','z','Location','Northwest');
legend boxoff

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Part 3: Determine controllability, observability, stabilizability,
%         and detectability

% determining controllability matrix and rank
Co = ctrb(A,B);
rank_Co = rank(Co);
fprintf('Controllability matrix rank: %d\n', rank_Co);

% determining observability matrix and rank
Ob = obsv(A,C);
rank_Ob = rank(Ob);
fprintf('Observability matrix rank: %d\n', rank_Ob);

% determining stabilizability matrix and rank
stbl = [A B];
rank_stbl = rank(stbl);
fprintf('Stabilizability matrix rank: %d\n', rank_stbl);

% determining detectability matrix and rank
detec = [A; C];
rank_detec = rank(detec);
fprintf('Detectability matrix rank: %d\n', rank_detec);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Part 4: Design a state feedback controller

poles_cl = [-8 -8.5 -9 -9.5 -10 -10.5]; % poles for state-feedback controller
k = place(A,B,poles_cl); % calculate k gain matrix to place poles

f = 44; % precompensation gain to eliminate steady state error
A_cl = A-B*k; % closed-loop A matrix
B_cl = B*f; % closed-loop B matrix
sys_cl = ss(A_cl,B_cl,C,0); % create closed-loop state space object
[y_cl,t_cl,x_cl] = lsim(sys_cl,u,t,x0); % simulate closed-loop response

% closed-loop x, y, z positions
figure(2); 
plot(t_cl,y_cl(:,1),'LineWidth',2); % plot closed-loop x-position
hold on; grid on;
plot(t_cl,y_cl(:,2),'LineWidth',2); % plot closed-loop y-position
plot(t_cl,y_cl(:,3),'LineWidth',2); % plot closed-loop z-position
title('Closed Loop Response');
xlabel('Time [s]');
ylabel('Distance [m]');
legend('x','y','z','Location','Southeast');
legend boxoff

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Part 5: Design an observer to reconstruct the state

x0_obs = [x0' x0']; % initial state vector for observer
poles_obs = poles_cl.*10; % place observer poles 10x as far as CL poles
L = place(A',C',poles_obs)'; % find gains matrix L for observer poles
At = [A-B*k B*k; zeros(size(A)) A-L*C]; % A-tilde matrix
Bt = [B*f; zeros(size(B))]; % B-tilde matrix
Ct = [C zeros(size(C))]; % C-tilde matrix
sys_obs = ss(At,Bt,Ct,0); % create observer state space system
[y_obs,t_obs,x_obs] = lsim(sys_obs,u,t,x0_obs); % simulate observer response

figure(3); % plot observer response
states = x_obs(:,1:2:6); % x, y, z states from observer
e = x_obs(:,7:2:12); % error from observer
x_hat = states - e; % calculate estimated states
x_ = states(:,1); % x-state
y_ = states(:,2); % y-state
z_ = states(:,3); % z-state
x_est = x_hat(:,1); % estimated x-state
y_est = x_hat(:,2); % estimated y-state
z_est = x_hat(:,3); % estimated z-state

% plot x, y , z states
plot(t_obs,x_,'LineWidth',2); hold on; grid on; 
plot(t_obs,y_,'LineWidth',2);
plot(t_obs,z_,'LineWidth',2); 

% plot estimated x, y, z states
plot(t_obs,x_est,'LineWidth',2);
plot(t_obs,y_est,'LineWidth',2);
plot(t_obs,z_est,'LineWidth',2);

title('Observer');
xlabel('Time [s]');
ylabel('Distance [m]');
legend('x','y','z','x-est','y-est','z-est','Location','Southeast');
legend boxoff

figure(4); % plot the first 0.5 seconds of observer to see convergence
% plot x, y , z states
len = 1:51;
plot(t_obs(len),x_(len),'LineWidth',2); hold on; grid on; 
plot(t_obs(len),y_(len),'LineWidth',2);
plot(t_obs(len),z_(len),'LineWidth',2); 

% plot estimated x, y, z states
plot(t_obs(len),x_est(len),'LineWidth',2);
plot(t_obs(len),y_est(len),'LineWidth',2);
plot(t_obs(len),z_est(len),'LineWidth',2);

title('Observer (Zoomed In)');
xlabel('Time [s]');
ylabel('Distance [m]');
legend('x','y','z','x-est','y-est','z-est','Location','Southeast');
legend boxoff

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


