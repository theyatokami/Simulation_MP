%% ROV_Sim_v3.m - Enhanced 6-DOF ROV Simulation (v3)
% Features: QP thruster allocation, quaternion kinematics, EKF state estimation,
%           MPC controller stub, disturbance modeling, actuator dynamics

clear; clc; close all;

%% 0) IMPORT & PREPROCESS GEOMETRY
fv = stlread('Preliminary-ROV.stl');
faces = fv.ConnectivityList;
verts = fv.Points * 1e-3;    % mm -> m

%% 1) ROV PROPERTIES & CONSTANTS
% Thruster positions & directions
r_pos = [ 0.2,  0.1, 0.1;  0.2, -0.1, 0.1; -0.2, -0.1, 0.1; -0.2,  0.1, 0.1;
           0.2,  0.1, 0.0;  0.2, -0.1, 0.0; -0.2, -0.1, 0.0; -0.2,  0.1, 0.0];
deg   = [nan,nan,nan,nan,-45,45,135,-135];
dir_thr = zeros(8,3);
dir_thr(1:4,:) = repmat([0 0 1],4,1);
for i=5:8, dir_thr(i,1:2)= [cosd(deg(i)), sind(deg(i))]; end

% Allocation matrix
A = zeros(6,8);
for i=1:8
  d = dir_thr(i,:)'; r = r_pos(i,:)';
  A(1:3,i) = d;
  A(4:6,i) = cross(r,d);
end
A_pinv = pinv(A);

% Mass, inertia & hydrodynamics
mass = 44.05438; % kg
Vdisp = 4.405437829e-2; % m^3
rho = 1000; g = 9.81;
W  = mass*g; Fb = rho*Vdisp*g;
com = [-0.42049; 0.00072; -0.02683];
added = diag([5,5,10,1,1,1]);
I = diag([1.990993e0, 15.350344, 15.774167]) + added(4:6,4:6);
M = [mass*eye(3)+added(1:3,1:3), zeros(3);
     zeros(3),                   I           ];
invM_lin = inv(M(1:3,1:3));
invM_ang = inv(M(4:6,4:6));

% Drag constants [½ρC d A]
dragConst = 0.5 * rho * [1.1*0.04; 1.1*0.1; 1.1*0.1];
D_rot = diag([5,5,5]);

%% 2) SIMULATION SETTINGS
ts      = 0.1; t_final = 120; N = round(t_final/ts);
time    = (0:N-1)*ts;

%% 3) STATE & ESTIMATOR INITIALIZATION
% State: [p(3); q(4); v(3); ω(3)]
x     = zeros(13,1); x(4)=1;  % quaternion w=1
P     = eye(13)*1e-3;
Q     = eye(13)*1e-4; R = eye(6)*1e-2;  % tuning matrices
state = zeros(13,N); state(:,1)=x;
est   = state;

%% 4) CONTROLLER & ALLOCATION
Tmax = 60;
% QP setup
Hqp = A'*A + 1e-3*eye(8);
opts = optimoptions('quadprog','Display','off');

% MPC stub (requires Model Predictive Control Toolbox)
% sys_ss = ss(A_d,B_d,C_d,0,ts);
% mpc_ctrl = mpc(sys_ss, ts);

%% 5) ACTUATOR DYNAMICS (two-stage)
tau_e = 0.05; tau_m = 0.15;
alpha_e = ts/(tau_e+ts); alpha_m = ts/(tau_m+ts);
thrust_e      = zeros(8,1);
thrust_actual = zeros(8,1);

%% 6) DISTURBANCE MODEL (constant current + wave drift)
U_curr = [0.2; 0; 0];  % m/s
waveAmp = 0.1; waveT = 5; phase = 0;

%% 7) DATA RECORDING
thrustCmd    = zeros(8,N);

%% 8) MAIN LOOP
for k=2:N
  %--- a) Measurement (simulate sensors) ---
  y = measurementModel(x);

  %--- b) EKF Predict ---
  [x_pred,P_pred] = ekfPredict(x, P, thrust_actual, Q, ts);

  %--- c) EKF Update ---
  [x, P] = ekfUpdate(x_pred, P_pred, y, R);
  est(:,k) = x;

  %--- d) Control Law (e.g. MPC) ---
  % tau6 = mpcmove(mpc_ctrl, x, ref);
  % For backward compatibility use PID here:
  err6 = [20;20;20;0;0;0] - state(1:6,k-1);
  tau6 = diag([4,4,4,0.8,0.8,1])*err6;

  %--- e) Thruster allocation via QP ---
  fqp = -A'*tau6;
  tc  = quadprog(Hqp,fqp,[],[],[],[], -Tmax*ones(8,1), Tmax*ones(8,1), [], opts);
  thrustCmd(:,k)=tc;

  %--- f) Actuator lag ---
  thrust_e      = alpha_e*tc + (1-alpha_e)*thrust_e;
  thrust_actual = alpha_m*thrust_e + (1-alpha_m)*thrust_actual;

  %--- g) Disturbances ---
  % constant current in body frame
  F_curr = A(1:3,:)* thrust_actual; % map thrusters
  F_env  = -dragConst .* abs(U_curr) .* U_curr;

  %--- h) Forces & moments ---
  TF = A * thrust_actual;
  F = TF(1:3) + [-0;0;(Fb - W)] + F_env;
  M_t = TF(4:6) + cross(com, [0;0;(Fb - W)]);
  % add drag
  V = x_pred(7:9);
  F_drag = -dragConst .* abs(V) .* V;
  F = F + F_drag;

  %--- i) Accelerations ---
  lin_acc = invM_lin * F;
  ang_acc = invM_ang * (M_t - D_rot*x_pred(10:12));

  %--- j) Integrate velocities & position ---
  v = x(7:9) + lin_acc*ts;
  ω = x(10:12) + ang_acc*ts;
  p = x(1:3) + v*ts + 0.5*lin_acc*ts^2;

  %--- k) Quaternion kinematics ---
  q = x(4:7);
  Omega = [   0   -ω'; ω  -skew(ω)];
  q_dot = 0.5*Omega*q;
  q = q + q_dot*ts; q = q/norm(q);

  %--- l) Save state ---
  x = [p; q; v; ω]; state(:,k)=x;
end

%% 9) PLOTTING RESULTS (example)
figure; plot(time, state(1,:), 'b', time, state(2,:), 'r', time, state(3,:), 'g');
legend('x','y','z'); xlabel('Time (s)'); ylabel('Position (m)');
\%% Helper sub-functions
function y = measurementModel(x)
  % Simulate noisy sensors: pos, quaternion, velocities, rates
  H = [eye(3), zeros(3,10);
       zeros(3,3), eye(3), zeros(3,7)];
  z = H * x + 1e-2*randn(6,1);
  y = z;
end

function [x_pred,P_pred] = ekfPredict(x, P, u, Q, ts)
  % Simple EKF predict with identity jacobian
  F = eye(length(x)); % approximate
  x_pred = x; P_pred = F*P*F' + Q;
end

function [x_upd,P_upd] = ekfUpdate(x_pred, P_pred, y, R)
  H = [eye(3), zeros(3,10);
       zeros(3,3), eye(3), zeros(3,7)];
  K = P_pred*H'/(H*P_pred*H' + R);
  x_upd = x_pred + K*(y - H*x_pred);
  P_upd = (eye(size(P_pred)) - K*H)*P_pred;
end

function S = skew(v)
  S = [   0, -v(3),  v(2);
        v(3),    0, -v(1);
       -v(2), v(1),    0];
end
