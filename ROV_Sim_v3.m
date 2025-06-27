%% ROV_Sim_v4.m – 6-DOF ROV Simulation Ready for ANSYS Integration
% Single-file script incorporating placeholders for ANSYS-derived parameters.

clear; clc; close all;

%% 0) IMPORT, DECIMATE & PREPROCESS GEOMETRY
scriptDir = fileparts(mfilename('fullpath'));
stlFile   = fullfile(scriptDir,'..','Preliminary-ROV.stl');
if ~isfile(stlFile), error('STL not found: %s',stlFile); end
fv_full   = stlread(stlFile);
faces0    = fv_full.ConnectivityList;
verts0_mm = fv_full.Points;
[faces, verts_mm] = reducepatch(faces0, verts0_mm, 0.2);  % 20% of faces
vertices  = verts_mm * 1e-3;  % convert mm→m

%% 1) ANSYS PARAMETERS PLACEHOLDER
% Replace 'ansys_results.mat' with your ANSYS output file when ready
% Expected variables inside ansys_results.mat:
%  - mass (kg)
%  - Vdisp (m^3)\%  - com (3×1 vector, m)
%  - I (3×3 inertia matrix, kg·m^2)
%  - added (6×6 added-mass matrix)
%  - CdA (3×1 vector [CdAx; CdAy; CdAz])
%  - D_rot (3×3 rotational damping)
%  - Fb (buoyancy force, N)
%  - any other custom coefficients
if isfile(fullfile(scriptDir,'ansys_results.mat'))
    ansysData = load(fullfile(scriptDir,'ansys_results.mat'));
    mass     = ansysData.mass;
    Vdisp    = ansysData.Vdisp;
    com      = ansysData.com;
    I_mat    = ansysData.I;
    added    = ansysData.added;
    CdA      = ansysData.CdA;
    D_rot    = ansysData.D_rot;
    Fb       = ansysData.Fb;
else
    % Default/stub values until ANSYS data available
    mass  = 44.05438;             % kg
    Vdisp = 4.405437829e-2;       % m^3
    com   = [-0.42049;0.00072;-0.02683];
    I_mat = diag([1.990993,15.350344,15.774167]);
    added = diag([5,5,10,1,1,1]);
    CdA   = 0.5*1000*[1.1*0.04;1.1*0.1;1.1*0.1];  % ½ρC·A defaults
    D_rot = diag([5,5,5]);
    Fb    = 1000*Vdisp*9.81;
    fprintf('Warning: Using default hydrodynamic & inertial parameters.\n');
end
W = mass * 9.81;  % weight

% Build 6×6 mass/inertia matrix
M = [mass*eye(3)+added(1:3,1:3), zeros(3);
     zeros(3),                   I_mat + added(4:6,4:6)];
invM_lin = inv(M(1:3,1:3));
invM_ang = inv(M(4:6,4:6));

%% 2) DESIRED POSE & THRUSTER SETUP
desired = [20;20;20; 0;0;0];  % [x;y;z;roll;pitch;yaw]
r_pos = [0.2,0.1,0.1; 0.2,-0.1,0.1; -0.2,-0.1,0.1; -0.2,0.1,0.1;
         0.2,0.1,0.0; 0.2,-0.1,0.0; -0.2,-0.1,0.0; -0.2,0.1,0.0];
deg = [nan,nan,nan,nan,-45,45,135,-135];
dir_thr = repmat([0,0,1],8,1);
for i=5:8, dir_thr(i,1:2) = [cosd(deg(i)), sind(deg(i))]; end
A = zeros(6,8);
for i=1:8, d=dir_thr(i,:)'; r=r_pos(i,:)'; A(1:3,i)=d; A(4:6,i)=cross(r,d); end

%% 3) CONTROLLER, ACTUATOR & ESTIMATOR INIT
% Simulation time
ts      = 0.1; t_final = 120; N = round(t_final/ts); t=(0:N-1)*ts;
% State vector [p; angles; Vel; angRates]
state = zeros(12,N);
% PID gains
Kp = diag([4,4,4,0.8,0.8,1]);
Ki = diag([0.001,0.001,0.001,0,0,0]);
Kd = diag([0.6,0.6,0.6,1.5,1.5,2]);
intErr = zeros(6,1); prevErr = zeros(6,1);
% QP allocation
tmp     = A'*A + 1e-3*eye(8);
opts    = optimoptions('quadprog','Display','off');
Tmax    = 60;
% Actuator lags
tau_e   = 0.05; tau_m = 0.15;
alpha_e = ts/(tau_e+ts); alpha_m = ts/(tau_m+ts);
thrust_e      = zeros(8,1);
thrust_act    = zeros(8,1);
% Data recording
thrustHist = zeros(8,N);

%% 4) VISUALIZATION SETUP
% 2D figure with 5 subplots
gFig = figure('Name','2D Plots');
ax(1)=subplot(5,1,1); hP(1)=plot(nan,nan,'b'); hold on; hP(2)=plot(nan,nan,'r'); hP(3)=plot(nan,nan,'g'); title('Position');
ax(2)=subplot(5,1,2); hO(1)=plot(nan,nan,'b'); hold on; hO(2)=plot(nan,nan,'r'); hO(3)=plot(nan,nan,'g'); title('Orientation');
ax(3)=subplot(5,1,3); hV(1)=plot(nan,nan,'b'); hold on; hV(2)=plot(nan,nan,'r'); hV(3)=plot(nan,nan,'g'); title('Velocities');
ax(4)=subplot(5,1,4); hW(1)=plot(nan,nan,'b'); hold on; hW(2)=plot(nan,nan,'r'); hW(3)=plot(nan,nan,'g'); title('Angular Rates');
ax(5)=subplot(5,1,5); for i=1:8, hT(i)=plot(nan,nan); hold on; end; title('Thruster Cmds');
% 3D plots
gThr = figure('Name','3D Thruster View'); axThr = axes(gThr); grid(axThr,'on'); axis(axThr,'equal'); hold(axThr,'on');
arwScale = 0.1;
gBody=figure('Name','Trajectory & Body'); axBody = axes(gBody); grid(axBody,'on'); axis(axBody,'equal'); hold(axBody,'on');
hTraj = plot3(axBody,nan,nan,nan,'b--'); scatter3(axBody,desired(1),desired(2),desired(3),'rx');
g3D = figure('Name','3D Hull+Thrusters'); ax3D = axes(g3D); grid(ax3D,'on'); axis(ax3D,'equal'); hold(ax3D,'on');
tr = hgtransform('Parent',ax3D);
patch('Faces',faces,'Vertices',vertices,'FaceColor','cyan','FaceAlpha',0.3,'Parent',tr);

%% 5) MAIN SIMULATION LOOP
for k=2:N
  %--- PID control ---
  err   = desired - state(1:6,k-1);
  intErr= intErr + err*ts;
  dErr  = (err - prevErr)/ts; prevErr=err;
  tau   = Kp*err + Ki*intErr + Kd*dErr;
  %--- Thruster allocation via QP ---
  tc         = quadprog(tmp, -A'*tau, [],[],[],[], -Tmax*ones(8,1), Tmax*ones(8,1), [], opts);
  thrustHist(:,k)=tc;
  %--- Actuator dynamics ---
  thrust_e   = alpha_e*tc + (1-alpha_e)*thrust_e;
  thrust_act = alpha_m*thrust_e + (1-alpha_m)*thrust_act;
  %--- Forces & moments ---
  TF  = A*thrust_act;
  F   = TF(1:3);
  M_t = TF(4:6) + cross(com, [0;0;(Fb-W)]);
  % hydrodynamic drag
  V   = state(7:9,k-1);
  Fd  = -CdA .* abs(V) .* V;
  F   = F + Fd;
  %--- Accelerations & integration ---
  acc_lin      = invM_lin * F;
  acc_ang      = invM_ang * (M_t - D_rot * state(10:12,k-1));
  state(7:9,k)   = state(7:9,k-1) + acc_lin*ts;
  state(10:12,k) = state(10:12,k-1) + acc_ang*ts;
  state(1:3,k)   = state(1:3,k-1) + state(7:9,k-1)*ts + 0.5*acc_lin*ts^2;
  state(4:6,k)   = state(4:6,k-1) + state(10:12,k-1)*ts + 0.5*acc_ang*ts^2;
  %--- Visualization updates ---
  tvec = t(1:k);
  % 2D
  set(hP(1),'XData',tvec,'YData',state(1,1:k)); set(hP(2),'XData',tvec,'YData',state(2,1:k)); set(hP(3),'XData',tvec,'YData',state(3,1:k));
  set(hO(1),'XData',tvec,'YData',state(4,1:k)); set(hO(2),'XData',tvec,'YData',state(5,1:k)); set(hO(3),'XData',tvec,'YData',state(6,1:k));
  set(hV(1),'XData',tvec,'YData',state(7,1:k)); set(hV(2),'XData',tvec,'YData',state(8,1:k)); set(hV(3),'XData',tvec,'YData',state(9,1:k));
  set(hW(1),'XData',tvec,'YData',state(10,1:k)); set(hW(2),'XData',tvec,'YData',state(11,1:k)); set(hW(3),'XData',tvec,'YData',state(12,1:k));
  for i=1:8, set(hT(i),'XData',tvec,'YData',thrustHist(i,1:k)); end
  drawnow limitrate;
  % 3D Thruster only view
  delete(findall(axThr,'Type','Quiver')); delete(findall(axThr,'Type','Text'));
  pos=state(1:3,k); phi=state(4,k); th=state(5,k); psi=state(6,k);
  Rz=[cos(psi),-sin(psi),0; sin(psi),cos(psi),0;0,0,1];
  Ry=[cos(th),0,sin(th);0,1,0;-sin(th),0,cos(th)];
  Rx=[1,0,0;0,cos(phi),-sin(phi);0,sin(phi),cos(phi)];
  R = Rz*Ry*Rx;
  for i=1:8
    Pw = pos + R*r_pos(i,:)';
    Vw = R*(dir_thr(i,:)'*thrust_act(i)*arwScale);
    quiver3(axThr,Pw(1),Pw(2),Pw(3),Vw(1),Vw(2),Vw(3),'r','LineWidth',2,'MaxHeadSize',2);
    text(axThr,Pw(1)+1.1*Vw(1),Pw(2)+1.1*Vw(2),Pw(3)+1.1*Vw(3),sprintf('%.1f',thrust_act(i)),'FontWeight','bold');
  end
  % 3D Trajectory
  set(hTraj,'XData',state(1,1:k),'YData',state(2,1:k),'ZData',state(3,1:k));
  % 3D Hull
  T4=eye(4); T4(1:3,1:3)=R; T4(1:3,4)=pos; set(tr,'Matrix',T4);
  drawnow limitrate;
end
% End of ROV_Sim_v4.m
