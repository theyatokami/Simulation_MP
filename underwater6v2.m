%% 0) Import & decimate hull geometry from STL (mm → m)
fv_full    = stlread('Preliminary-ROV.stl');          % returns triangulation object
faces0     = fv_full.ConnectivityList;                % original faces
verts0_mm  = fv_full.Points;                          % original vertices (mm)
[faces, verts_mm] = reducepatch(faces0, verts0_mm, 0.2);  % keep 20% of faces
vertices   = verts_mm * 1e-3;                         % convert mm → m

%% 1) Desired Pose [x; y; z; roll; pitch; yaw]
desired = [20; 20; 20; 0; 0; 0];

%% 2) Thruster Definitions & Allocation Matrix A (6×8)
thruster_positions = [  % [x, y, z] in body frame (m)
   0.2,  0.1,  0.1;   % T1
   0.2, -0.1,  0.1;   % T2
  -0.2, -0.1,  0.1;   % T3
  -0.2,  0.1,  0.1;   % T4
   0.2,  0.1,  0.0;   % T5
   0.2, -0.1,  0.0;   % T6
  -0.2, -0.1,  0.0;   % T7
  -0.2,  0.1,  0.0];  % T8

theta = [nan,nan,nan,nan,-45,45,135,-135];
thruster_directions = zeros(8,3);
thruster_directions(1:4,:) = repmat([0,0,1],4,1);   % vertical thrusters
for i = 5:8
    thruster_directions(i,1:2) = [cosd(theta(i)), sind(theta(i))];
end

A = zeros(6,8);
for i = 1:8
    d = thruster_directions(i,:)';
    r = thruster_positions(i,:)';
    A(1:3,i)   = d;           % force
    A(4:6,i)   = cross(r,d);  % moment
end

%% 3) Simulation Setup
ts        = 0.1;            % time step [s]
t_final   = 120;            % total time [s]
N         = round(t_final/ts);
state     = zeros(12, N);
timeArray = (0:N-1)*ts;

% Preallocate thrust history (8 thrusters × N time steps)
thrustHistory = zeros(8, N);

%% 4) Vehicle Dynamics, Buoyancy & Drag (using CAD‐derived mass/inertia)
% CAD mass/volume (converted to SI)
mass_g    = 44054.38;             % [g]
mass      = mass_g * 1e-3;        % [kg]
Vdisp_mm3 = 44054378.29;          % [mm^3]
Vdisp     = Vdisp_mm3 * 1e-9;     % [m^3]
g         = 9.81;                 
rho       = 1000;                 % [kg/m^3]
W         = mass * g;             % total weight
Fb        = rho * Vdisp * g;      % total buoyant force

% Center of mass (CAD): (mm → m)
com_mm = [-420.49, 0.72, -26.83];
com    = com_mm * 1e-3;           % [m]

% Added‐mass (retain original approximate values)
added_mass = diag([5,5,10, 1,1,1]);  % [kg] for linear x,y,z and rotational x,y,z

% Inertia from CAD (output coordinate system) in g·mm^2
Ixx_gmm = 1990992913.93;  Iyy_gmm = 15350343950.54;  Izz_gmm = 15774126677.55;
Ixy_gmm = -14479665.77;   Ixz_gmm = 1499586321.98;   Iyz_gmm = -1136504.63;

% Convert to kg·m^2: multiply by 1e-9
Ixx = Ixx_gmm * 1e-9;
Iyy = Iyy_gmm * 1e-9;
Izz = Izz_gmm * 1e-9;
Ixy = Ixy_gmm * 1e-9;
Ixz = Ixz_gmm * 1e-9;
Iyz = Iyz_gmm * 1e-9;

% Build 6×6 mass/inertia matrix M
M = zeros(6);
M(1:3,1:3) = mass * eye(3) + added_mass(1:3,1:3);
M(4:6,4:6) = [ Ixx, -Ixy, -Ixz;
               -Ixy,  Iyy, -Iyz;
               -Ixz, -Iyz,  Izz ] + added_mass(4:6,4:6);

% Hydrodynamic drag parameters
A_x  = 0.2 * 0.2;  Cdx = 1.1;
A_y  = 0.5 * 0.2;  Cdy = 1.1;
A_z  = 0.5 * 0.2;  Cdz = 1.1;

% Rotational damping
D_rot = diag([5,5,5]);

%% 5) PID Controller Gains
Kp = diag([4,4,4,0.8,0.8,1.0]);
Ki = diag([0.001,0.001,0.001,0,0,0]);
Kd = diag([0.6,0.6,0.6,1.5,1.5,2.0]);
integralError = zeros(6,1);
prevError     = zeros(6,1);

%% 6) Thruster Limits & GUI
Tmax = 60; Tmin = -60;
global userThrust thrusterEdits
userThrust = zeros(8,1);

%% 7) 2D Visualization Setup (5 subplots including thrust history)
fig2D = figure('Name','2D Plots','NumberTitle','off');

% Position subplot
subplot(5,1,1);
hPos(1) = plot(nan,nan,'b','LineWidth',2); hold on;
hPos(2) = plot(nan,nan,'r','LineWidth',2);
hPos(3) = plot(nan,nan,'g','LineWidth',2); hold off;
legend('x','y','z'); xlabel('Time (s)'); ylabel('Position (m)');
title('Position vs Time');

% Orientation subplot
subplot(5,1,2);
hOri(1) = plot(nan,nan,'b','LineWidth',2); hold on;
hOri(2) = plot(nan,nan,'r','LineWidth',2);
hOri(3) = plot(nan,nan,'g','LineWidth',2); hold off;
legend('Roll','Pitch','Yaw'); xlabel('Time (s)'); ylabel('Orientation (rad)');
title('Orientation vs Time');

% Linear Velocities subplot
subplot(5,1,3);
hVel(1) = plot(nan,nan,'b','LineWidth',2); hold on;
hVel(2) = plot(nan,nan,'r','LineWidth',2);
hVel(3) = plot(nan,nan,'g','LineWidth',2); hold off;
legend('u','v','w'); xlabel('Time (s)'); ylabel('Vel (m/s)');
title('Linear Velocities vs Time');

% Angular Velocities subplot
subplot(5,1,4);
hAng(1) = plot(nan,nan,'b','LineWidth',2); hold on;
hAng(2) = plot(nan,nan,'r','LineWidth',2);
hAng(3) = plot(nan,nan,'g','LineWidth',2); hold off;
legend('p','q','r'); xlabel('Time (s)'); ylabel('ω (rad/s)');
title('Angular Velocities vs Time');

% Thruster thrusts subplot
subplot(5,1,5);
for i = 1:8
    hThr(i) = plot(nan,nan,'LineWidth',1.5); hold on;
end
hold off;
legend('T1','T2','T3','T4','T5','T6','T7','T8','Location','northeast');
xlabel('Time (s)'); ylabel('Thrust (N)');
title('Thruster Commands vs Time');

%% 8) 3D Thruster-Only View
figThr = figure('Name','3D Thruster View','NumberTitle','off');
axThr = axes('Parent',figThr); grid(axThr,'on'); axis(axThr,'equal');
xlabel(axThr,'X'); ylabel(axThr,'Y'); zlabel(axThr,'Z'); view(axThr,3); hold(axThr,'on');
hRx = line(axThr,nan,nan,nan,'Color','b','LineWidth',1);
hRy = line(axThr,nan,nan,nan,'Color','r','LineWidth',1);
hRz = line(axThr,nan,nan,nan,'Color','g','LineWidth',1);
arrowScaleThr = 0.1;

%% 9) 3D Trajectory & Simple Body Box
figBody = figure('Name','Trajectory & Body','NumberTitle','off');
axBody = axes('Parent',figBody); grid(axBody,'on'); axis(axBody,'equal');
xlabel(axBody,'X'); ylabel(axBody,'Y'); zlabel(axBody,'Z'); view(axBody,3); hold(axBody,'on');
scatter3(axBody, state(1,1), state(2,1), state(3,1), 80, 'go','filled','DisplayName','Start');
scatter3(axBody, desired(1), desired(2), desired(3), 80, 'rx','LineWidth',2,'DisplayName','Desired');
hTraj = plot3(axBody, nan,nan,nan, 'b--','LineWidth',1.5,'DisplayName','Trajectory');
legend(axBody,'Location','best');
% simple rectangular box for body
hx = 0.25; hy = 0.1; hz = 0.1;
vertsBox = [ hx, hy, hz;  -hx, hy, hz;  -hx,-hy, hz;  hx,-hy, hz;
             hx, hy,-hz;  -hx, hy,-hz;  -hx,-hy,-hz;  hx,-hy,-hz ];
facesBox = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
bodyPatch = patch('Vertices',vertsBox,'Faces',facesBox,'FaceColor','cyan','FaceAlpha',0.3,'Parent',axBody);

%% 10) 3D Hull + Thrusters + Path Visualizer
fig3D = figure('Name','3D Hull + Thrusters + Path','NumberTitle','off');
ax3D = axes('Parent',fig3D); hold(ax3D,'on'); grid(ax3D,'on'); axis(ax3D,'equal'); view(ax3D,3);
tr     = hgtransform('Parent',ax3D);
patch('Faces',faces,'Vertices',vertices,'FaceColor','cyan','FaceAlpha',0.3,'Parent',tr);
xlabel(ax3D,'X'); ylabel(ax3D,'Y'); zlabel(ax3D,'Z');
hPath3D = plot3(ax3D, NaN, NaN, NaN, 'b-', 'LineWidth',1.5);
for i = 1:8
    hQ3(i) = quiver3(ax3D,0,0,0,0,0,0,'r','LineWidth',1.5,'MaxHeadSize',2,'AutoScale','off');
    hT3(i) = text(ax3D,0,0,0,'','FontSize',8,'FontWeight','bold','HorizontalAlignment','center');
end
arrowScale3 = 0.03;

%% 11) Main Simulation Loop
tic;
for k = 2:N
  % a) PID computation
  pose = state(1:6,k-1);
  err  = desired - pose;
  integralError = integralError + err * ts;
  deriv        = (err - prevError) / ts;
  prevError    = err;
  tau6         = Kp*err + Ki*integralError + Kd*deriv;

  % b) Thruster commands with uniform scaling to avoid any saturation
  raw_tc = pinv(A) * tau6;         % unconstrained solution
  peak   = max(abs(raw_tc));      
  if peak > Tmax
      scale = Tmax / peak;         % compute shrink factor
      raw_tc = raw_tc * scale;     % scale entire vector
  end
  tc = raw_tc;                     % now |tc(i)| ≤ Tmax ∀i


  % Store thrust commands in history
  thrustHistory(:,k) = tc;

  % c) Forces & torques
  TF = A * tc;
  F  = TF(1:3);   Mt = TF(4:6);

  % Neutral buoyancy: skip adding (Fb - W), since Fb == W
  % F(3) = F(3) + (Fb - W);

  % Restoring moment due to COM offset (zero if Fb==W)
  Mt = Mt + cross(com', [0;0;(Fb - W)]);

  u = state(7,k-1); v = state(8,k-1); w = state(9,k-1);
  F_drag = [ -0.5*rho*Cdx*A_x*abs(u)*u;
             -0.5*rho*Cdy*A_y*abs(v)*v;
             -0.5*rho*Cdz*A_z*abs(w)*w ];
  F = F + F_drag;

  % d) Accelerations
  lin_acc = M(1:3,1:3) \ F;
  ang_acc = M(4:6,4:6) \ (Mt - D_rot*state(10:12,k-1));

  % e) Integrate velocities
  state(7:9,k)   = state(7:9,k-1)   + lin_acc * ts;
  state(10:12,k) = state(10:12,k-1) + ang_acc * ts;

  % f) Integrate positions/orientations
  state(1:3,k) = state(1:3,k-1) + state(7:9,k-1)*ts + 0.5*lin_acc*ts^2;
  state(4:6,k) = state(4:6,k-1) + state(10:12,k-1)*ts + 0.5*ang_acc*ts^2;

  % g) Update 2D plots
  if mod(k,5)==0 || k==N
    t = timeArray(1:k);
    for i = 1:3
      set(hPos(i), 'XData', t, 'YData', state(i,1:k));
      set(hOri(i), 'XData', t, 'YData', state(i+3,1:k));
      set(hVel(i), 'XData', t, 'YData', state(i+6,1:k));
      set(hAng(i), 'XData', t, 'YData', state(i+9,1:k));
    end
    % Update thruster thrusts with true history
    for i = 1:8
      set(hThr(i), 'XData', t, 'YData', thrustHistory(i,1:k));
    end
    drawnow;
  end

  % h) Update 3D Thruster-Only View
  pos = state(1:3,k);
  phi = state(4,k); th = state(5,k); psi = state(6,k);
  R = [ cos(psi)*cos(th),  cos(psi)*sin(th)*sin(phi)-sin(psi)*cos(phi),  cos(psi)*sin(th)*cos(phi)+sin(psi)*sin(phi);
        sin(psi)*cos(th),  sin(psi)*sin(th)*sin(phi)+cos(psi)*cos(phi),  sin(psi)*sin(th)*cos(phi)-cos(psi)*sin(phi);
       -sin(th),           cos(th)*sin(phi),                             cos(th)*cos(phi) ];
  L = 0.1;
  Xe = pos + R*[L;0;0]; Ye = pos + R*[0;L;0]; Ze = pos + R*[0;0;L];
  set(hRx, 'XData', [pos(1), Xe(1)], 'YData', [pos(2), Xe(2)], 'ZData', [pos(3), Xe(3)]);
  set(hRy, 'XData', [pos(1), Ye(1)], 'YData', [pos(2), Ye(2)], 'ZData', [pos(3), Ye(3)]);
  set(hRz, 'XData', [pos(1), Ze(1)], 'YData', [pos(2), Ze(2)], 'ZData', [pos(3), Ze(3)]);
  delete(findall(axThr,'Type','Quiver')); delete(findall(axThr,'Type','Text'));
  for i = 1:8
    Pw = pos + R*thruster_positions(i,:)';
    Vw = R*(thruster_directions(i,:)' * tc(i) * arrowScaleThr);
    quiver3(axThr, Pw(1), Pw(2), Pw(3), Vw(1), Vw(2), Vw(3), 'r', 'LineWidth',2, 'MaxHeadSize',2);
    text(axThr, Pw(1)+Vw(1)*1.1, Pw(2)+Vw(2)*1.1, Pw(3)+Vw(3)*1.1, ...
         sprintf('%.1f', tc(i)), 'FontSize',10, 'FontWeight','bold', 'HorizontalAlignment','center');
  end

  % i) Update Body View
  set(hTraj, 'XData', state(1,1:k), 'YData', state(2,1:k), 'ZData', state(3,1:k));
  vertsWorld = (R*vertsBox')' + pos';
  set(bodyPatch, 'Vertices', vertsWorld);

  % j) Update 3D Hull+Thrusters+Path
  T4 = eye(4); T4(1:3,1:3) = R; T4(1:3,4) = pos;
  set(tr, 'Matrix', T4);
  for i = 1:8
    Pw3 = pos + R*thruster_positions(i,:)';
    Fv3 = R*(thruster_directions(i,:)' * tc(i) * arrowScale3);
    set(hQ3(i), 'XData', Pw3(1), 'YData', Pw3(2), 'ZData', Pw3(3), ...
               'UData', Fv3(1), 'VData', Fv3(2), 'WData', Fv3(3));
    set(hT3(i), 'Position', Pw3 + Fv3*1.1, 'String', sprintf('%.1f', tc(i)));
  end
  set(hPath3D, 'XData', state(1,1:k), 'YData', state(2,1:k), 'ZData', state(3,1:k));

  drawnow limitrate;
  elapsed = toc;
  if elapsed < ts, pause(ts - elapsed); end
  tic;
end

%% Callback: Manual Thruster Override
function applyCallback(~,~)
  global userThrust thrusterEdits
  for i = 1:8
    userThrust(i) = str2double(get(thrusterEdits(i), 'String'));
  end
  disp('Manual thruster override:');
  disp(userThrust');
end