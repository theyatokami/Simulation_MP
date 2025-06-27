classdef ROVSimGUI2 < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                  matlab.ui.Figure
        RunSimulationButton       matlab.ui.control.Button
        % Inertial Properties
        MassLabel                 matlab.ui.control.Label
        MassEditField             matlab.ui.control.NumericEditField
        VdispLabel                matlab.ui.control.Label
        VdispEditField            matlab.ui.control.NumericEditField
        COMxLabel                 matlab.ui.control.Label
        COMxEditField             matlab.ui.control.NumericEditField
        COMyLabel                 matlab.ui.control.Label
        COMyEditField             matlab.ui.control.NumericEditField
        COMzLabel                 matlab.ui.control.Label
        COMzEditField             matlab.ui.control.NumericEditField
        IxxLabel                  matlab.ui.control.Label
        IxxEditField              matlab.ui.control.NumericEditField
        IyyLabel                  matlab.ui.control.Label
        IyyEditField              matlab.ui.control.NumericEditField
        IzzLabel                  matlab.ui.control.Label
        IzzEditField              matlab.ui.control.NumericEditField
        % Hydrodynamic Properties
        AddedMxLabel              matlab.ui.control.Label
        AddedMxEditField          matlab.ui.control.NumericEditField
        AddedMyLabel              matlab.ui.control.Label
        AddedMyEditField          matlab.ui.control.NumericEditField
        AddedMzLabel              matlab.ui.control.Label
        AddedMzEditField          matlab.ui.control.NumericEditField
        AddedIxxLabel             matlab.ui.control.Label
        AddedIxxEditField         matlab.ui.control.NumericEditField
        AddedIyyLabel             matlab.ui.control.Label
        AddedIyyEditField         matlab.ui.control.NumericEditField
        AddedIzzLabel             matlab.ui.control.Label
        AddedIzzEditField         matlab.ui.control.NumericEditField
        CdAxLabel                 matlab.ui.control.Label
        CdAxEditField             matlab.ui.control.NumericEditField
        CdAyLabel                 matlab.ui.control.Label
        CdAyEditField             matlab.ui.control.NumericEditField
        CdAzLabel                 matlab.ui.control.Label
        CdAzEditField             matlab.ui.control.NumericEditField
        DRollLabel                matlab.ui.control.Label
        DRollEditField            matlab.ui.control.NumericEditField
        DPitchLabel               matlab.ui.control.Label
        DPitchEditField           matlab.ui.control.NumericEditField
        DYawLabel                 matlab.ui.control.Label
        DYawEditField             matlab.ui.control.NumericEditField
        % Controller Gains (simplified to 6 DOFs)
        KpxLabel                  matlab.ui.control.Label
        KpxEditField              matlab.ui.control.NumericEditField
        KpyLabel                  matlab.ui.control.Label
        KpyEditField              matlab.ui.control.NumericEditField
        KpzLabel                  matlab.ui.control.Label
        KpzEditField              matlab.ui.control.NumericEditField
        KprollLabel               matlab.ui.control.Label
        KprollEditField           matlab.ui.control.NumericEditField
        KppitchLabel              matlab.ui.control.Label
        KppitchEditField          matlab.ui.control.NumericEditField
        KpyawLabel                matlab.ui.control.Label
        KpyawEditField            matlab.ui.control.NumericEditField
        KixLabel                  matlab.ui.control.Label
        KixEditField              matlab.ui.control.NumericEditField
        KiyLabel                  matlab.ui.control.Label
        KiyEditField              matlab.ui.control.NumericEditField
        KizLabel                  matlab.ui.control.Label
        KizEditField              matlab.ui.control.NumericEditField
        KirollLabel               matlab.ui.control.Label
        KirollEditField           matlab.ui.control.NumericEditField
        KipitchLabel              matlab.ui.control.Label
        KipitchEditField          matlab.ui.control.NumericEditField
        KiyawLabel                matlab.ui.control.Label
        KiyawEditField            matlab.ui.control.NumericEditField
        KdxLabel                  matlab.ui.control.Label
        KdxEditField              matlab.ui.control.NumericEditField
        KdyLabel                  matlab.ui.control.Label
        KdyEditField              matlab.ui.control.NumericEditField
        KdzLabel                  matlab.ui.control.Label
        KdzEditField              matlab.ui.control.NumericEditField
        KdrollLabel               matlab.ui.control.Label
        KdrollEditField           matlab.ui.control.NumericEditField
        KdpitchLabel              matlab.ui.control.Label
        KdpitchEditField          matlab.ui.control.NumericEditField
        KdyawLabel                matlab.ui.control.Label
        KdyawEditField            matlab.ui.control.NumericEditField
        % Simulation Settings
        TsLabel                   matlab.ui.control.Label
        TsEditField               matlab.ui.control.NumericEditField
        TfinalLabel               matlab.ui.control.Label
        TfinalEditField           matlab.ui.control.NumericEditField
        DesiredXLabel             matlab.ui.control.Label
        DesiredXEditField         matlab.ui.control.NumericEditField
        DesiredYLabel             matlab.ui.control.Label
        DesiredYEditField         matlab.ui.control.NumericEditField
        DesiredZLabel             matlab.ui.control.Label
        DesiredZEditField         matlab.ui.control.NumericEditField
        DesiredRollLabel          matlab.ui.control.Label
        DesiredRollEditField      matlab.ui.control.NumericEditField
        DesiredPitchLabel         matlab.ui.control.Label
        DesiredPitchEditField     matlab.ui.control.NumericEditField
        DesiredYawLabel           matlab.ui.control.Label
        DesiredYawEditField       matlab.ui.control.NumericEditField
        StatusLabel               matlab.ui.control.Label
    end

    % App-specific properties
    properties (Access = private)
        gFig    % 2D Plots Figure
        ax      % 2D Plot Axes
        hP      % Position Plot Handles
        hO      % Orientation Plot Handles
        hV      % Velocity Plot Handles
        hW      % Angular Rate Plot Handles
        hT      % Thruster Command Plot Handles
        gThr    % 3D Thruster View Figure
        axThr   % 3D Thruster Axes
        gBody   % 3D Trajectory Figure
        axBody  % 3D Trajectory Axes
        hTraj   % Trajectory Plot Handle
        g3D     % 3D Hull Figure
        ax3D    % 3D Hull Axes
        tr      % Hull Transform Object
    end

    methods (Access = private)

        % Startup function to initialize plots
        function startupFcn(app)
            % Load STL geometry
            scriptDir = fileparts(mfilename('fullpath'));
            stlFile = fullfile(scriptDir, 'Preliminary-ROV.stl');
            if ~isfile(stlFile)
                error('STL not found: %s', stlFile);
            end
            fv_full = stlread(stlFile);
            faces0 = fv_full.ConnectivityList;
            verts0_mm = fv_full.Points;
            [faces, verts_mm] = reducepatch(faces0, verts0_mm, 0.2);
            vertices = verts_mm * 1e-3; % mm to m

            % 2D Plots
            app.gFig = figure('Name', '2D Plots');
            app.ax(1) = subplot(5,1,1); 
            app.hP(1) = plot(app.ax(1), nan, nan, 'b'); hold(app.ax(1), 'on');
            app.hP(2) = plot(app.ax(1), nan, nan, 'r'); 
            app.hP(3) = plot(app.ax(1), nan, nan, 'g'); 
            title(app.ax(1), 'Position');
            app.ax(2) = subplot(5,1,2); 
            app.hO(1) = plot(app.ax(2), nan, nan, 'b'); hold(app.ax(2), 'on');
            app.hO(2) = plot(app.ax(2), nan, nan, 'r'); 
            app.hO(3) = plot(app.ax(2), nan, nan, 'g'); 
            title(app.ax(2), 'Orientation');
            app.ax(3) = subplot(5,1,3); 
            app.hV(1) = plot(app.ax(3), nan, nan, 'b'); hold(app.ax(3), 'on');
            app.hV(2) = plot(app.ax(3), nan, nan, 'r'); 
            app.hV(3) = plot(app.ax(3), nan, nan, 'g'); 
            title(app.ax(3), 'Velocities');
            app.ax(4) = subplot(5,1,4); 
            app.hW(1) = plot(app.ax(4), nan, nan, 'b'); hold(app.ax(4), 'on');
            app.hW(2) = plot(app.ax(4), nan, nan, 'r'); 
            app.hW(3) = plot(app.ax(4),ombra, 'off');
            title(app.ax(4), 'Angular Rates');
            app.ax(5) = subplot(5,1,5); 
            for i = 1:8
                app.hT(i) = plot(app.ax(5), nan, nan); hold(app.ax(5), 'on');
            end
            title(app.ax(5), 'Thruster Cmds');

            % 3D Thruster View
            app.gThr = figure('Name', '3D Thruster View');
            app.axThr = axes(app.gThr); 
            grid(app.axThr, 'on'); 
            axis(app.axThr, 'equal'); 
            hold(app.axThr, 'on');

            % 3D Trajectory
            app.gBody = figure('Name', 'Trajectory & Body');
            app.axBody = axes(app.gBody); 
            grid(app.axBody, 'on'); 
            axis(app.axBody, 'equal'); 
            hold(app.axBody, 'on');
            app.hTraj = plot3(app.axBody, nan, nan, nan, 'b--');
            scatter3(app.axBody, app.DesiredXEditField.Value, app.DesiredYEditField.Value, app.DesiredZEditField.Value, 'rx');

            % 3D Hull
            app.g3D = figure('Name', '3D Hull+Thrusters');
            app.ax3D = axes(app.g3D); 
            grid(app.ax3D, 'on'); 
            axis(app.ax3D, 'equal'); 
            hold(app.ax3D, 'on');
            app.tr = hgtransform('Parent', app.ax3D);
            patch('Faces', faces, 'Vertices', vertices, 'FaceColor', 'cyan', 'FaceAlpha', 0.3, 'Parent', app.tr);
        end

        % Run Simulation Button Callback
        function RunSimulationButtonPushed(app, event)
            app.StatusLabel.Text = 'Simulation running...';

            % Collect parameters
            mass = app.MassEditField.Value;
            Vdisp = app.VdispEditField.Value;
            com = [app.COMxEditField.Value; app.COMyEditField.Value; app.COMzEditField.Value];
            I_mat = diag([app.IxxEditField.Value, app.IyyEditField.Value, app.IzzEditField.Value]);
            added = diag([app.AddedMxEditField.Value, app.AddedMyEditField.Value, app.AddedMzEditField.Value, ...
                          app.AddedIxxEditField.Value, app.AddedIyyEditField.Value, app.AddedIzzEditField.Value]);
            CdA = [app.CdAxEditField.Value; app.CdAyEditField.Value; app.CdAzEditField.Value];
            D_rot = diag([app.DRollEditField.Value, app.DPitchEditField.Value, app.DYawEditField.Value]);
            Fb = 1000 * Vdisp * 9.81;
            W = mass * 9.81;

            % Mass matrix
            M = [mass*eye(3) + added(1:3,1:3), zeros(3); zeros(3), I_mat + added(4:6,4:6)];
            invM_lin = inv(M(1:3,1:3));
            invM_ang = inv(M(4:6,4:6));

            % Desired pose
            desired = [app.DesiredXEditField.Value; app.DesiredYEditField.Value; app.DesiredZEditField.Value; ...
                       app.DesiredRollEditField.Value; app.DesiredPitchEditField.Value; app.DesiredYawEditField.Value];

            % Thruster setup (fixed for now)
            r_pos = [0.2,0.1,0.1; 0.2,-0.1,0.1; -0.2,-0.1,0.1; -0.2,0.1,0.1;
                     0.2,0.1,0.0; 0.2,-0.1,0.0; -0.2,-0.1,0.0; -0.2,0.1,0.0];
            deg = [nan,nan,nan,nan,-45,45,135,-135];
            dir_thr = repmat([0,0,1],8,1);
            for i = 5:8
                dir_thr(i,1:2) = [cosd(deg(i)), sind(deg(i))];
            end
            A = zeros(6,8);
            for i = 1:8
                d = dir_thr(i,:)';
                r = r_pos(i,:)';
                A(1:3,i) = d;
                A(4:6,i) = cross(r,d);
            end

            % Controller and simulation setup
            ts = app.TsEditField.Value;
            t_final = app.TfinalEditField.Value;
            N = round(t_final / ts);
            t = (0:N-1) * ts;
            state = zeros(12, N);
            Kp = diag([app.KpxEditField.Value, app.KpyEditField.Value, app.KpzEditField.Value, ...
                       app.KprollEditField.Value, app.KppitchEditField.Value, app.KpyawEditField.Value]);
            Ki = diag([app.KixEditField.Value, app.KiyEditField.Value, app.KizEditField.Value, ...
                       app.KirollEditField.Value, app.KipitchEditField.Value, app.KiyawEditField.Value]);
            Kd = diag([app.KdxEditField.Value, app.KdyEditField.Value, app.KdzEditField.Value, ...
                       app.KdrollEditField.Value, app.KdpitchEditField.Value, app.KdyawEditField.Value]);
            intErr = zeros(6,1);
            prevErr = zeros(6,1);
            tmp = A'*A + 1e-3*eye(8);
            opts = optimoptions('quadprog', 'Display', 'off');
            Tmax = 60;
            tau_e = 0.05;
            tau_m = 0.15;
            alpha_e = ts/(tau_e+ts);
            alpha_m = ts/(tau_m+ts);
            thrust_e = zeros(8,1);
            thrust_act = zeros(8,1);
            thrustHist = zeros(8,N);
            arwScale = 0.1;

            % Simulation loop
            for k = 2:N
                % PID control
                err = desired - state(1:6,k-1);
                intErr = intErr + err*ts;
                dErr = (err - prevErr)/ts;
                prevErr = err;
                tau = Kp*err + Ki*intErr + Kd*dErr;

                % Thruster allocation
                tc = quadprog(tmp, -A'*tau, [], [], [], [], -Tmax*ones(8,1), Tmax*ones(8,1), [], opts);
                thrustHist(:,k) = tc;

                % Actuator dynamics
                thrust_e = alpha_e*tc + (1-alpha_e)*thrust_e;
                thrust_act = alpha_m*thrust_e + (1-alpha_m)*thrust_act;

                % Forces and moments
                TF = A*thrust_act;
                F = TF(1:3);
                M_t = TF(4:6) + cross(com, [0;0;(Fb-W)]);
                V = state(7:9,k-1);
                Fd = -CdA .* abs(V) .* V;
                F = F + Fd;

                % Dynamics
                acc_lin = invM_lin * F;
                acc_ang = invM_ang * (M_t - D_rot * state(10:12,k-1));
                state(7:9,k) = state(7:9,k-1) + acc_lin*ts;
                state(10:12,k) = state(10:12,k-1) + acc_ang*ts;
                state(1:3,k) = state(1:3,k-1) + state(7:9,k-1)*ts + 0.5*acc_lin*ts^2;
                state(4:6,k) = state(4:6,k-1) + state(10:12,k-1)*ts + 0.5*acc_ang*ts^2;

                % Visualization
                tvec = t(1:k);
                set(app.hP(1), 'XData', tvec, 'YData', state(1,1:k));
                set(app.hP(2), 'XData', tvec, 'YData', state(2,1:k));
                set(app.hP(3), 'XData', tvec, 'YData', state(3,1:k));
                set(app.hO(1), 'XData', tvec, 'YData', state(4,1:k));
                set(app.hO(2), 'XData', tvec, 'YData', state(5,1:k));
                set(app.hO(3), 'XData', tvec, 'YData', state(6,1:k));
                set(app.hV(1), 'XData', tvec, 'YData', state(7,1:k));
                set(app.hV(2), 'XData', tvec, 'YData', state(8,1:k));
                set(app.hV(3), 'XData', tvec, 'YData', state(9,1:k));
                set(app.hW(1), 'XData', tvec, 'YData', state(10,1:k));
                set(app.hW(2), 'XData', tvec, 'YData', state(11,1:k));
                set(app.hW(3), 'XData', tvec, 'YData', state(12,1:k));
                for i = 1:8
                    set(app.hT(i), 'XData', tvec, 'YData', thrustHist(i,1:k));
                end

                % 3D Thruster View
                delete(findall(app.axThr, 'Type', 'Quiver'));
                delete(findall(app.axThr, 'Type', 'Text'));
                pos = state(1:3,k);
                phi = state(4,k);
                th = state(5,k);
                psi = state(6,k);
                Rz = [cos(psi),-sin(psi),0; sin(psi),cos(psi),0; 0,0,1];
                Ry = [cos(th),0,sin(th); 0,1,0; -sin(th),0,cos(th)];
                Rx = [1,0,0; 0,cos(phi),-sin(phi); 0,sin(phi),cos(phi)];
                R = Rz*Ry*Rx;
                for i = 1:8
                    Pw = pos + R*r_pos(i,:)';
                    Vw = R*(dir_thr(i,:)'*thrust_act(i)*arwScale);
                    quiver3(app.axThr, Pw(1), Pw(2), Pw(3), Vw(1), Vw(2), Vw(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
                    text(app.axThr, Pw(1)+1.1*Vw(1), Pw(2)+1.1*Vw(2), Pw(3)+1.1*Vw(3), sprintf('%.1f', thrust_act(i)), 'FontWeight', 'bold');
                end

                % 3D Trajectory
                set(app.hTraj, 'XData', state(1,1:k), 'YData', state(2,1:k), 'ZData', state(3,1:k));

                % 3D Hull
                T4 = eye(4);
                T4(1:3,1:3) = R;
                T4(1:3,4) = pos;
                set(app.tr, 'Matrix', T4);

                drawnow limitrate;
            end
            app.StatusLabel.Text = 'Simulation completed.';
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 600 800];
            app.UIFigure.Name = 'ROV Simulation GUI';

            % Create RunSimulationButton
            app.RunSimulationButton = uibutton(app.UIFigure, 'push');
            app.RunSimulationButton.ButtonPushedFcn = createCallbackFcn(app, @RunSimulationButtonPushed, true);
            app.RunSimulationButton.Position = [250 50 100 22];
            app.RunSimulationButton.Text = 'Run Simulation';

            % Inertial Properties
            app.MassLabel = uilabel(app.UIFigure);
            app.MassLabel.Position = [20 750 100 22];
            app.MassLabel.Text = 'Mass (kg):';
            app.MassEditField = uieditfield(app.UIFigure, 'numeric');
            app.MassEditField.Position = [120 750 100 22];
            app.MassEditField.Value = 44.05438;

            app.VdispLabel = uilabel(app.UIFigure);
            app.VdispLabel.Position = [20 720 100 22];
            app.VdispLabel.Text = 'Vdisp (m^3):';
            app.VdispEditField = uieditfield(app.UIFigure, 'numeric');
            app.VdispEditField.Position = [120 720 100 22];
            app.VdispEditField.Value = 4.405437829e-2;

            app.COMxLabel = uilabel(app.UIFigure);
            app.COMxLabel.Position = [20 690 100 22];
            app.COMxLabel.Text = 'COM x (m):';
            app.COMxEditField = uieditfield(app.UIFigure, 'numeric');
            app.COMxEditField.Position = [120 690 100 22];
            app.COMxEditField.Value = -0.42049;

            app.COMyLabel = uilabel(app.UIFigure);
            app.COMyLabel.Position = [20 660 100 22];
            app.COMyLabel.Text = 'COM y (m):';
            app.COMyEditField = uieditfield(app.UIFigure, 'numeric');
            app.COMyEditField.Position = [120 660 100 22];
            app.COMyEditField.Value = 0.00072;

            app.COMzLabel = uilabel(app.UIFigure);
            app.COMzLabel.Position = [20 630 100 22];
            app.COMzLabel.Text = 'COM z (m):';
            app.COMzEditField = uieditfield(app.UIFigure, 'numeric');
            app.COMzEditField.Position = [120 630 100 22];
            app.COMzEditField.Value = -0.02683;

            app.IxxLabel = uilabel(app.UIFigure);
            app.IxxLabel.Position = [20 600 100 22];
            app.IxxLabel.Text = 'Ixx (kg·m^2):';
            app.IxxEditField = uieditfield(app.UIFigure, 'numeric');
            app.IxxEditField.Position = [120 600 100 22];
            app.IxxEditField.Value = 1.990993;

            app.IyyLabel = uilabel(app.UIFigure);
            app.IyyLabel.Position = [20 570 100 22];
            app.IyyLabel.Text = 'Iyy (kg·m^2):';
            app.IyyEditField = uieditfield(app.UIFigure, 'numeric');
            app.IyyEditField.Position = [120 570 100 22];
            app.IyyEditField.Value = 15.350344;

            app.IzzLabel = uilabel(app.UIFigure);
            app.IzzLabel.Position = [20 540 100 22];
            app.IzzLabel.Text = 'Izz (kg·m^2):';
            app.IzzEditField = uieditfield(app.UIFigure, 'numeric');
            app.IzzEditField.Position = [120 540 100 22];
            app.IzzEditField.Value = 15.774167;

            % Hydrodynamic Properties
            app.AddedMxLabel = uilabel(app.UIFigure);
            app.AddedMxLabel.Position = [300 750 100 22];
            app.AddedMxLabel.Text = 'Added m_x (kg):';
            app.AddedMxEditField = uieditfield(app.UIFigure, 'numeric');
            app.AddedMxEditField.Position = [400 750 100 22];
            app.AddedMxEditField.Value = 5;

            app.AddedMyLabel = uilabel(app.UIFigure);
            app.AddedMyLabel.Position = [300 720 100 22];
            app.AddedMyLabel.Text = 'Added m_y (kg):';
            app.AddedMyEditField = uieditfield(app.UIFigure, 'numeric');
            app.AddedMyEditField.Position = [400 720 100 22];
            app.AddedMyEditField.Value = 5;

            app.AddedMzLabel = uilabel(app.UIFigure);
            app.AddedMzLabel.Position = [300 690 100 22];
            app.AddedMzLabel.Text = 'Added m_z (kg):';
            app.AddedMzEditField = uieditfield(app.UIFigure, 'numeric');
            app.AddedMzEditField.Position = [400 690 100 22];
            app.AddedMzEditField.Value = 10;

            app.AddedIxxLabel = uilabel(app.UIFigure);
            app.AddedIxxLabel.Position = [300 660 100 22];
            app.AddedIxxLabel.Text = 'Added Ixx (kg·m^2):';
            app.AddedIxxEditField = uieditfield(app.UIFigure, 'numeric');
            app.AddedIxxEditField.Position = [400 660 100 22];
            app.AddedIxxEditField.Value = 1;

            app.AddedIyyLabel = uilabel(app.UIFigure);
            app.AddedIyyLabel.Position = [300 630 100 22];
            app.AddedIyyLabel.Text = 'Added Iyy (kg·m^2):';
            app.AddedIyyEditField = uieditfield(app.UIFigure, 'numeric');
            app.AddedIyyEditField.Position = [400 630 100 22];
            app.AddedIyyEditField.Value = 1;

            app.AddedIzzLabel = uilabel(app.UIFigure);
            app.AddedIzzLabel.Position = [300 600 100 22];
            app.AddedIzzLabel.Text = 'Added Izz (kg·m^2):';
            app.AddedIzzEditField = uieditfield(app.UIFigure, 'numeric');
            app.AddedIzzEditField.Position = [400 600 100 22];
            app.AddedIzzEditField.Value = 1;

            app.CdAxLabel = uilabel(app.UIFigure);
            app.CdAxLabel.Position = [300 570 100 22];
            app.CdAxLabel.Text = 'CdA_x (N/(m/s)^2):';
            app.CdAxEditField = uieditfield(app.UIFigure, 'numeric');
            app.CdAxEditField.Position = [400 570 100 22];
            app.CdAxEditField.Value = 0.5*1000*1.1*0.04;

            app.CdAyLabel = uilabel(app.UIFigure);
            app.CdAyLabel.Position = [300 540 100 22];
            app.CdAyLabel.Text = 'CdA_y (N/(m/s)^2):';
            app.CdAyEditField = uieditfield(app.UIFigure, 'numeric');
            app.CdAyEditField.Position = [400 540 100 22];
            app.CdAyEditField.Value = 0.5*1000*1.1*0.1;

            app.CdAzLabel = uilabel(app.UIFigure);
            app.CdAzLabel.Position = [300 510 100 22];
            app.CdAzLabel.Text = 'CdA_z (N/(m/s)^2):';
            app.CdAzEditField = uieditfield(app.UIFigure, 'numeric');
            app.CdAzEditField.Position = [400 510 100 22];
            app.CdAzEditField.Value = 0.5*1000*1.1*0.1;

            app.DRollLabel = uilabel(app.UIFigure);
            app.DRollLabel.Position = [300 480 100 22];
            app.DRollLabel.Text = 'D_roll (N·m/(rad/s)):';
            app.DRollEditField = uieditfield(app.UIFigure, 'numeric');
            app.DRollEditField.Position = [400 480 100 22];
            app.DRollEditField.Value = 5;

            app.DPitchLabel = uilabel(app.UIFigure);
            app.DPitchLabel.Position = [300 450 100 22];
            app.DPitchLabel.Text = 'D_pitch (N·m/(rad/s)):';
            app.DPitchEditField = uieditfield(app.UIFigure, 'numeric');
            app.DPitchEditField.Position = [400 450 100 22];
            app.DPitchEditField.Value = 5;

            app.DYawLabel = uilabel(app.UIFigure);
            app.DYawLabel.Position = [300 420 100 22];
            app.DYawLabel.Text = 'D_yaw (N·m/(rad/s)):';
            app.DYawEditField = uieditfield(app.UIFigure, 'numeric');
            app.DYawEditField.Position = [400 420 100 22];
            app.DYawEditField.Value = 5;

            % Controller Gains
            app.KpxLabel = uilabel(app.UIFigure);
            app.KpxLabel.Position = [20 510 100 22];
            app.KpxLabel.Text = 'Kp_x:';
            app.KpxEditField = uieditfield(app.UIFigure, 'numeric');
            app.KpxEditField.Position = [120 510 100 22];
            app.KpxEditField.Value = 4;

            app.KpyLabel = uilabel(app.UIFigure);
            app.KpyLabel.Position = [20 480 100 22];
            app.KpyLabel.Text = 'Kp_y:';
            app.KpyEditField = uieditfield(app.UIFigure, 'numeric');
            app.KpyEditField.Position = [120 480 100 22];
            app.KpyEditField.Value = 4;

            app.KpzLabel = uilabel(app.UIFigure);
            app.KpzLabel.Position = [20 450 100 22];
            app.KpzLabel.Text = 'Kp_z:';
            app.KpzEditField = uieditfield(app.UIFigure, 'numeric');
            app.KpzEditField.Position = [120 450 100 22];
            app.KpzEditField.Value = 4;

            app.KprollLabel = uilabel(app.UIFigure);
            app.KprollLabel.Position = [20 420 100 22];
            app.KprollLabel.Text = 'Kp_roll:';
            app.KprollEditField = uieditfield(app.UIFigure, 'numeric');
            app.KprollEditField.Position = [120 420 100 22];
            app.KprollEditField.Value = 0.8;

            app.KppitchLabel = uilabel(app.UIFigure);
            app.KppitchLabel.Position = [20 390 100 22];
            app.KppitchLabel.Text = 'Kp_pitch:';
            app.KppitchEditField = uieditfield(app.UIFigure, 'numeric');
            app.KppitchEditField.Position = [120 390 100 22];
            app.KppitchEditField.Value = 0.8;

            app.KpyawLabel = uilabel(app.UIFigure);
            app.KpyawLabel.Position = [20 360 100 22];
            app.KpyawLabel.Text = 'Kp_yaw:';
            app.KpyawEditField = uieditfield(app.UIFigure, 'numeric');
            app.KpyawEditField.Position = [120 360 100 22];
            app.KpyawEditField.Value = 1;

            app.KixLabel = uilabel(app.UIFigure);
            app.KixLabel.Position = [20 330 100 22];
            app.KixLabel.Text = 'Ki_x:';
            app.KixEditField = uieditfield(app.UIFigure, 'numeric');
            app.KixEditField.Position = [120 330 100 22];
            app.KixEditField.Value = 0.001;

            app.KiyLabel = uilabel(app.UIFigure);
            app.KiyLabel.Position = [20 300 100 22];
            app.KiyLabel.Text = 'Ki_y:';
            app.KiyEditField = uieditfield(app.UIFigure, 'numeric');
            app.KiyEditField.Position = [120 300 100 22];
            app.KiyEditField.Value = 0.001;

            app.KizLabel = uilabel(app.UIFigure);
            app.KizLabel.Position = [20 270 100 22];
            app.KizLabel.Text = 'Ki_z:';
            app.KizEditField = uieditfield(app.UIFigure, 'numeric');
            app.KizEditField.Position = [120 270 100 22];
            app.KizEditField.Value = 0.001;

            app.KirollLabel = uilabel(app.UIFigure);
            app.KirollLabel.Position = [20 240 100 22];
            app.KirollLabel.Text = 'Ki_roll:';
            app.KirollEditField = uieditfield(app.UIFigure, 'numeric');
            app.KirollEditField.Position = [120 240 100 22];
            app.KirollEditField.Value = 0;

            app.KipitchLabel = uilabel(app.UIFigure);
            app.KipitchLabel.Position = [20 210 100 22];
            app.KipitchLabel.Text = 'Ki_pitch:';
            app.KipitchEditField = uieditfield(app.UIFigure, 'numeric');
            app.KipitchEditField.Position = [120 210 100 22];
            app.KipitchEditField.Value = 0;

            app.KiyawLabel = uilabel(app.UIFigure);
            app.KiyawLabel.Position = [20 180 100 22];
            app.KiyawLabel.Text = 'Ki_yaw:';
            app.KiyawEditField = uieditfield(app.UIFigure, 'numeric');
            app.KiyawEditField.Position = [120 180 100 22];
            app.KiyawEditField.Value = 0;

            app.KdxLabel = uilabel(app.UIFigure);
            app.KdxLabel.Position = [300 390 100 22];
            app.KdxLabel.Text = 'Kd_x:';
            app.KdxEditField = uieditfield(app.UIFigure, 'numeric');
            app.KdxEditField.Position = [400 390 100 22];
            app.KdxEditField.Value = 0.6;

            app.KdyLabel = uilabel(app.UIFigure);
            app.KdyLabel.Position = [300 360 100 22];
            app.KdyLabel.Text = 'Kd_y:';
            app.KdyEditField = uieditfield(app.UIFigure, 'numeric');
            app.KdyEditField.Position = [400 360 100 22];
            app.KdyEditField.Value = 0.6;

            app.KdzLabel = uilabel(app.UIFigure);
            app.KdzLabel.Position = [300 330 100 22];
            app.KdzLabel.Text = 'Kd_z:';
            app.KdzEditField = uieditfield(app.UIFigure, 'numeric');
            app.KdzEditField.Position = [400 330 100 22];
            app.KdzEditField.Value = 0.6;

            app.KdrollLabel = uilabel(app.UIFigure);
            app.KdrollLabel.Position = [300 300 100 22];
            app.KdrollLabel.Text = 'Kd_roll:';
            app.KdrollEditField = uieditfield(app.UIFigure, 'numeric');
            app.KdrollEditField.Position = [400 300 100 22];
            app.KdrollEditField.Value = 1.5;

            app.KdpitchLabel = uilabel(app.UIFigure);
            app.KdpitchLabel.Position = [300 270 100 22];
            app.KdpitchLabel.Text = 'Kd_pitch:';
            app.KdpitchEditField = uieditfield(app.UIFigure, 'numeric');
            app.KdpitchEditField.Position = [400 270 100 22];
            app.KdpitchEditField.Value = 1.5;

            app.KdyawLabel = uilabel(app.UIFigure);
            app.KdyawLabel.Position = [300 240 100 22];
            app.KdyawLabel.Text = 'Kd_yaw:';
            app.KdyawEditField = uieditfield(app.UIFigure, 'numeric');
            app.KdyawEditField.Position = [400 240 100 22];
            app.KdyawEditField.Value = 2;

            % Simulation Settings
            app.TsLabel = uilabel(app.UIFigure);
            app.TsLabel.Position = [20 150 100 22];
            app.TsLabel.Text = 'Time step (s):';
            app.TsEditField = uieditfield(app.UIFigure, 'numeric');
            app.TsEditField.Position = [120 150 100 22];
            app.TsEditField.Value = 0.1;

            app.TfinalLabel = uilabel(app.UIFigure);
            app.TfinalLabel.Position = [20 120 100 22];
            app.TfinalLabel.Text = 'Final time (s):';
            app.TfinalEditField = uieditfield(app.UIFigure, 'numeric');
            app.TfinalEditField.Position = [120 120 100 22];
            app.TfinalEditField.Value = 120;

            app.DesiredXLabel = uilabel(app.UIFigure);
            app.DesiredXLabel.Position = [20 90 100 22];
            app.DesiredXLabel.Text = 'Desired x (m):';
            app.DesiredXEditField = uieditfield(app.UIFigure, 'numeric');
            app.DesiredXEditField.Position = [120 90 100 22];
            app.DesiredXEditField.Value = 20;

            app.DesiredYLabel = uilabel(app.UIFigure);
            app.DesiredYLabel.Position = [20 60 100 22];
            app.DesiredYLabel.Text = 'Desired y (m):';
            app.DesiredYEditField = uieditfield(app.UIFigure, 'numeric');
            app.DesiredYEditField.Position = [120 60 100 22];
            app.DesiredYEditField.Value = 20;

            app.DesiredZLabel = uilabel(app.UIFigure);
            app.DesiredZLabel.Position = [20 30 100 22];
            app.DesiredZLabel.Text = 'Desired z (m):';
            app.DesiredZEditField = uieditfield(app.UIFigure, 'numeric');
            app.DesiredZEditField.Position = [120 30 100 22];
            app.DesiredZEditField.Value = 20;

            app.DesiredRollLabel = uilabel(app.UIFigure);
            app.DesiredRollLabel.Position = [300 150 100 22];
            app.DesiredRollLabel.Text = 'Desired roll (deg):';
            app.DesiredRollEditField = uieditfield(app.UIFigure, 'numeric');
            app.DesiredRollEditField.Position = [400 150 100 22];
            app.DesiredRollEditField.Value = 0;

            app.DesiredPitchLabel = uilabel(app.UIFigure);
            app.DesiredPitchLabel.Position = [300 120 100 22];
            app.DesiredPitchLabel.Text = 'Desired pitch (deg):';
            app.DesiredPitchEditField = uieditfield(app.UIFigure, 'numeric');
            app.DesiredPitchEditField.Position = [400 120 100 22];
            app.DesiredPitchEditField.Value = 0;

            app.DesiredYawLabel = uilabel(app.UIFigure);
            app.DesiredYawLabel.Position = [300 90 100 22];
            app.DesiredYawLabel.Text = 'Desired yaw (deg):';
            app.DesiredYawEditField = uieditfield(app.UIFigure, 'numeric');
            app.DesiredYawEditField.Position = [400 90 100 22];
            app.DesiredYawEditField.Value = 0;

            % Status Label
            app.StatusLabel = uilabel(app.UIFigure);
            app.StatusLabel.Position = [250 20 100 22];
            app.StatusLabel.Text = '';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = ROVSimGUI2
            % Run startup function
            startupFcn(app)

            % Create components
            createComponents(app)
        end

        % Code that executes before app deletion
        function delete(app)
            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
            delete(app.gFig)
            delete(app.gThr)
            delete(app.gBody)
            delete(app.g3D)
        end
    end
end