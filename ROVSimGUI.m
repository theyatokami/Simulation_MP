classdef ROVSimGUI < matlab.apps.AppBase
    % ROVSimApp: Full-featured GUI for 6-DOF ROV Simulation with ANSYS Integration and Visualizations

    properties (Access = public)
        UIFigure                 matlab.ui.Figure
        LoadSTLButton            matlab.ui.control.Button
        LoadANSYSButton          matlab.ui.control.Button
        RunSimButton             matlab.ui.control.Button
        ResetButton              matlab.ui.control.Button
        MassEdit                 matlab.ui.control.NumericEditField
        VdispEdit                matlab.ui.control.NumericEditField
        COMTable                 matlab.ui.control.Table
        InertiaTable             matlab.ui.control.Table
        AddedMassTable           matlab.ui.control.Table
        CdATable                 matlab.ui.control.Table
        DrotTable                matlab.ui.control.Table
        ThrusterConfigTable      matlab.ui.control.Table
        GainsTable               matlab.ui.control.Table
        UIAxes2D                 matlab.ui.control.UIAxes
        UIAxesThruster3D         matlab.ui.control.UIAxes
        UIAxesHull3D             matlab.ui.control.UIAxes
        UIAxesTrajectory3D       matlab.ui.control.UIAxes
    end

    properties (Access = private)
        simParams                % Simulation parameters struct
        mesh                     % geometry mesh (faces, vertices)
        timeVec                  % simulation time vector
        stateHistory             % state history (12 x N)
        thrustHistory            % thrust history (8 x N)
    end

    methods (Access = private)
        function startup(app)
            % Initialize default parameters
            p = struct();
            p.mass = 44.05438;
            p.Vdisp = 4.4054e-2;
            p.com = [-0.42049, 0.00072, -0.02683];
            p.I = diag([1.990993, 15.350344, 15.774167]);
            p.added = diag([5, 5, 10, 1, 1, 1]);
            p.CdA = [0.5 * 1000 * 1.1 * 0.04; 0.5 * 1000 * 1.1 * 0.1; 0.5 * 1000 * 1.1 * 0.1];
            p.D_rot = diag([5, 5, 5]);
            p.Fb = 1000 * p.Vdisp * 9.81;
            r = [0.2, 0.1, 0.1; 0.2, -0.1, 0.1; -0.2, -0.1, 0.1; -0.2, 0.1, 0.1;
                 0.2, 0.1, 0; 0.2, -0.1, 0; -0.2, -0.1, 0; -0.2, 0.1, 0];
            dirs = repmat([0, 0, 1], 4, 1);
            dirs = [dirs; cosd([-45 45 135 -135])' sind([-45 45 135 -135])' zeros(4, 1)];
            p.r_pos = r;
            p.dir_thr = dirs;
            p.Kp = diag([4, 4, 4, 0.8, 0.8, 1]);
            p.Ki = diag([0.001, 0.001, 0.001, 0, 0, 0]);
            p.Kd = diag([0.6, 0.6, 0.6, 1.5, 1.5, 2]);
            p.ts = 0.1;
            p.t_final = 120;
            app.simParams = p;
            % Populate UI tables and fields
            app.MassEdit.Value = p.mass;
            app.VdispEdit.Value = p.Vdisp;
            app.COMTable.Data = p.com;
            app.InertiaTable.Data = diag(p.I)';
            app.AddedMassTable.Data = diag(p.added)';
            app.CdATable.Data = p.CdA';
            app.DrotTable.Data = diag(p.D_rot)';
            thrData = [p.r_pos, p.dir_thr];
            app.ThrusterConfigTable.Data = thrData;
            gainsData = [diag(p.Kp), diag(p.Ki), diag(p.Kd)];
            app.GainsTable.Data = gainsData;
        end

        function LoadSTL(app, ~, ~)
            % Load STL, decimate, and plot hull
            [file, path] = uigetfile('*.stl');
            if isequal(file, 0), return; end
            fv = stlread(fullfile(path, file));
            [f, v] = reducepatch(fv.ConnectivityList, fv.Points, 0.2);
            v = v * 1e-3;
            app.mesh.faces = f;
            app.mesh.vertices = v;
            % Plot in Hull axes
            cla(app.UIAxesHull3D);
            patch('Faces', f, 'Vertices', v, 'FaceColor', 'cyan', 'FaceAlpha', 0.3, 'Parent', app.UIAxesHull3D);
            axis(app.UIAxesHull3D, 'equal'); grid(app.UIAxesHull3D, 'on');
        end

        function LoadANSYS(app, ~, ~)
            % Load parameters from ANSYS .mat
            [file, path] = uigetfile('*.mat');
            if isequal(file, 0), return; end
            data = load(fullfile(path, file));
            fn = fieldnames(data);
            for k = 1:length(fn)
                if isfield(app.simParams, fn{k})
                    app.simParams.(fn{k}) = data.(fn{k});
                end
            end
            startup(app);
        end

        function RunSim(app, ~, ~)
            % Execute full simulation loop and update all visualizations
            p = app.simParams;
            N = round(p.t_final / p.ts);
            t = (0:N-1) * p.ts;
            state = zeros(12, N);
            thrust = zeros(8, N);
        
            % Precompute mass matrices and allocation
            M = [p.mass * eye(3) + p.added(1:3,1:3), zeros(3);
                 zeros(3), p.I + p.added(4:6,4:6)];
            invM_lin = inv(M(1:3,1:3));
            invM_ang = inv(M(4:6,4:6));
            A = zeros(6, 8);
            for i = 1:8
                d = p.dir_thr(i, :)';
                r = p.r_pos(i, :)';
                A(1:3, i) = d;
                A(4:6, i) = cross(r, d);
            end
            Q = A' * A + 1e-3 * eye(8);
            opts = optimoptions('quadprog', 'Display', 'off');
        
            % Initialize simulation variables before the loop
            intErr = zeros(6,1);    % Integral error for PID control
            prevErr = zeros(6,1);   % Previous error for derivative term
            thr_e = zeros(8,1);     % Expected thrust (actuator lag)
            thr_a = zeros(8,1);     % Actual thrust (actuator lag)
        
            % Simulation loop
            for k = 2:N
                err = [20;20;20;0;0;0] - state(1:6,k-1);
                intErr = intErr + err * p.ts;
                dErr = (err - prevErr) / p.ts;
                prevErr = err;
                tau = p.Kp * err + p.Ki * intErr + p.Kd * dErr;
                tc = quadprog(Q, -A'*tau, [],[],[],[], -60*ones(8,1), 60*ones(8,1), [], opts);
                thrust(:,k) = tc;
                % Actuator lag
                thr_e = p.ts / (p.ts + 0.05) * tc + (0.05 / (p.ts + 0.05)) * thr_e;
                thr_a = p.ts / (p.ts + 0.15) * thr_e + (0.15 / (p.ts + 0.15)) * thr_a;
                TF = A * thr_a;
                F = TF(1:3) - p.CdA .* abs(state(7:9,k-1)) .* state(7:9,k-1);
                M_t = TF(4:6) + cross(p.com', [0;0;(p.Fb - p.mass*9.81)]);
                acc_lin = invM_lin * F;
                acc_ang = invM_ang * (M_t - p.D_rot * state(10:12,k-1));
                state(7:9,k) = state(7:9,k-1) + acc_lin * p.ts;
                state(10:12,k) = state(10:12,k-1) + acc_ang * p.ts;
                state(1:3,k) = state(1:3,k-1) + state(7:9,k-1) * p.ts + 0.5 * acc_lin * p.ts^2;
                state(4:6,k) = state(4:6,k-1) + state(10:12,k-1) * p.ts + 0.5 * acc_ang * p.ts^2;
            end
        
            % Store results and update plots
            app.timeVec = t;
            app.stateHistory = state;
            app.thrustHistory = thrust;
            % Update plots
            % 2D plots in a single UIAxes
            cla(app.UIAxes2D);
            hold(app.UIAxes2D, 'on');
            plot(app.UIAxes2D, t, state(1, :), '-r', 'DisplayName', 'X');
            plot(app.UIAxes2D, t, state(2, :), '-g', 'DisplayName', 'Y');
            plot(app.UIAxes2D, t, state(3, :), '-b', 'DisplayName', 'Z');
            plot(app.UIAxes2D, t, state(4, :), '-m', 'DisplayName', 'Roll');
            plot(app.UIAxes2D, t, state(5, :), '-c', 'DisplayName', 'Pitch');
            plot(app.UIAxes2D, t, state(6, :), '-k', 'DisplayName', 'Yaw');
            plot(app.UIAxes2D, t, state(7, :), '--r', 'DisplayName', 'Vx');
            plot(app.UIAxes2D, t, state(8, :), '--g', 'DisplayName', 'Vy');
            plot(app.UIAxes2D, t, state(9, :), '--b', 'DisplayName', 'Vz');
            plot(app.UIAxes2D, t, state(10, :), '--m', 'DisplayName', 'P');
            plot(app.UIAxes2D, t, state(11, :), '--c', 'DisplayName', 'Q');
            plot(app.UIAxes2D, t, state(12, :), '--k', 'DisplayName', 'R');
            hold(app.UIAxes2D, 'off');
            legend(app.UIAxes2D, 'Location', 'best');
            title(app.UIAxes2D, 'Simulation Results');
            % 3D Thruster view
            cla(app.UIAxesThruster3D);
            hold(app.UIAxesThruster3D, 'on'); grid(app.UIAxesThruster3D, 'on'); axis(app.UIAxesThruster3D, 'equal');
            arwScale = 0.1;
            R = eye(3); % Assuming no rotation for simplicity
            for i = 1:8
                pos = state(1:3, end) + R * p.r_pos(i, :)';
                vec = R * (p.dir_thr(i, :)' * thrust(i, end) * arwScale);
                quiver3(app.UIAxesThruster3D, pos(1), pos(2), pos(3), vec(1), vec(2), vec(3), 'r');
            end
            hold(app.UIAxesThruster3D, 'off');
            % 3D Hull at final pose
            cla(app.UIAxesHull3D);
            T = eye(4); T(1:3, 4) = state(1:3, end);
            hg = hgtransform('Parent', app.UIAxesHull3D);
            patch('Faces', app.mesh.faces, 'Vertices', app.mesh.vertices, 'FaceColor', 'cyan', 'FaceAlpha', 0.3, 'Parent', hg);
            hg.Matrix = T;
            axis(app.UIAxesHull3D, 'equal'); grid(app.UIAxesHull3D, 'on');
            % 3D Trajectory
            cla(app.UIAxesTrajectory3D);
            plot3(app.UIAxesTrajectory3D, state(1, :), state(2, :), state(3, :), '-b'); hold(app.UIAxesTrajectory3D, 'on');
            plot3(app.UIAxesTrajectory3D, 20, 20, 20, 'rx', 'MarkerSize', 10);
            axis(app.UIAxesTrajectory3D, 'equal'); grid(app.UIAxesTrajectory3D, 'on'); hold(app.UIAxesTrajectory3D, 'off');
        end

        function Reset(app, ~, ~)
            cla(app.UIAxes2D); cla(app.UIAxesThruster3D);
            cla(app.UIAxesHull3D); cla(app.UIAxesTrajectory3D);
            startup(app);
        end
    end

    methods (Access = private)
        function createComponents(app)
            app.UIFigure = uifigure('Name', 'ROV Simulation Platform', 'Position', [100 100 1400 900]);
            % Control Buttons
            app.LoadSTLButton = uibutton(app.UIFigure, 'Text', 'Load STL', 'Position', [20 860 100 30], 'ButtonPushedFcn', @(s, e) app.LoadSTL(s, e));
            app.LoadANSYSButton = uibutton(app.UIFigure, 'Text', 'Load ANSYS', 'Position', [140 860 100 30], 'ButtonPushedFcn', @(s, e) app.LoadANSYS(s, e));
            app.RunSimButton = uibutton(app.UIFigure, 'Text', 'Run Sim', 'Position', [260 860 100 30], 'ButtonPushedFcn', @(s, e) app.RunSim(s, e));
            app.ResetButton = uibutton(app.UIFigure, 'Text', 'Reset', 'Position', [380 860 100 30], 'ButtonPushedFcn', @(s, e) app.Reset(s, e));
            % Inputs
            app.MassEdit = uieditfield(app.UIFigure, 'numeric', 'Position', [20 820 100 22], 'LowerLimit', 0);
            app.VdispEdit = uieditfield(app.UIFigure, 'numeric', 'Position', [140 820 100 22], 'LowerLimit', 0);
            app.COMTable = uitable(app.UIFigure, 'Position', [20 690 260 120], 'ColumnName', {'COM X', 'COM Y', 'COM Z'});
            app.InertiaTable = uitable(app.UIFigure, 'Position', [20 550 260 120], 'ColumnName', {'Ixx', 'Iyy', 'Izz'});
            app.AddedMassTable = uitable(app.UIFigure, 'Position', [20 410 260 120], 'ColumnName', {'Am11', 'Am22', 'Am33', 'Am44', 'Am55', 'Am66'});
            app.CdATable = uitable(app.UIFigure, 'Position', [20 350 260 80], 'ColumnName', {'CdAx', 'CdAy', 'CdAz'});
            app.DrotTable = uitable(app.UIFigure, 'Position', [20 230 260 120], 'ColumnName', {'Dr11', 'Dr22', 'Dr33'});
            app.ThrusterConfigTable = uitable(app.UIFigure, 'Position', [300 550 420 260], 'ColumnName', {'rX', 'rY', 'rZ', 'dX', 'dY', 'dZ'});
            app.GainsTable = uitable(app.UIFigure, 'Position', [300 350 420 180], 'ColumnName', {'Kp', 'Ki', 'Kd'});
            % Axes Layout
            app.UIAxes2D = uiaxes(app.UIFigure, 'Position', [750 520 620 340], 'Box', 'on');
            title(app.UIAxes2D, '2D Plots');
            app.UIAxesThruster3D = uiaxes(app.UIFigure, 'Position', [20 20 420 340], 'Box', 'on');
            title(app.UIAxesThruster3D, '3D Thruster Forces');
            app.UIAxesHull3D = uiaxes(app.UIFigure, 'Position', [460 20 420 340], 'Box', 'on');
            title(app.UIAxesHull3D, 'Hull & Orientation');
            app.UIAxesTrajectory3D = uiaxes(app.UIFigure, 'Position', [900 20 420 340], 'Box', 'on');
            title(app.UIAxesTrajectory3D, 'Trajectory');
        end
    end

    methods (Access = public)
        function app = ROVSimGUI
            createComponents(app);
            startup(app);
        end
        function delete(app)
            delete(app.UIFigure);
        end
    end
end