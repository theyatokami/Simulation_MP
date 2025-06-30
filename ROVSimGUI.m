classdef ROVSimGUI < matlab.apps.AppBase
    % ROVSimGUI: Full-featured GUI for 6-DOF ROV Simulation with ANSYS Integration and Visualizations

    properties (Access = public)
        UIFigure                 matlab.ui.Figure
        TabGroup                 matlab.ui.container.TabGroup
        ParametersTab            matlab.ui.container.Tab
        SimulationTab            matlab.ui.container.Tab
        VisualizationTab         matlab.ui.container.Tab
        ResultsTab               matlab.ui.container.Tab
        
        % Control buttons
        LoadANSYSButton          matlab.ui.control.Button
        RunSimButton             matlab.ui.control.Button
        ResetButton              matlab.ui.control.Button
        
        % Parameters Tab Controls
        BasicPropsPanel          matlab.ui.container.Panel
        MassLabel                matlab.ui.control.Label
        MassEdit                 matlab.ui.control.NumericEditField
        VdispLabel               matlab.ui.control.Label
        VdispEdit                matlab.ui.control.NumericEditField
        
        COMPanel                 matlab.ui.container.Panel
        COMTable                 matlab.ui.control.Table
        
        InertiaPanel             matlab.ui.container.Panel
        InertiaTable             matlab.ui.control.Table
        
        AddedMassPanel           matlab.ui.container.Panel
        AddedMassTable           matlab.ui.control.Table
        
        DragPanel                matlab.ui.container.Panel
        CdATable                 matlab.ui.control.Table
        DrotTable                matlab.ui.control.Table
        
        ThrusterPanel            matlab.ui.container.Panel
        ThrusterConfigTable      matlab.ui.control.Table
        
        ControlPanel             matlab.ui.container.Panel
        GainsTable               matlab.ui.control.Table
        DesiredPoseTable         matlab.ui.control.Table
        
        % Simulation Tab Controls
        UIAxes2D                 matlab.ui.control.UIAxes
        
        % Visualization Tab Controls
        UIAxesThruster3D         matlab.ui.control.UIAxes
        UIAxesTrajectory3D       matlab.ui.control.UIAxes
        UIAxesHull3D             matlab.ui.control.UIAxes

        % Results Tab Controls
        UIAxesError              matlab.ui.control.UIAxes
        UIAxesControlEffort      matlab.ui.control.UIAxes
        ResultsTextArea          matlab.ui.control.TextArea
    end

    properties (Access = private)
        simParams                % Simulation parameters struct
        timeVec                  % Simulation time vector
        stateHistory             % State history (12 x N)
        thrustHistory            % Thrust history (8 x N)
        faces                    % STL faces
        vertices                 % STL vertices
        tr                       % Hull transform object
        errorHistory             % Error history (6 x N)
    end

    methods (Access = private)
        function startup(app)
            % Load STL geometry
            scriptDir = fileparts(mfilename('fullpath'));
            stlFile = fullfile(scriptDir, 'Preliminary-ROV.stl');
            if ~isfile(stlFile)
                warning('Geometric model not found: %s. Using default geometry.', stlFile);
                % Create a simple box geometry as fallback
                [x, y, z] = meshgrid([-0.5, 0.5], [-0.2, 0.2], [-0.1, 0.1]);
                app.vertices = [x(:), y(:), z(:)];
                app.faces = convhull(app.vertices);
            else
                fv_full = stlread(stlFile);
                [app.faces, verts_mm] = reducepatch(fv_full.ConnectivityList, fv_full.Points, 0.2);
                app.vertices = verts_mm * 1e-3; % mm to m
            end

            % Initialize default parameters
            p = struct();
            p.mass = 44.05438;
            p.Vdisp = 4.4054e-2;
            p.com = [-0.42049; 0.00072; -0.02683];
            p.I = diag([1.990993, 15.350344, 15.774167]);
            p.added = diag([5, 5, 10, 1, 1, 1]);
            p.CdA = [0.5 * 1000 * 1.1 * 0.04; 0.5 * 1000 * 1.1 * 0.1; 0.5 * 1000 * 1.1 * 0.1];
            p.D_rot = diag([5, 5, 5]);
            p.Fb = 1000 * p.Vdisp * 9.81;
            p.r_pos = [0.2, 0.1, 0.1; 0.2, -0.1, 0.1; -0.2, -0.1, 0.1; -0.2, 0.1, 0.1;
                       0.2, 0.1, 0; 0.2, -0.1, 0; -0.2, -0.1, 0; -0.2, 0.1, 0];
            deg = [nan, nan, nan, nan, -45, 45, 135, -135];
            p.dir_thr = [repmat([0, 0, 1], 4, 1); [cosd(deg(5:8))', sind(deg(5:8))', zeros(4, 1)]];
            p.Kp = diag([4, 4, 4, 0.8, 0.8, 1]);
            p.Ki = diag([0.001, 0.001, 0.001, 0, 0, 0]);
            p.Kd = diag([0.6, 0.6, 0.6, 1.5, 1.5, 2]);
            p.ts = 0.1;
            p.t_final = 120;
            p.desired = [20; 20; 20; 0; 0; 0]; % Desired pose [x, y, z, roll, pitch, yaw]
            app.simParams = p;

            % Populate UI tables and fields
            app.MassEdit.Value = p.mass;
            app.VdispEdit.Value = p.Vdisp;
            app.COMTable.Data = p.com';
            app.InertiaTable.Data = diag(p.I)';
            app.AddedMassTable.Data = diag(p.added)';
            app.CdATable.Data = p.CdA';
            app.DrotTable.Data = diag(p.D_rot)';
            app.ThrusterConfigTable.Data = [p.r_pos, p.dir_thr];
            app.GainsTable.Data = [diag(p.Kp), diag(p.Ki), diag(p.Kd)];
            app.DesiredPoseTable.Data = p.desired';
        end

        function LoadANSYS(app, ~, ~)
            % Load parameters from ANSYS .mat
            [file, path] = uigetfile('*.mat', 'Select ANSYS Results File');
            if isequal(file, 0), return; end
            data = load(fullfile(path, file));
            fn = fieldnames(data);
            for k = 1:length(fn)
                if isfield(app.simParams, fn{k})
                    app.simParams.(fn{k}) = data.(fn{k});
                end
            end
            app.startup();
        end

        function RunSim(app, ~, ~)
            % Update simParams from GUI inputs
            try
                app.simParams.mass = app.MassEdit.Value;
                app.simParams.Vdisp = app.VdispEdit.Value;
                app.simParams.com = app.COMTable.Data';
                app.simParams.I = diag(app.InertiaTable.Data);
                app.simParams.added = diag(app.AddedMassTable.Data);
                app.simParams.CdA = app.CdATable.Data';
                app.simParams.D_rot = diag(app.DrotTable.Data);
                app.simParams.r_pos = app.ThrusterConfigTable.Data(:, 1:3);
                app.simParams.dir_thr = app.ThrusterConfigTable.Data(:, 4:6);
                app.simParams.Kp = diag(app.GainsTable.Data(:, 1));
                app.simParams.Ki = diag(app.GainsTable.Data(:, 2));
                app.simParams.Kd = diag(app.GainsTable.Data(:, 3));
                app.simParams.desired = app.DesiredPoseTable.Data';
            catch ME
                uialert(app.UIFigure, sprintf('Invalid input: %s', ME.message), 'Input Error');
                return;
            end

            % Switch to simulation tab
            app.TabGroup.SelectedTab = app.SimulationTab;

            p = app.simParams;
            N = round(p.t_final / p.ts);
            app.timeVec = (0:N-1) * p.ts;
            app.stateHistory = zeros(12, N);
            app.thrustHistory = zeros(8, N);
            app.errorHistory = zeros(6, N);

            % Precompute mass matrices and allocation
            try
                M = [p.mass * eye(3) + p.added(1:3,1:3), zeros(3); zeros(3), p.I + p.added(4:6,4:6)];
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
            catch ME
                uialert(app.UIFigure, sprintf('Error setting up dynamics: %s', ME.message), 'Setup Error');
                return;
            end

            % Initialize graphics for simulation tab
            ax2 = app.UIAxes2D; cla(ax2); hold(ax2, 'on');
            coords = {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'};
            cols = {'r', 'g', 'b', 'm', 'c', 'k'};
            hPos = struct();
            for i = 1:6
                hPos.(coords{i}) = animatedline(ax2, 'Color', cols{i}, 'DisplayName', coords{i});
            end
            vcoords = {'Vx', 'Vy', 'Vz'};
            vcols = {'r', 'g', 'b'};
            hVel = struct();
            for i = 1:3
                hVel.(vcoords{i}) = animatedline(ax2, 'LineStyle', '--', 'Color', vcols{i}, 'DisplayName', vcoords{i});
            end
            acoords = {'P', 'Q', 'R'};
            acols = {'m', 'c', 'k'};
            hAng = struct();
            for i = 1:3
                hAng.(acoords{i}) = animatedline(ax2, 'LineStyle', '--', 'Color', acols{i}, 'DisplayName', acoords{i});
            end
            title(ax2, 'Simulation Results'); legend(ax2, 'show'); hold(ax2, 'off');

            % Initialize graphics for visualization tab
            axT = app.UIAxesThruster3D; cla(axT); hold(axT, 'on');
            quiv = gobjects(8, 1);
            for i = 1:8
                quiv(i) = quiver3(axT, 0, 0, 0, 0, 0, 0, 'r', 'MaxHeadSize', 0.5, 'DisplayName', sprintf('Thruster %d', i));
            end
            grid(axT, 'on'); axis(axT, 'equal'); title(axT, '3D Thruster Forces'); hold(axT, 'off');

            axTraj = app.UIAxesTrajectory3D; cla(axTraj); hold(axTraj, 'on');
            hTraj = animatedline(axTraj, 'Color', 'b', 'LineWidth', 1.5, 'DisplayName', 'Trajectory');
            plot3(axTraj, p.desired(1), p.desired(2), p.desired(3), 'rx', 'MarkerSize', 10, 'DisplayName', 'Desired Pose');
            grid(axTraj, 'on'); axis(axTraj, 'equal'); title(axTraj, 'Trajectory'); legend(axTraj, 'show'); hold(axTraj, 'off');

            axHull = app.UIAxesHull3D; cla(axHull); hold(axHull, 'on');
            app.tr = hgtransform('Parent', axHull);
            patch('Faces', app.faces, 'Vertices', app.vertices, 'FaceColor', 'cyan', 'FaceAlpha', 0.3, 'Parent', app.tr, 'DisplayName', 'ROV Hull');
            grid(axHull, 'on'); axis(axHull, 'equal'); title(axHull, '3D Hull'); legend(axHull, 'show'); hold(axHull, 'off');

            % Initialize state & control variables
            state = zeros(12, 1);
            intErr = zeros(6, 1);
            prevErr = p.desired; % Initialize with initial error
            thr_e = zeros(8, 1);
            thr_a = zeros(8, 1);
            Tmax = 60;
            tau_e = 0.05;
            tau_m = 0.15;
            alpha_e = p.ts / (tau_e + p.ts);
            alpha_m = p.ts / (tau_m + p.ts);
            arwScale = 0.1;

            % Store initial error
            app.errorHistory(:, 1) = p.desired - state(1:6);

            % Simulation loop
            tic;
            try
                for k = 2:N
                    % PID control
                    err = p.desired - state(1:6);
                    app.errorHistory(:, k) = err;
                    intErr = intErr + err * p.ts;
                    dErr = (err - prevErr) / p.ts;
                    prevErr = err;
                    tau = p.Kp * err + p.Ki * intErr + p.Kd * dErr;
                    tc = quadprog(Q, -A' * tau, [], [], [], [], -Tmax * ones(8, 1), Tmax * ones(8, 1), [], opts);
                    app.thrustHistory(:, k) = tc;

                    % Actuator dynamics
                    thr_e = alpha_e * tc + (1 - alpha_e) * thr_e;
                    thr_a = alpha_m * thr_e + (1 - alpha_m) * thr_a;

                    % Forces and moments
                    TF = A * thr_a;
                    F = TF(1:3) - p.CdA .* abs(state(7:9)) .* state(7:9);
                    M_t = TF(4:6) + cross(p.com, [0; 0; (p.Fb - p.mass * 9.81)]);

                    % Dynamics
                    acc_lin = invM_lin * F;
                    acc_ang = invM_ang * (M_t - p.D_rot * state(10:12));
                    state(7:9) = state(7:9) + acc_lin * p.ts;
                    state(10:12) = state(10:12) + acc_ang * p.ts;
                    state(1:3) = state(1:3) + state(7:9) * p.ts + 0.5 * acc_lin * p.ts^2;
                    state(4:6) = state(4:6) + state(10:12) * p.ts + 0.5 * acc_ang * p.ts^2;
                    app.stateHistory(:, k) = state;

                    % Update plots
                    tnow = app.timeVec(k);
                    for i = 1:6
                        addpoints(hPos.(coords{i}), tnow, state(i));
                    end
                    for i = 1:3
                        addpoints(hVel.(vcoords{i}), tnow, state(6+i));
                        addpoints(hAng.(acoords{i}), tnow, state(9+i));
                    end
                    for i = 1:8
                        pos = state(1:3) + p.r_pos(i, :)';
                        vec = p.dir_thr(i, :)' * thr_a(i) * arwScale;
                        set(quiv(i), 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), ...
                                     'UData', vec(1), 'VData', vec(2), 'WData', vec(3));
                    end
                    addpoints(hTraj, state(1), state(2), state(3));

                    % Update hull
                    phi = state(4); th = state(5); psi = state(6);
                    Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
                    Ry = [cos(th), 0, sin(th); 0, 1, 0; -sin(th), 0, cos(th)];
                    Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
                    R = Rz * Ry * Rx;
                    T4 = eye(4); T4(1:3, 1:3) = R; T4(1:3, 4) = state(1:3);
                    set(app.tr, 'Matrix', T4);

                    drawnow limitrate;
                    elapsed = toc;
                    wait = tnow - elapsed;
                    if wait > 0, pause(wait); end
                end
                app.updateResultsTab();
                uialert(app.UIFigure, 'Simulation completed successfully.', 'Success', 'Icon', 'success');
            catch ME
                uialert(app.UIFigure, sprintf('Simulation error: %s', ME.message), 'Error', 'Icon', 'error');
            end
        end

        function updateResultsTab(app)
            % Clear previous plots
            cla(app.UIAxesError);
            cla(app.UIAxesControlEffort);

            % Plot error over time
            axErr = app.UIAxesError;
            hold(axErr, 'on');
            coords = {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'};
            cols = {'r', 'g', 'b', 'm', 'c', 'k'};
            for i = 1:6
                plot(axErr, app.timeVec, app.errorHistory(i, :), 'Color', cols{i}, 'DisplayName', sprintf('%s Error', coords{i}));
            end
            title(axErr, 'Error Over Time');
            xlabel(axErr, 'Time (s)');
            ylabel(axErr, 'Error');
            legend(axErr, 'show');
            grid(axErr, 'on');
            hold(axErr, 'off');

            % Plot control effort (thrust magnitude)
            axEff = app.UIAxesControlEffort;
            hold(axEff, 'on');
            for i = 1:8
                plot(axEff, app.timeVec, abs(app.thrustHistory(i, :)), 'DisplayName', sprintf('Thruster %d', i));
            end
            title(axEff, 'Control Effort (Thrust Magnitude)');
            xlabel(axEff, 'Time (s)');
            ylabel(axEff, 'Thrust (N)');
            legend(axEff, 'show');
            grid(axEff, 'on');
            hold(axEff, 'off');

            % Calculate and display results
            resultsStr = sprintf('Simulation Results:\n\n');

            % Time to arrival (within 5% of desired pose, ensuring sustained threshold)
            thresholds = 0.05 * max(abs(app.simParams.desired), 0.1); % Avoid division by zero for small desired values
            arrival_times = zeros(6, 1);
            for i = 1:6
                err = abs(app.errorHistory(i, :));
                % Find first index where error stays below threshold for at least 5 timesteps
                idx = find(err < thresholds(i), 5, 'first');
                if length(idx) == 5 && all(diff(idx) == 1) && idx(1) > 1 % Ensure consecutive and not at t=0
                    arrival_times(i) = app.timeVec(idx(1));
                else
                    arrival_times(i) = Inf;
                end
            end
            resultsStr = [resultsStr, sprintf('Time to Arrival (within 5%% of desired pose):\n')];
            for i = 1:6
                if isinf(arrival_times(i))
                    resultsStr = [resultsStr, sprintf('  %s: Did not reach threshold\n', coords{i})];
                else
                    resultsStr = [resultsStr, sprintf('  %s: %.2f s\n', coords{i}, arrival_times(i))];
                end
            end

            % Steady-state error
            final_err = app.errorHistory(:, end);
            resultsStr = [resultsStr, sprintf('\nFinal Steady-State Error:\n')];
            for i = 1:6
                resultsStr = [resultsStr, sprintf('  %s: %.4f\n', coords{i}, final_err(i))];
            end

            % Stability analysis (variance of states in last 10%)
            N = length(app.timeVec);
            last_10pct = round(0.9 * N):N;
            state_variance = var(app.stateHistory(:, last_10pct), 0, 2);
            resultsStr = [resultsStr, sprintf('\nState Variance (Last 10%% of Simulation):\n')];
            for i = 1:6
                resultsStr = [resultsStr, sprintf('  %s: %.4f\n', coords{i}, state_variance(i))];
            end

            % Maximum overshoot
            max_overshoot = max(abs(app.errorHistory), [], 2);
            resultsStr = [resultsStr, sprintf('\nMaximum Overshoot:\n')];
            for i = 1:6
                resultsStr = [resultsStr, sprintf('  %s: %.4f\n', coords{i}, max_overshoot(i))];
            end

            % Energy consumption
            energy = sum(sum(app.thrustHistory.^2)) * app.simParams.ts;
            resultsStr = [resultsStr, sprintf('\nTotal Control Effort: %.2f N²s\n', energy)];

            % Check for potential instability in orientation
            if any(isinf(arrival_times(4:6)))
                resultsStr = [resultsStr, sprintf('\nWarning: Orientation (Roll, Pitch, Yaw) may be unstable. Consider increasing angular PID gains or checking thruster configuration.\n')];
            end

            % Update text area
            app.ResultsTextArea.Value = resultsStr;
        end

        function Reset(app, ~, ~)
            cla(app.UIAxes2D);
            cla(app.UIAxesThruster3D);
            cla(app.UIAxesTrajectory3D);
            cla(app.UIAxesHull3D);
            cla(app.UIAxesError);
            cla(app.UIAxesControlEffort);
            app.ResultsTextArea.Value = '';
            app.stateHistory = [];
            app.thrustHistory = [];
            app.errorHistory = [];
            app.timeVec = [];
            app.startup();
        end
    end

    methods (Access = private)
        function createComponents(app)
            app.UIFigure = uifigure('Name', 'ROV Simulation Platform', 'Position', [100 100 1400 900]);

            % Create Tab Group
            app.TabGroup = uitabgroup(app.UIFigure, 'Position', [10 10 1380 880]);
            
            % Create Parameters Tab
            app.ParametersTab = uitab(app.TabGroup, 'Title', 'Parameters');
            
            % Create Simulation Tab
            app.SimulationTab = uitab(app.TabGroup, 'Title', 'Simulation');
            
            % Create Visualization Tab
            app.VisualizationTab = uitab(app.TabGroup, 'Title', '3D Visualization');
            
            % Create Results Tab
            app.ResultsTab = uitab(app.TabGroup, 'Title', 'Results');

            % Control Buttons (visible on all tabs)
            app.LoadANSYSButton = uibutton(app.UIFigure, 'Text', 'Load ANSYS', 'Position', [20 890 100 30], ...
                                           'ButtonPushedFcn', @(s, e) app.LoadANSYS(s, e));
            app.RunSimButton = uibutton(app.UIFigure, 'Text', 'Run Simulation', 'Position', [140 890 120 30], ...
                                        'ButtonPushedFcn', @(s, e) app.RunSim(s, e));
            app.ResetButton = uibutton(app.UIFigure, 'Text', 'Reset', 'Position', [280 890 100 30], ...
                                       'ButtonPushedFcn', @(s, e) app.Reset(s, e));

            % === PARAMETERS TAB ===
            % Basic Properties Panel
            app.BasicPropsPanel = uipanel(app.ParametersTab, 'Title', 'Basic Properties', ...
                                          'Position', [20 740 350 100], 'FontWeight', 'bold');
            
            app.MassLabel = uilabel(app.BasicPropsPanel, 'Text', 'Mass (kg):', ...
                                    'Position', [10 50 80 22], 'FontWeight', 'bold');
            app.MassEdit = uieditfield(app.BasicPropsPanel, 'numeric', 'Position', [100 50 100 22], ...
                                       'Value', 44.05438, 'Limits', [0 Inf]);
            
            app.VdispLabel = uilabel(app.BasicPropsPanel, 'Text', 'Volume (m³):', ...
                                     'Position', [10 20 80 22], 'FontWeight', 'bold');
            app.VdispEdit = uieditfield(app.BasicPropsPanel, 'numeric', 'Position', [100 20 120 22], ...
                                        'Value', 4.4054e-2, 'Limits', [0 Inf]);

            % Center of Mass Panel
            app.COMPanel = uipanel(app.ParametersTab, 'Title', 'Center of Mass (m)', ...
                                   'Position', [20 640 350 80], 'FontWeight', 'bold');
            app.COMTable = uitable(app.COMPanel, ...
                                   'Units', 'normalized', 'Position', [0 0 1 1], ...
                                   'ColumnName', {'X', 'Y', 'Z'}, 'ColumnEditable', true, ...
                                   'ColumnWidth', 'auto');

            % Inertia Panel
            app.InertiaPanel = uipanel(app.ParametersTab, 'Title', 'Inertia Matrix Diagonal (kg⋅m²)', ...
                                       'Position', [20 540 350 80], 'FontWeight', 'bold');
            app.InertiaTable = uitable(app.InertiaPanel, ...
                                       'Units', 'normalized', 'Position', [0 0 1 1], ...
                                       'ColumnName', {'Ixx', 'Iyy', 'Izz'}, 'ColumnEditable', true, ...
                                       'ColumnWidth', 'auto');

            % Added Mass Panel
            app.AddedMassPanel = uipanel(app.ParametersTab, 'Title', 'Added Mass Diagonal', ...
                                         'Position', [20 440 350 80], 'FontWeight', 'bold');
            app.AddedMassTable = uitable(app.AddedMassPanel, ...
                                         'Units', 'normalized', 'Position', [0 0 1 1], ...
                                         'ColumnName', {'Am11', 'Am22', 'Am33', 'Am44', 'Am55', 'Am66'}, ...
                                         'ColumnEditable', true, 'ColumnWidth', 'auto');

            % Drag Panel
            app.DragPanel = uipanel(app.ParametersTab, 'Title', 'Drag Coefficients', ...
                                    'Position', [20 290 350 130], 'FontWeight', 'bold');
            uilabel(app.DragPanel, 'Text', 'CdA (Linear Drag):', 'Position', [10 85 150 22], 'FontWeight', 'bold');

    app.CdATable = uitable(app.DragPanel, ...
                           'Units', 'normalized', 'Position', [0 0.5 1 0.5], ...
                           'ColumnName', {'CdAx', 'CdAy', 'CdAz'}, 'ColumnEditable', true, ...
                           'ColumnWidth', 'auto');

            uilabel(app.DragPanel, 'Text', 'Rotational Drag:', 'Position', [10 25 150 22], 'FontWeight', 'bold');
            app.DrotTable = uitable(app.DragPanel, ...
                                    'Units', 'normalized', 'Position', [0 0 1 0.5], ...
                                    'ColumnName', {'Dr11', 'Dr22', 'Dr33'}, 'ColumnEditable', true, ...
                                    'ColumnWidth', 'auto');

            % Thruster Configuration Panel
            app.ThrusterPanel = uipanel(app.ParametersTab, 'Title', 'Thruster Configuration', ...
                                        'Position', [390 440 500 400], 'FontWeight', 'bold');
            app.ThrusterConfigTable = uitable(app.ThrusterPanel, ...
                                              'Units', 'normalized', 'Position', [0 0 1 1], ...
                                              'ColumnName', {'Pos X', 'Pos Y', 'Pos Z', 'Dir X', 'Dir Y', 'Dir Z'}, ...
                                              'RowName', {'T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7', 'T8'}, ...
                                              'ColumnEditable', true, 'ColumnWidth', 'auto');

            % Control Gains Panel
            app.ControlPanel = uipanel(app.ParametersTab, 'Title', 'Control Parameters', ...
                                       'Position', [910 440 450 400], 'FontWeight', 'bold');
            uilabel(app.ControlPanel, 'Text', 'PID Gains:', 'Position', [10 350 100 22], 'FontWeight', 'bold');
            app.GainsTable = uitable(app.ControlPanel, ...
                                     'Units', 'normalized', 'Position', [0 0.45 1 0.55], ...
                                     'ColumnName', {'Kp', 'Ki', 'Kd'}, ...
                                     'RowName', {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'}, ...
                                     'ColumnEditable', true, 'ColumnWidth', 'auto');
            
            uilabel(app.ControlPanel, 'Text', 'Desired Pose:', 'Position', [10 170 100 22], 'FontWeight', 'bold');
            app.DesiredPoseTable = uitable(app.ControlPanel, ...
                                           'Units', 'normalized', 'Position', [0 0 1 0.45], ...
                                           'ColumnName', {'X (m)', 'Y (m)', 'Z (m)', 'Roll (rad)', 'Pitch (rad)', 'Yaw (rad)'}, ...
                                           'ColumnEditable', true, 'Data', [20, 20, 20, 0, 0, 0], ...
                                           'ColumnWidth', 'auto');

            % === SIMULATION TAB ===
            app.UIAxes2D = uiaxes(app.SimulationTab, 'Position', [50 50 1280 780], 'Box', 'on');
            title(app.UIAxes2D, 'Real-time Simulation Results');
            xlabel(app.UIAxes2D, 'Time (s)');
            ylabel(app.UIAxes2D, 'State Variables');

            % === VISUALIZATION TAB ===
            app.UIAxesThruster3D = uiaxes(app.VisualizationTab, 'Position', [20 450 420 380], 'Box', 'on');
            title(app.UIAxesThruster3D, '3D Thruster Forces');
            
            app.UIAxesTrajectory3D = uiaxes(app.VisualizationTab, 'Position', [460 450 420 380], 'Box', 'on');
            title(app.UIAxesTrajectory3D, '3D Trajectory');
            
            app.UIAxesHull3D = uiaxes(app.VisualizationTab, 'Position', [900 450 420 380], 'Box', 'on');
            title(app.UIAxesHull3D, '3D Hull Visualization');

            % === RESULTS TAB ===
            app.UIAxesError = uiaxes(app.ResultsTab, 'Position', [20 450 640 380], 'Box', 'on');
            title(app.UIAxesError, 'Error Over Time');
            xlabel(app.UIAxesError, 'Time (s)');
            ylabel(app.UIAxesError, 'Error');

            app.UIAxesControlEffort = uiaxes(app.ResultsTab, 'Position', [680 450 640 380], 'Box', 'on');
            title(app.UIAxesControlEffort, 'Control Effort (Thrust Magnitude)');
            xlabel(app.UIAxesControlEffort, 'Time (s)');
            ylabel(app.UIAxesControlEffort, 'Thrust (N)');

            app.ResultsTextArea = uitextarea(app.ResultsTab, 'Position', [20 50 1280 380], ...
                                             'FontSize', 12, 'Editable', 'off');
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