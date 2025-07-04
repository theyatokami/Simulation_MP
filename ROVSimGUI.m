classdef ROVSimGUI < matlab.apps.AppBase
    % ROVSimGUI: GUI for 6-DOF ROV Simulation with CSV drag loading

    properties (Access = public)
        UIFigure                 matlab.ui.Figure
        TabGroup                 matlab.ui.container.TabGroup
        ParametersTab            matlab.ui.container.Tab
        SimulationTab            matlab.ui.container.Tab
        VisualizationTab         matlab.ui.container.Tab
        ResultsTab               matlab.ui.container.Tab
        DragFittingTab           matlab.ui.container.Tab
        
        % Control buttons
        LoadMATButton            matlab.ui.control.Button
        LoadDragButton           matlab.ui.control.Button
        RunSimButton             matlab.ui.control.Button
        ResetButton              matlab.ui.control.Button
        FitDragButton            matlab.ui.control.Button
        
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

        % Drag Fitting Tab Controls
        NumPointsLabel           matlab.ui.control.Label
        NumPointsEdit            matlab.ui.control.NumericEditField
        LinearDragTable          matlab.ui.control.Table
        RotationalDragTable      matlab.ui.control.Table
        UIAxesLinearDrag         matlab.ui.control.UIAxes
        UIAxesRotationalDrag     matlab.ui.control.UIAxes
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
            p.polyDragLin = []; % Polynomial coefficients for linear drag (3x6)
            p.polyDragRot = []; % Polynomial coefficients for rotational drag (3x6)
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
            app.NumPointsEdit.Value = 11; % Default number of points
            app.updateDragTables();
        end

        function updateDragTables(app)
            % Update drag tables based on number of points
            try
                numPoints = round(app.NumPointsEdit.Value);
                if numPoints < 6
                    uialert(app.UIFigure, 'Number of points must be at least 6 for quintic fit.', 'Input Error');
                    app.NumPointsEdit.Value = 6;
                    numPoints = 6;
                end
                v = linspace(-1, 1, numPoints)'; % Default range, editable by user
                app.LinearDragTable.Data = [v, zeros(numPoints, 3)];
                app.RotationalDragTable.Data = [v, zeros(numPoints, 3)];
            catch ME
                uialert(app.UIFigure, sprintf('Error updating drag tables: %s', ME.message), 'Error', 'Icon', 'error');
            end
        end

        function LoadMAT(app, ~, ~)
            % Load parameters from .mat file
            [file, path] = uigetfile('*.mat', 'Select MAT File');
            if isequal(file, 0), return; end
            fullPath = fullfile(path, file);
            try
                data = load(fullPath);
                fn = fieldnames(data);
                p = app.simParams; % Start with current parameters
                for k = 1:length(fn)
                    if isfield(p, fn{k})
                        p.(fn{k}) = data.(fn{k});
                    end
                end
                app.simParams = p;

                % Update UI with loaded parameters
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
                uialert(app.UIFigure, sprintf('Parameters loaded from %s.', fullPath), 'Success', 'Icon', 'success');
            catch ME
                uialert(app.UIFigure, sprintf('Error loading MAT file: %s', ME.message), 'Error', 'Icon', 'error');
            end
        end

        function LoadDragFromCSV(app, ~, ~)
            % Load drag data from CSV file
            [file, path] = uigetfile('*.csv', 'Select Drag Data CSV File', 'drag_data.csv');
            if isequal(file, 0), return; end
            fullPath = fullfile(path, file);
            try
                % Read the entire file as text
                fid = fopen(fullPath, 'r');
                if fid == -1
                    uialert(app.UIFigure, 'Cannot open CSV file.', 'File Error');
                    return;
                end
                lines = textscan(fid, '%s', 'Delimiter', '\n', 'Whitespace', '');
                fclose(fid);
                lines = lines{1};

                % Initialize data storage
                linearData = [];
                rotationalData = [];
                section = '';

                % Parse lines
                for i = 1:length(lines)
                    line = strtrim(lines{i});
                    if isempty(line) || strcmp(line, ','), continue; end % Skip empty or comma-only lines

                    if strcmpi(line, 'Linear')
                        section = 'Linear';
                    elseif strcmpi(line, 'Rotational')
                        section = 'Rotational';
                    elseif ~isempty(section) && ~strcmp(line(1), '%') % Skip comments
                        % Split by comma and convert to numbers
                        data = str2double(regexp(line, '[^,]+', 'match'));
                        if length(data) == 4 && all(~isnan(data)) % Ensure 4 valid columns
                            if strcmp(section, 'Linear')
                                linearData = [linearData; data];
                            elseif strcmp(section, 'Rotational')
                                rotationalData = [rotationalData; data];
                            end
                        end
                    end
                end

                % Validate data
                if isempty(linearData) || isempty(rotationalData)
                    uialert(app.UIFigure, 'CSV file must contain both Linear and Rotational sections with data.', 'File Error');
                    return;
                end
                if size(linearData, 2) ~= 4 || size(rotationalData, 2) ~= 4
                    uialert(app.UIFigure, 'Each section must have 4 columns (A, B, C, D).', 'File Error');
                    return;
                end
                if size(linearData, 1) < 6 || size(rotationalData, 1) < 6
                    uialert(app.UIFigure, 'Each section must contain at least 6 data points.', 'Input Error');
                    return;
                end

                % Update tables
                app.NumPointsEdit.Value = max(size(linearData, 1), size(rotationalData, 1));
                app.LinearDragTable.Data = linearData;
                app.RotationalDragTable.Data = rotationalData;
                uialert(app.UIFigure, sprintf('Drag data loaded from %s.', fullPath), 'Success', 'Icon', 'success');
            catch ME
                uialert(app.UIFigure, sprintf('Error loading drag data: %s', ME.message), 'Error', 'Icon', 'error');
            end
        end

        function FitDragPolynomials(app, ~, ~)
            try
                % Get data from tables
                linData = app.LinearDragTable.Data;
                rotData = app.RotationalDragTable.Data;
                
                % Validate input
                if isempty(linData) || isempty(rotData)
                    uialert(app.UIFigure, 'Drag tables cannot be empty.', 'Input Error');
                    return;
                end
                if any(isnan(linData(:))) || any(isnan(rotData(:)))
                    uialert(app.UIFigure, 'Drag tables contain invalid (NaN) values.', 'Input Error');
                    return;
                end

                % Fit polynomials (quintic for 6 coefficients)
                v = linData(:, 1);
                pLin = zeros(3, 6); % 3 axes, 6 coefficients each
                pRot = zeros(3, 6); % 3 axes, 6 coefficients each
                for i = 1:3
                    pLin(i, :) = polyfit(v, linData(:, i+1), 5);
                    pRot(i, :) = polyfit(v, rotData(:, i+1), 5);
                end

                % Store in simParams
                app.simParams.polyDragLin = pLin;
                app.simParams.polyDragRot = pRot;

                % Update plots
                cla(app.UIAxesLinearDrag);
                cla(app.UIAxesRotationalDrag);
                hold(app.UIAxesLinearDrag, 'on');
                hold(app.UIAxesRotationalDrag, 'on');
                vFit = linspace(min(v), max(v), 100)';
                for i = 1:3
                    plot(app.UIAxesLinearDrag, vFit, polyval(pLin(i, :), vFit), 'DisplayName', sprintf('Axis %d', i));
                    plot(app.UIAxesRotationalDrag, vFit, polyval(pRot(i, :), vFit), 'DisplayName', sprintf('Axis %d', i));
                end
                title(app.UIAxesLinearDrag, 'Linear Drag Fit');
                title(app.UIAxesRotationalDrag, 'Rotational Drag Fit');
                legend(app.UIAxesLinearDrag, 'show');
                legend(app.UIAxesRotationalDrag, 'show');
                hold(app.UIAxesLinearDrag, 'off');
                hold(app.UIAxesRotationalDrag, 'off');

                uialert(app.UIFigure, 'Drag polynomials fitted successfully.', 'Success', 'Icon', 'success');
            catch ME
                uialert(app.UIFigure, sprintf('Error fitting drag polynomials: %s', ME.message), 'Error', 'Icon', 'error');
            end
        end

        function RunSim(app, ~, ~)
            % Update simParams from GUI inputs
            try
                app.simParams.mass = app.MassEdit.Value;
                app.simParams.Vdisp = app.VdispEdit.Value;
                app.simParams.com = app.COMTable.Data';
                app.simParams.I = diag(app.InertiaTable.Data);
                app.simParams.added = diag(app.AddedMassTable.Data);
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
            Tmax = 40;
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

                    % Forces and moments (using polynomial drag if available)
                    vLin = state(7:9);
                    vRot = state(10:12);
                    if ~isempty(p.polyDragLin)
                        F = zeros(3, 1);
                        for i = 1:3
                            F(i) = polyval(p.polyDragLin(i, :), vLin(i));
                        end
                    else
                        F = -p.CdA .* abs(vLin) .* vLin; % Fallback to quadratic drag
                    end
                    if ~isempty(p.polyDragRot)
                        M_t = zeros(3, 1);
                        for i = 1:3
                            M_t(i) = polyval(p.polyDragRot(i, :), vRot(i));
                        end
                    else
                        M_t = -p.D_rot * vRot; % Fallback to linear rotational drag
                    end
                    TF = A * thr_a;
                    F = F + TF(1:3);
                    M_t = M_t + TF(4:6) + cross(p.com, [0; 0; (p.Fb - p.mass * 9.81)]);

                    % Dynamics
                    acc_lin = invM_lin * F;
                    acc_ang = invM_ang * M_t;
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
            thresholds = 0.05 * max(abs(app.simParams.desired), 0.1); % Avoid division by zero
            arrival_times = zeros(6, 1);
            for i = 1:6
                err = abs(app.errorHistory(i, :));
                idx = find(err < thresholds(i), 5, 'first');
                if length(idx) == 5 && all(diff(idx) == 1) && idx(1) > 1
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
            
            % Create Drag Fitting Tab
            app.DragFittingTab = uitab(app.TabGroup, 'Title', 'Drag Fitting');

            % Control Buttons
            app.LoadMATButton = uibutton(app.UIFigure, 'Text', 'Load MAT', 'Position', [20 890 100 30], ...
                                         'ButtonPushedFcn', @(s, e) app.LoadMAT(s, e));
            app.LoadDragButton = uibutton(app.UIFigure, 'Text', 'Load Drag CSV', 'Position', [140 890 120 30], ...
                                          'ButtonPushedFcn', @(s, e) app.LoadDragFromCSV(s, e));
            app.RunSimButton = uibutton(app.UIFigure, 'Text', 'Run Simulation', 'Position', [280 890 120 30], ...
                                        'ButtonPushedFcn', @(s, e) app.RunSim(s, e));
            app.ResetButton = uibutton(app.UIFigure, 'Text', 'Reset', 'Position', [420 890 100 30], ...
                                       'ButtonPushedFcn', @(s, e) app.Reset(s, e));
            app.FitDragButton = uibutton(app.UIFigure, 'Text', 'Fit Drag Polynomials', 'Position', [540 890 150 30], ...
                                         'ButtonPushedFcn', @(s, e) app.FitDragPolynomials(s, e));

            % === PARAMETERS TAB ===
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

            app.COMPanel = uipanel(app.ParametersTab, 'Title', 'Center of Mass (m)', ...
                                   'Position', [20 640 350 80], 'FontWeight', 'bold');
            app.COMTable = uitable(app.COMPanel, 'Units', 'normalized', 'Position', [0 0 1 1], ...
                                   'ColumnName', {'X', 'Y', 'Z'}, 'ColumnEditable', true, 'ColumnWidth', 'auto');

            app.InertiaPanel = uipanel(app.ParametersTab, 'Title', 'Inertia Matrix Diagonal (kg m²)', ...
                                       'Position', [20 540 350 80], 'FontWeight', 'bold');
            app.InertiaTable = uitable(app.InertiaPanel, 'Units', 'normalized', 'Position', [0 0 1 1], ...
                                       'ColumnName', {'Ixx', 'Iyy', 'Izz'}, 'ColumnEditable', true, 'ColumnWidth', 'auto');

            app.AddedMassPanel = uipanel(app.ParametersTab, 'Title', 'Added Mass Diagonal', ...
                                         'Position', [20 440 350 80], 'FontWeight', 'bold');
            app.AddedMassTable = uitable(app.AddedMassPanel, 'Units', 'normalized', 'Position', [0 0 1 1], ...
                                         'ColumnName', {'Am11', 'Am22', 'Am33', 'Am44', 'Am55', 'Am66'}, ...
                                         'ColumnEditable', true, 'ColumnWidth', 'auto');

            app.DragPanel = uipanel(app.ParametersTab, 'Title', 'Drag Coefficients', ...
                                    'Position', [20 290 350 130], 'FontWeight', 'bold');
            uilabel(app.DragPanel, 'Text', 'CdA (Linear Drag):', 'Position', [10 85 150 22], 'FontWeight', 'bold');
            app.CdATable = uitable(app.DragPanel, 'Units', 'normalized', 'Position', [0 0.5 1 0.5], ...
                                   'ColumnName', {'CdAx', 'CdAy', 'CdAz'}, 'ColumnEditable', true, 'ColumnWidth', 'auto');
            uilabel(app.DragPanel, 'Text', 'Rotational Drag:', 'Position', [10 25 150 22], 'FontWeight', 'bold');
            app.DrotTable = uitable(app.DragPanel, 'Units', 'normalized', 'Position', [0 0 1 0.5], ...
                                    'ColumnName', {'Dr11', 'Dr22', 'Dr33'}, 'ColumnEditable', true, 'ColumnWidth', 'auto');

            app.ThrusterPanel = uipanel(app.ParametersTab, 'Title', 'Thruster Configuration', ...
                                        'Position', [390 440 500 400], 'FontWeight', 'bold');
            app.ThrusterConfigTable = uitable(app.ThrusterPanel, 'Units', 'normalized', 'Position', [0 0 1 1], ...
                                              'ColumnName', {'Pos X', 'Pos Y', 'Pos Z', 'Dir X', 'Dir Y', 'Dir Z'}, ...
                                              'RowName', {'T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7', 'T8'}, ...
                                              'ColumnEditable', true, 'ColumnWidth', 'auto');

            app.ControlPanel = uipanel(app.ParametersTab, 'Title', 'Control Parameters', ...
                                       'Position', [910 440 450 400], 'FontWeight', 'bold');
            uilabel(app.ControlPanel, 'Text', 'PID Gains:', 'Position', [10 350 100 22], 'FontWeight', 'bold');
            app.GainsTable = uitable(app.ControlPanel, 'Units', 'normalized', 'Position', [0 0.45 1 0.55], ...
                                     'ColumnName', {'Kp', 'Ki', 'Kd'}, 'RowName', {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'}, ...
                                     'ColumnEditable', true, 'ColumnWidth', 'auto');
            uilabel(app.ControlPanel, 'Text', 'Desired Pose:', 'Position', [10 170 100 22], 'FontWeight', 'bold');
            app.DesiredPoseTable = uitable(app.ControlPanel, 'Units', 'normalized', 'Position', [0 0 1 0.45], ...
                                           'ColumnName', {'X (m)', 'Y (m)', 'Z (m)', 'Roll (rad)', 'Pitch (rad)', 'Yaw (rad)'}, ...
                                           'ColumnEditable', true, 'Data', [20, 20, 20, 0, 0, 0], 'ColumnWidth', 'auto');

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

            % === DRAG FITTING TAB ===
            app.NumPointsLabel = uilabel(app.DragFittingTab, 'Text', 'Number of Points:', ...
                                         'Position', [20 830 120 22], 'FontWeight', 'bold');
            app.NumPointsEdit = uieditfield(app.DragFittingTab, 'numeric', 'Position', [150 830 100 22], ...
                                            'Value', 11, 'Limits', [6 Inf], 'ValueChangedFcn', @(s, e) app.updateDragTables());
            app.LinearDragTable = uitable(app.DragFittingTab, 'Position', [20 600 600 200], ...
                                          'ColumnName', {'A', 'B', 'C', 'D'}, ...
                                          'ColumnEditable', true, 'ColumnWidth', 'auto');
            app.RotationalDragTable = uitable(app.DragFittingTab, 'Position', [650 600 600 200], ...
                                              'ColumnName', {'A', 'B', 'C', 'D'}, ...
                                              'ColumnEditable', true, 'ColumnWidth', 'auto');
            app.UIAxesLinearDrag = uiaxes(app.DragFittingTab, 'Position', [20 350 600 200], 'Box', 'on');
            title(app.UIAxesLinearDrag, 'Linear Drag Fit');
            app.UIAxesRotationalDrag = uiaxes(app.DragFittingTab, 'Position', [650 350 600 200], 'Box', 'on');
            title(app.UIAxesRotationalDrag, 'Rotational Drag Fit');
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