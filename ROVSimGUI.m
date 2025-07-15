classdef ROVSimGUI < matlab.apps.AppBase
    % ROVSimGUI: GUI for 6-DOF ROV Simulation with .mat parameter storage and Cascade PID control

    % === Properties ===
    properties (Access = public)
        % GUI Components
        UIFigure                 matlab.ui.Figure
        TabGroup                 matlab.ui.container.TabGroup
        ParametersTab            matlab.ui.container.Tab
        SimulationTab            matlab.ui.container.Tab
        VisualizationTab         matlab.ui.container.Tab
        ResultsTab               matlab.ui.container.Tab
        DragFittingTab           matlab.ui.container.Tab
        PIDTuningTab             matlab.ui.container.Tab
        
        % Control Buttons
        LoadDragButton           matlab.ui.control.Button
        RunSimButton             matlab.ui.control.Button
        ResetButton              matlab.ui.control.Button
        FitDragButton            matlab.ui.control.Button
        ForceLoadMatButton       matlab.ui.control.Button
        ResetDefaultsButton      matlab.ui.control.Button
        
        % Parameters Tab Controls
        BasicPropsPanel          matlab.ui.container.Panel
        MassLabel                matlab.ui.control.Label
        MassEdit                 matlab.ui.control.NumericEditField
        VdispLabel               matlab.ui.control.Label
        VdispEdit                matlab.ui.control.NumericEditField
        MaxThrustLabel           matlab.ui.control.Label
        MaxThrustEdit            matlab.ui.control.NumericEditField
        
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
        VelocityGainsTable       matlab.ui.control.Table
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

        % PID Tuning Tab Controls
        TuningGoalDropDown       matlab.ui.control.DropDown
        TunePIDButton            matlab.ui.control.Button
    end

    properties (Access = private)
        simParams                % Simulation parameters struct
        timeVec                  % Simulation time vector
        stateHistory             % State history (12 x N)
        thrustHistory            % Thrust history (8 x N)
        faces                    % STL faces
        vertices                 % STL vertices
        tr                       % Hull transform object
        errorHistory             % Pose error history (6 x N)
        matFile                  % Path to parameters .mat file
    end

    % === Private Methods ===
    methods (Access = private)
        % --- Startup and Initialization ---
        function startup(app)
            % Initialize .mat file path and load ROV geometry/parameters
            scriptDir = fileparts(mfilename('fullpath'));
            app.matFile = fullfile(scriptDir, 'rov_parameters.mat');

            % Load STL geometry for ROV visualization
            stlFile = fullfile(scriptDir, 'Preliminary-ROV.stl');
            if ~isfile(stlFile)
                warning('Geometric model not found: %s. Using default geometry.', stlFile);
                [x, y, z] = meshgrid([-0.5, 0.5], [-0.2, 0.2], [-0.1, 0.1]);
                app.vertices = [x(:), y(:), z(:)];
                app.faces = convhull(app.vertices);
            else
                fv_full = stlread(stlFile);
                [app.faces, verts_mm] = reducepatch(fv_full.ConnectivityList, fv_full.Points, 0.2);
                app.vertices = verts_mm * 1e-3; % Convert mm to m
            end

            % Load parameters from .mat or use defaults
            app.forceLoadParameters();
        end

        % --- Force Load Parameters from .mat ---
        function forceLoadParameters(app)
            % Force reload parameters from .mat file
            if isfile(app.matFile)
                try
                    load(app.matFile, 'simParams');
                    % Define required fields
                    requiredFields = {'mass', 'Vdisp', 'maxThrust', 'com', 'I', 'added', ...
                                      'CdA', 'D_rot', 'r_pos', 'dir_thr', 'Kp', 'Ki', 'Kd', ...
                                      'Kv_p', 'Kv_i', 'Kv_d', 'desired', 'polyDragLin', 'polyDragRot', ...
                                      'ts', 't_final', 'Fb'};
                    % Check if all required fields exist
                    missingFields = requiredFields(~isfield(simParams, requiredFields));
                    if isempty(missingFields)
                        app.simParams = simParams;
                        disp('Parameters loaded successfully from .mat file.');
                    else
                        warning('Missing fields in .mat file: %s. Initializing defaults.', strjoin(missingFields, ', '));
                        app.initializeDefaultParameters();
                        save(app.matFile, 'simParams');
                    end
                catch ME
                    uialert(app.UIFigure, sprintf('Error loading .mat: %s. Initializing defaults.', ME.message), 'Error', 'Icon', 'error');
                    app.initializeDefaultParameters();
                    save(app.matFile, 'simParams');
                end
            else
                app.initializeDefaultParameters();
                save(app.matFile, 'simParams');
            end

            % Populate UI components with defaults if not already set
            try
                app.MassEdit.Value = app.simParams.mass;
                app.VdispEdit.Value = app.simParams.Vdisp;
                app.MaxThrustEdit.Value = app.simParams.maxThrust;
                app.COMTable.Data = app.simParams.com';
                app.InertiaTable.Data = diag(app.simParams.I)';
                app.AddedMassTable.Data = diag(app.simParams.added)';
                app.CdATable.Data = app.simParams.CdA';
                app.DrotTable.Data = diag(app.simParams.D_rot)';
                app.ThrusterConfigTable.Data = [app.simParams.r_pos, app.simParams.dir_thr];
                app.GainsTable.Data = [diag(app.simParams.Kp), diag(app.simParams.Ki), diag(app.simParams.Kd)];
                app.VelocityGainsTable.Data = [diag(app.simParams.Kv_p), diag(app.simParams.Kv_i), diag(app.simParams.Kv_d)];
                app.DesiredPoseTable.Data = app.simParams.desired';
                app.NumPointsEdit.Value = 11;
                app.updateDragTables();
                app.TuningGoalDropDown.Items = {'Optimized Tuning'};
                app.TuningGoalDropDown.Value = 'Optimized Tuning';
            catch ME
                uialert(app.UIFigure, sprintf('Error populating UI: %s', ME.message), 'Error', 'Icon', 'error');
            end
        end

        % --- Parameter Initialization ---
        function initializeDefaultParameters(app)
            % Initialize default simulation parameters
            p = struct();
            p.mass = 44.05438; % Mass in kg
            p.Vdisp = 4.4054e-2; % Displaced volume in m³
            p.maxThrust = 40; % Maximum thruster force in N
            p.Fb = 1000 * p.Vdisp * 9.81; % Buoyancy force
            p.ts = 0.1; % Time step in s
            p.t_final = 120; % Total simulation time in s
            p.com = [-0.42049; 0.00072; -0.02683]; % Center of Mass (m)
            p.I = diag([1.990993, 15.350344, 15.774167]); % Inertia matrix (diagonal, kg·m²)
            p.added = diag([5, 5, 10, 1, 1, 1]); % Added mass matrix (diagonal)
            p.CdA = [0.5 * 1000 * 1.1 * 0.04; 0.5 * 1000 * 1.1 * 0.1; 0.5 * 1000 * 1.1 * 0.1]; % Linear drag coefficients (N·s²/m²)
            p.D_rot = diag([5, 5, 5]); % Rotational drag matrix (diagonal)
            p.r_pos = [0.2, 0.1, 0.1; 0.2, -0.1, 0.1; -0.2, -0.1, 0.1; -0.2, 0.1, 0.1;
                       0.2, 0.1, 0; 0.2, -0.1, 0; -0.2, -0.1, 0; -0.2, 0.1, 0]; % Thruster positions (m)
            deg = [nan, nan, nan, nan, -45, 45, 135, -135];
            p.dir_thr = [repmat([0, 0, 1], 4, 1); [cosd(deg(5:8))', sind(deg(5:8))', zeros(4, 1)]]; % Thruster directions
            p.Kp = diag([4, 4, 4, 0.8, 0.8, 1]); % Outer loop PID gains (pose control)
            p.Ki = diag([0.001, 0.001, 0.001, 0, 0, 0]);
            p.Kd = diag([0.6, 0.6, 0.6, 1.5, 1.5, 2]);
            p.Kv_p = diag([10, 10, 10, 5, 5, 5]); % Inner loop PID gains (velocity control)
            p.Kv_i = diag([0.01, 0.01, 0.01, 0, 0, 0]);
            p.Kv_d = diag([2, 2, 2, 1, 1, 1]);
            p.desired = [20; 20; 20; 0; 0; 0]; % Desired pose (x, y, z in m; roll, pitch, yaw in rad)
            p.polyDragLin = []; % Drag polynomial fits (empty by default)
            p.polyDragRot = [];
            app.simParams = p;
        end

        % --- Parameter Saving ---
        function saveParametersToMat(app)
            % Save simulation parameters to .mat file
            try
                simParams = app.simParams;
                save(app.matFile, 'simParams');
                disp(['Parameters saved successfully to: ', app.matFile]);
            catch ME
                uialert(app.UIFigure, sprintf('Failed to save .mat: %s', ME.message), 'Error', 'Icon', 'error');
            end
        end

        % --- Reset to Default Parameters ---
        function resetToDefaults(app, ~, ~)
            % Reset parameters to defaults and update UI
            try
                app.initializeDefaultParameters();
                app.saveParametersToMat();
                app.forceLoadParameters(); % Update UI with default values
                uialert(app.UIFigure, 'Parameters reset to defaults successfully.', 'Success', 'Icon', 'success');
            catch ME
                uialert(app.UIFigure, sprintf('Error resetting parameters: %s', ME.message), 'Error', 'Icon', 'error');
            end
        end

        % --- Drag Table Management ---
        function updateDragTables(app)
            % Update drag tables based on number of points
            try
                numPoints = round(app.NumPointsEdit.Value);
                if numPoints < 6
                    uialert(app.UIFigure, 'Number of points must be at least 6 for quintic fit.', 'Input Error');
                    app.NumPointsEdit.Value = 6;
                    numPoints = 6;
                end
                v = linspace(-1, 1, numPoints)';
                app.LinearDragTable.Data = [v, zeros(numPoints, 3)];
                app.RotationalDragTable.Data = [v, zeros(numPoints, 3)];
                app.saveParametersToMat();
            catch ME
                uialert(app.UIFigure, sprintf('Error updating drag tables: %s', ME.message), 'Error', 'Icon', 'error');
            end
        end

        % --- Drag Data Loading ---
        function LoadDragFromCSV(app, ~, ~)
            % Load drag data from a CSV file
            [file, path] = uigetfile('*.csv', 'Select Drag Data CSV File', 'drag_data.csv');
            if isequal(file, 0), return; end
            fullPath = fullfile(path, file);
            try
                fid = fopen(fullPath, 'r');
                if fid == -1
                    uialert(app.UIFigure, 'Cannot open CSV file.', 'File Error');
                    return;
                end
                lines = textscan(fid, '%s', 'Delimiter', '\n', 'Whitespace', '');
                fclose(fid);
                lines = lines{1};

                linearData = [];
                rotationalData = [];
                section = '';

                for i = 1:length(lines)
                    line = strtrim(lines{i});
                    if isempty(line) || strcmp(line, ','), continue; end
                    if strcmpi(line, 'Linear')
                        section = 'Linear';
                    elseif strcmpi(line, 'Rotational')
                        section = 'Rotational';
                    elseif ~isempty(section) && ~strcmp(line(1), '%')
                        data = str2double(regexp(line, '[^,]+', 'match'));
                        if length(data) == 4 && all(~isnan(data))
                            if strcmp(section, 'Linear')
                                linearData = [linearData; data];
                            elseif strcmp(section, 'Rotational')
                                rotationalData = [rotationalData; data];
                            end
                        end
                    end
                end

                if isempty(linearData) || isempty(rotationalData)
                    uialert(app.UIFigure, 'CSV file must contain both Linear and Rotational sections with data.', 'File Error');
                    return;
                end
                if size(linearData, 2) ~= 4 || size(rotationalData, 2) ~= 4
                    uialert(app.UIFigure, 'Each section must have 4 columns (A, B, C, D).', 'File Error');
                    return;
                end
                if size(linearData, 1) < 6 || size(rotationalData, 1) < 6
                    uialert(app.UIFigure, 'Each section must have at least 6 data points.', 'Input Error');
                    return;
                end

                app.NumPointsEdit.Value = max(size(linearData, 1), size(rotationalData, 1));
                app.LinearDragTable.Data = linearData;
                app.RotationalDragTable.Data = rotationalData;
                app.saveParametersToMat();
                uialert(app.UIFigure, sprintf('Drag data loaded from %s.', fullPath), 'Success', 'Icon', 'success');
            catch ME
                uialert(app.UIFigure, sprintf('Error loading drag data: %s', ME.message), 'Error', 'Icon', 'error');
            end
        end

        % --- Drag Polynomial Fitting ---
        function FitDragPolynomials(app, ~, ~)
            % Fit quintic polynomials to drag data and plot results
            try
                linData = app.LinearDragTable.Data;
                rotData = app.RotationalDragTable.Data;
                
                if isempty(linData) || isempty(rotData)
                    uialert(app.UIFigure, 'Drag tables cannot be empty.', 'Input Error');
                    return;
                end
                if any(isnan(linData(:))) || any(isnan(rotData(:)))
                    uialert(app.UIFigure, 'Drag tables contain invalid (NaN) values.', 'Input Error');
                    return;
                end

                v = linData(:, 1);
                pLin = zeros(3, 6);
                pRot = zeros(3, 6);
                for i = 1:3
                    pLin(i, :) = polyfit(v, linData(:, i+1), 5);
                    pRot(i, :) = polyfit(v, rotData(:, i+1), 5);
                end

                app.simParams.polyDragLin = pLin;
                app.simParams.polyDragRot = pRot;

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

                app.saveParametersToMat();
                uialert(app.UIFigure, 'Drag polynomials fitted successfully.', 'Success', 'Icon', 'success');
            catch ME
                uialert(app.UIFigure, sprintf('Error fitting drag polynomials: %s', ME.message), 'Error', 'Icon', 'error');
            end
        end

        % --- Automatic PID Tuning ---
        function TunePIDAutomatically(app, ~, ~)
            % Automatically tune PID gains for optimized performance
            p = app.simParams;

            % Define tighter gain bounds
            Kp_lb = zeros(6, 1); Kp_ub = 5 * ones(6, 1);
            Ki_lb = zeros(6, 1); Ki_ub = 0.05 * ones(6, 1);
            Kd_lb = zeros(6, 1); Kd_ub = 1 * ones(6, 1);
            Kv_p_lb = zeros(6, 1); Kv_p_ub = 10 * ones(6, 1);
            Kv_i_lb = zeros(6, 1); Kv_i_ub = 0.05 * ones(6, 1);
            Kv_d_lb = zeros(6, 1); Kv_d_ub = 1 * ones(6, 1);

            % Initial guess (current values)
            Kp0 = diag(app.simParams.Kp);
            Ki0 = diag(app.simParams.Ki);
            Kd0 = diag(app.simParams.Kd);
            Kv_p0 = diag(app.simParams.Kv_p);
            Kv_i0 = diag(app.simParams.Kv_i);
            Kv_d0 = diag(app.simParams.Kv_d);

            % Objective function for optimization
            costFunc = @(x) app.computeCost(x, p);

            % Optimization variables: [Kp Ki Kd Kv_p Kv_i Kv_d] for all 6 axes
            x0 = [Kp0; Ki0; Kd0; Kv_p0; Kv_i0; Kv_d0];
            lb = [Kp_lb; Ki_lb; Kd_lb; Kv_p_lb; Kv_i_lb; Kv_d_lb];
            ub = [Kp_ub; Ki_ub; Kd_ub; Kv_p_ub; Kv_i_ub; Kv_d_ub];

            % Use fmincon for constrained optimization
            options = optimoptions('fmincon', 'Display', 'iter', 'MaxIterations', 50, 'MaxFunctionEvaluations', 500);
            [x_opt, ~] = fmincon(costFunc, x0, [], [], [], [], lb, ub, [], options);

            % Extract optimized gains
            idx = 1:6;
            app.simParams.Kp = diag(x_opt(idx));
            app.simParams.Ki = diag(x_opt(idx + 6));
            app.simParams.Kd = diag(x_opt(idx + 12));
            app.simParams.Kv_p = diag(x_opt(idx + 18));
            app.simParams.Kv_i = diag(x_opt(idx + 24));
            app.simParams.Kv_d = diag(x_opt(idx + 30));

            % Update UI tables
            app.GainsTable.Data = [diag(app.simParams.Kp), diag(app.simParams.Ki), diag(app.simParams.Kd)];
            app.VelocityGainsTable.Data = [diag(app.simParams.Kv_p), diag(app.simParams.Kv_i), diag(app.simParams.Kv_d)];
            app.saveParametersToMat();
            uialert(app.UIFigure, 'PID tuned for optimized performance (minimal overshoot, minimal error, faster response) successfully.', 'Success', 'Icon', 'success');
        end

        % --- Cost Function for PID Tuning ---
        function cost = computeCost(app, x, p)
            % Simulate with current gains and compute cost for optimized tuning
            p.Kp = diag(x(1:6));
            p.Ki = diag(x(7:12));
            p.Kd = diag(x(13:18));
            p.Kv_p = diag(x(19:24));
            p.Kv_i = diag(x(25:30));
            p.Kv_d = diag(x(31:36));
        
            % Longer simulation to evaluate steady-state
            N = round(p.t_final / p.ts);
            timeVec = (0:N-1) * p.ts;
            stateHistory = zeros(12, N);
            errorHistory = zeros(6, N);
            thrustHistory = zeros(8, N);
        
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
        
            state = zeros(12, 1);
            intErr = zeros(6, 1);
            prevErr = p.desired;
            v_intErr = zeros(6, 1);
            v_prevErr = zeros(6, 1);
            thr_e = zeros(8, 1);
            thr_a = zeros(8, 1);
            Tmax = p.maxThrust;
            tau_e = 0.05;
            tau_m = 0.15;
            alpha_e = p.ts / (tau_e + p.ts);
            alpha_m = p.ts / (tau_m + p.ts);
        
            for k = 2:N
                err = p.desired - state(1:6);
                errorHistory(:, k) = err;
                intErr = intErr + err * p.ts;
                dErr = (err - prevErr) / p.ts;
                prevErr = err;
                v_desired = p.Kp * err + p.Ki * intErr + p.Kd * dErr;
        
                v_err = v_desired - state(7:12);
                v_intErr = v_intErr + v_err * p.ts;
                v_dErr = (v_err - v_prevErr) / p.ts;
                v_prevErr = v_err;
                tau = p.Kv_p * v_err + p.Kv_i * v_intErr + p.Kv_d * v_dErr;
        
                tc = quadprog(Q, -A' * tau, [], [], [], [], -Tmax * ones(8, 1), Tmax * ones(8, 1), [], opts);
                thrustHistory(:, k) = tc;
        
                thr_e = alpha_e * tc + (1 - alpha_e) * thr_e;
                thr_a = alpha_m * thr_e + (1 - alpha_m) * thr_a;
        
                vLin = state(7:9);
                vRot = state(10:12);
                if ~isempty(p.polyDragLin)
                    F = zeros(3, 1);
                    for i = 1:3
                        F(i) = polyval(p.polyDragLin(i, :), vLin(i));
                    end
                else
                    F = -p.CdA .* abs(vLin) .* vLin;
                end
                if ~isempty(p.polyDragRot)
                    M_t = zeros(3, 1);
                    for i = 1:3
                        M_t(i) = polyval(p.polyDragRot(i, :), vRot(i));
                    end
                else
                    M_t = -p.D_rot * vRot;
                end
                TF = A * thr_a;
                F = F + TF(1:3);
                M_t = M_t + TF(4:6) + cross(p.com, [0; 0; (p.Fb - p.mass * 9.81)]);
        
                acc_lin = invM_lin * F;
                acc_ang = invM_ang * M_t;
                state(7:9) = state(7:9) + acc_lin * p.ts;
                state(10:12) = state(10:12) + acc_ang * p.ts;
                state(1:3) = state(1:3) + state(7:9) * p.ts + 0.5 * acc_lin * p.ts^2;
                state(4:6) = state(4:6) + state(10:12) * p.ts + 0.5 * acc_ang * p.ts^2;
                stateHistory(:, k) = state;
            end
        
            % Compute cost components with increased emphasis on translation axes
            thresholds = 0.05 * abs(p.desired); % 5% of desired value
            arrival_times = zeros(6, 1);
            for i = 1:6
                err = abs(errorHistory(i, :));
                idx = find(err < thresholds(i), 1, 'first');
                arrival_times(i) = timeVec(min(idx, N));
            end
            time_cost = mean(arrival_times) + 10 * max(arrival_times); % Scalar
        
            final_err = errorHistory(:, end);
            error_cost = 2 * mean(abs(final_err(1:3))) + mean(abs(final_err(4:6))) + 10 * max(abs(final_err)); % Double weight on translation
        
            max_overshoot = max(abs(errorHistory), [], 2) - abs(p.desired); % Vector of overshoots
            overshoot_cost = 2 * mean(max(max_overshoot(1:3, max_overshoot(1:3, :) > 0), 0)) + mean(max(max_overshoot(4:6, max_overshoot(4:6, :) > 0), 0)) ...
                           + 10 * max(max_overshoot(max_overshoot > 0), 0); % Double weight on translation
        
            % Penalty for large errors (exceeding 5% of desired)
            error_penalty = 0;
            for i = 1:6
                if any(abs(errorHistory(i, :)) > 1.05 * abs(p.desired(i)))
                    error_penalty = error_penalty + 100 * mean(abs(errorHistory(i, errorHistory(i, :) > 1.05 * abs(p.desired(i)))));
                end
            end
        
            % Weighted sum with adjusted priorities
            w1 = 0.5; w2 = 0.4; w3 = 0.1; % Increased overshoot weight, reduced time weight
            cost = w1 * overshoot_cost + w2 * error_cost + w3 * time_cost + error_penalty;
        
            % Debug output to check dimensions
            disp(['overshoot_cost size: ', mat2str(size(overshoot_cost))]);
            disp(['error_cost size: ', mat2str(size(error_cost))]);
            disp(['time_cost size: ', mat2str(size(time_cost))]);
            disp(['error_penalty size: ', mat2str(size(error_penalty))]);
            disp(['cost size: ', mat2str(size(cost))]);
        
            % Force scalar if not already
            cost = sum(cost(:));
        
            % Add penalty for instability
            if any(isnan(stateHistory(:))) || any(isinf(stateHistory(:)))
                cost = cost + 1000;
            end
        end

        % --- Simulation Execution ---
        function RunSim(app, ~, ~)
            % Run the ROV simulation with cascade PID control
            try
                app.simParams.mass = app.MassEdit.Value;
                app.simParams.Vdisp = app.VdispEdit.Value;
                app.simParams.maxThrust = app.MaxThrustEdit.Value;
                app.simParams.com = app.COMTable.Data';
                app.simParams.I = diag(app.InertiaTable.Data);
                app.simParams.added = diag(app.AddedMassTable.Data);
                app.simParams.r_pos = app.ThrusterConfigTable.Data(:, 1:3);
                app.simParams.dir_thr = app.ThrusterConfigTable.Data(:, 4:6);
                app.simParams.Kp = diag(app.GainsTable.Data(:, 1));
                app.simParams.Ki = diag(app.GainsTable.Data(:, 2));
                app.simParams.Kd = diag(app.GainsTable.Data(:, 3));
                app.simParams.Kv_p = diag(app.VelocityGainsTable.Data(:, 1));
                app.simParams.Kv_i = diag(app.VelocityGainsTable.Data(:, 2));
                app.simParams.Kv_d = diag(app.VelocityGainsTable.Data(:, 3));
                app.simParams.desired = app.DesiredPoseTable.Data';
                app.saveParametersToMat();
            catch ME
                uialert(app.UIFigure, sprintf('Invalid input: %s', ME.message), 'Input Error');
                return;
            end

            app.TabGroup.SelectedTab = app.SimulationTab;

            p = app.simParams;
            N = round(p.t_final / p.ts);
            app.timeVec = (0:N-1) * p.ts;
            app.stateHistory = zeros(12, N);
            app.thrustHistory = zeros(8, N);
            app.errorHistory = zeros(6, N);

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

            % 2D Plot
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

            state = zeros(12, 1);
            intErr = zeros(6, 1);
            prevErr = p.desired;
            v_intErr = zeros(6, 1);
            v_prevErr = zeros(6, 1);
            thr_e = zeros(8, 1);
            thr_a = zeros(8, 1);
            Tmax = p.maxThrust;
            tau_e = 0.05;
            tau_m = 0.15;
            alpha_e = p.ts / (tau_e + p.ts);
            alpha_m = p.ts / (tau_m + p.ts);
            arwScale = 0.1;

            app.errorHistory(:, 1) = p.desired - state(1:6);

            tic;
            try
                for k = 2:N
                    err = p.desired - state(1:6);
                    app.errorHistory(:, k) = err;
                    intErr = intErr + err * p.ts;
                    dErr = (err - prevErr) / p.ts;
                    prevErr = err;
                    v_desired = p.Kp * err + p.Ki * intErr + p.Kd * dErr;

                    v_err = v_desired - state(7:12);
                    v_intErr = v_intErr + v_err * p.ts;
                    v_dErr = (v_err - v_prevErr) / p.ts;
                    v_prevErr = v_err;
                    tau = p.Kv_p * v_err + p.Kv_i * v_intErr + p.Kv_d * v_dErr;

                    tc = quadprog(Q, -A' * tau, [], [], [], [], -Tmax * ones(8, 1), Tmax * ones(8, 1), [], opts);
                    app.thrustHistory(:, k) = tc;

                    thr_e = alpha_e * tc + (1 - alpha_e) * thr_e;
                    thr_a = alpha_m * thr_e + (1 - alpha_m) * thr_a;

                    vLin = state(7:9);
                    vRot = state(10:12);
                    if ~isempty(p.polyDragLin)
                        F = zeros(3, 1);
                        for i = 1:3
                            F(i) = polyval(p.polyDragLin(i, :), vLin(i));
                        end
                    else
                        F = -p.CdA .* abs(vLin) .* vLin;
                    end
                    if ~isempty(p.polyDragRot)
                        M_t = zeros(3, 1);
                        for i = 1:3
                            M_t(i) = polyval(p.polyDragRot(i, :), vRot(i));
                        end
                    else
                        M_t = -p.D_rot * vRot;
                    end
                    TF = A * thr_a;
                    F = F + TF(1:3);
                    M_t = M_t + TF(4:6) + cross(p.com, [0; 0; (p.Fb - p.mass * 9.81)]);

                    acc_lin = invM_lin * F;
                    acc_ang = invM_ang * M_t;
                    state(7:9) = state(7:9) + acc_lin * p.ts;
                    state(10:12) = state(10:12) + acc_ang * p.ts;
                    state(1:3) = state(1:3) + state(7:9) * p.ts + 0.5 * acc_lin * p.ts^2;
                    state(4:6) = state(4:6) + state(10:12) * p.ts + 0.5 * acc_ang * p.ts^2;
                    app.stateHistory(:, k) = state;

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

        % --- Results Visualization ---
        function updateResultsTab(app)
            cla(app.UIAxesError);
            cla(app.UIAxesControlEffort);

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

            resultsStr = sprintf('Simulation Results:\n\n');
            thresholds = 0.05 * max(abs(app.simParams.desired), 0.1);
            arrival_times = zeros(6, 1);
            final_err = app.errorHistory(:, end);
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
            resultsStr = [resultsStr, sprintf('\nFinal Steady-State Error:\n')];
            for i = 1:6
                resultsStr = [resultsStr, sprintf('  %s: %.4f\n', coords{i}, final_err(i))];
            end

            N = length(app.timeVec);
            last_10pct = round(0.9 * N):N;
            state_variance = var(app.stateHistory(:, last_10pct), 0, 2);
            resultsStr = [resultsStr, sprintf('\nState Variance (Last 10%% of Simulation):\n')];
            for i = 1:6
                resultsStr = [resultsStr, sprintf('  %s: %.4f\n', coords{i}, state_variance(i))];
            end

            max_overshoot = max(abs(app.errorHistory), [], 2);
            resultsStr = [resultsStr, sprintf('\nMaximum Overshoot:\n')];
            for i = 1:6
                resultsStr = [resultsStr, sprintf('  %s: %.4f\n', coords{i}, max_overshoot(i))];
            end

            energy = sum(sum(app.thrustHistory.^2)) * app.simParams.ts;
            resultsStr = [resultsStr, sprintf('\nTotal Control Effort: %.2f N²s\n', energy)];

            if any(isinf(arrival_times(4:6)))
                resultsStr = [resultsStr, sprintf('\nWarning: Orientation (Roll, Pitch, Yaw) may be unstable. Consider increasing angular PID gains or checking thruster configuration.\n')];
            end

            app.ResultsTextArea.Value = resultsStr;
        end

        % --- Parameter Updates ---
        function updateParameter(app, field, value, tableName)
            % Update simParams and save to .mat when UI inputs change
            try
                if strcmp(tableName, 'InertiaTable')
                    app.simParams.I = diag(value);
                elseif strcmp(tableName, 'AddedMassTable')
                    app.simParams.added = diag(value);
                elseif strcmp(tableName, 'DrotTable')
                    app.simParams.D_rot = diag(value);
                elseif strcmp(tableName, 'CdATable')
                    app.simParams.CdA = value';
                elseif strcmp(tableName, 'COMTable')
                    app.simParams.com = value';
                elseif strcmp(tableName, 'GainsTable')
                    app.simParams.Kp = diag(value(:,1));
                    app.simParams.Ki = diag(value(:,2));
                    app.simParams.Kd = diag(value(:,3));
                elseif strcmp(tableName, 'VelocityGainsTable')
                    app.simParams.Kv_p = diag(value(:,1));
                    app.simParams.Kv_i = diag(value(:,2));
                    app.simParams.Kv_d = diag(value(:,3));
                elseif strcmp(tableName, 'ThrusterConfigTable')
                    app.simParams.r_pos = value(:,1:3);
                    app.simParams.dir_thr = value(:,4:6);
                elseif strcmp(tableName, 'DesiredPoseTable')
                    app.simParams.desired = value';
                else
                    app.simParams.(field) = value;
                    if strcmp(field, 'Vdisp')
                        app.simParams.Fb = 1000 * value * 9.81;
                    end
                end
                app.saveParametersToMat();
            catch ME
                uialert(app.UIFigure, sprintf('Error updating parameter %s: %s', field, ME.message), 'Error', 'Icon', 'error');
            end
        end

        % --- Reset Simulation ---
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

    % --- GUI Creation ---
    methods (Access = private)
        function createComponents(app)
            % Create figure with fixed size
            app.UIFigure = uifigure('Name', 'ROV Simulation Platform', 'Position', [100 100 1400 900]);

            % TabGroup with adjusted size to accommodate buttons at top
            app.TabGroup = uitabgroup(app.UIFigure, 'Position', [10 50 1380 800]);
            
            % Create tabs
            app.ParametersTab = uitab(app.TabGroup, 'Title', 'Parameters');
            app.SimulationTab = uitab(app.TabGroup, 'Title', 'Simulation');
            app.VisualizationTab = uitab(app.TabGroup, 'Title', '3D Visualization');
            app.ResultsTab = uitab(app.TabGroup, 'Title', 'Results');
            app.DragFittingTab = uitab(app.TabGroup, 'Title', 'Drag Fitting');
            app.PIDTuningTab = uitab(app.TabGroup, 'Title', 'PID Tuning');

            % Buttons positioned at top, above TabGroup
            app.LoadDragButton = uibutton(app.UIFigure, 'Text', 'Load Drag CSV', 'Position', [20 850 120 25], ...
                                          'ButtonPushedFcn', @(s, e) app.LoadDragFromCSV(s, e));
            app.RunSimButton = uibutton(app.UIFigure, 'Text', 'Run Simulation', 'Position', [150 850 120 25], ...
                                        'ButtonPushedFcn', @(s, e) app.RunSim(s, e));
            app.ResetButton = uibutton(app.UIFigure, 'Text', 'Reset', 'Position', [280 850 120 25], ...
                                       'ButtonPushedFcn', @(s, e) app.Reset(s, e));
            app.FitDragButton = uibutton(app.UIFigure, 'Text', 'Fit Drag Polynomials', 'Position', [410 850 120 25], ...
                                         'ButtonPushedFcn', @(s, e) app.FitDragPolynomials(s, e));
            app.ForceLoadMatButton = uibutton(app.UIFigure, 'Text', 'Force Load .mat', 'Position', [540 850 120 25], ...
                                              'ButtonPushedFcn', @(s, e) app.forceLoadParameters());
            app.ResetDefaultsButton = uibutton(app.UIFigure, 'Text', 'Reset to Defaults', 'Position', [670 850 120 25], ...
                                               'ButtonPushedFcn', @(s, e) app.resetToDefaults());

            % Parameters Tab Components (scaled to fit TabGroup height of 800)
            app.BasicPropsPanel = uipanel(app.ParametersTab, 'Title', 'Basic Properties', ...
                                          'Position', [20 650 350 120], 'FontWeight', 'bold');
            app.MassLabel = uilabel(app.BasicPropsPanel, 'Text', 'Mass (kg):', ...
                                    'Position', [10 80 80 22], 'FontWeight', 'bold');
            app.MassEdit = uieditfield(app.BasicPropsPanel, 'numeric', 'Position', [100 80 100 22], ...
                                       'Value', 44.05438, 'Limits', [0 Inf], ...
                                       'ValueChangedFcn', @(s, e) app.updateParameter('mass', s.Value, 'MassEdit'));
            app.VdispLabel = uilabel(app.BasicPropsPanel, 'Text', 'Volume (m³):', ...
                                     'Position', [10 50 80 22], 'FontWeight', 'bold');
            app.VdispEdit = uieditfield(app.BasicPropsPanel, 'numeric', 'Position', [100 50 100 22], ...
                                        'Value', 4.4054e-2, 'Limits', [0 Inf], ...
                                        'ValueChangedFcn', @(s, e) app.updateParameter('Vdisp', s.Value, 'VdispEdit'));
            app.MaxThrustLabel = uilabel(app.BasicPropsPanel, 'Text', 'Max Thrust (N):', ...
                                         'Position', [10 20 100 22], 'FontWeight', 'bold');
            app.MaxThrustEdit = uieditfield(app.BasicPropsPanel, 'numeric', 'Position', [100 20 100 22], ...
                                            'Value', 40, 'Limits', [0 Inf], ...
                                            'ValueChangedFcn', @(s, e) app.updateParameter('maxThrust', s.Value, 'MaxThrustEdit'));

            app.COMPanel = uipanel(app.ParametersTab, 'Title', 'Center of Mass (m)', ...
                                   'Position', [20 510 350 100], 'FontWeight', 'bold');
            app.COMTable = uitable(app.COMPanel, 'Units', 'normalized', 'Position', [0 0 1 1], ...
                                   'ColumnName', {'X', 'Y', 'Z'}, 'ColumnEditable', true, 'ColumnWidth', 'auto', ...
                                   'Data', [0 0 0], ...
                                   'CellEditCallback', @(s, e) app.updateParameter('com', s.Data, 'COMTable'));

            app.InertiaPanel = uipanel(app.ParametersTab, 'Title', 'Inertia Matrix Diagonal (kg m²)', ...
                                       'Position', [20 380 350 100], 'FontWeight', 'bold');
            app.InertiaTable = uitable(app.InertiaPanel, 'Units', 'normalized', 'Position', [0 0 1 1], ...
                                       'ColumnName', {'Ixx', 'Iyy', 'Izz'}, 'ColumnEditable', true, 'ColumnWidth', 'auto', ...
                                       'Data', [0 0 0], ...
                                       'CellEditCallback', @(s, e) app.updateParameter('I', s.Data, 'InertiaTable'));

            app.AddedMassPanel = uipanel(app.ParametersTab, 'Title', 'Added Mass Diagonal', ...
                                         'Position', [20 250 350 100], 'FontWeight', 'bold');
            app.AddedMassTable = uitable(app.AddedMassPanel, 'Units', 'normalized', 'Position', [0 0 1 1], ...
                                         'ColumnName', {'Am11', 'Am22', 'Am33', 'Am44', 'Am55', 'Am66'}, ...
                                         'ColumnEditable', true, 'ColumnWidth', 'auto', ...
                                         'Data', [0 0 0 0 0 0], ...
                                         'CellEditCallback', @(s, e) app.updateParameter('added', s.Data, 'AddedMassTable'));

            app.DragPanel = uipanel(app.ParametersTab, 'Title', 'Drag Coefficients', ...
                                    'Position', [20 60 350 170], 'FontWeight', 'bold');
            uilabel(app.DragPanel, 'Text', 'CdA (Linear Drag):', 'Position', [10 120 150 22], 'FontWeight', 'bold');
            app.CdATable = uitable(app.DragPanel, 'Units', 'normalized', 'Position', [0 0.5 1 0.5], ...
                                   'ColumnName', {'CdAx', 'CdAy', 'CdAz'}, 'ColumnEditable', true, 'ColumnWidth', 'auto', ...
                                   'Data', [0 0 0], ...
                                   'CellEditCallback', @(s, e) app.updateParameter('CdA', s.Data, 'CdATable'));
            uilabel(app.DragPanel, 'Text', 'Rotational Drag:', 'Position', [10 40 150 22], 'FontWeight', 'bold');
            app.DrotTable = uitable(app.DragPanel, 'Units', 'normalized', 'Position', [0 0 1 0.4], ...
                                    'ColumnName', {'Dr11', 'Dr22', 'Dr33'}, 'ColumnEditable', true, 'ColumnWidth', 'auto', ...
                                    'Data', [0 0 0], ...
                                    'CellEditCallback', @(s, e) app.updateParameter('D_rot', s.Data, 'DrotTable'));

            app.ThrusterPanel = uipanel(app.ParametersTab, 'Title', 'Thruster Configuration', ...
                                        'Position', [390 60 500 350], 'FontWeight', 'bold');
            app.ThrusterConfigTable = uitable(app.ThrusterPanel, 'Units', 'normalized', 'Position', [0 0 1 1], ...
                                              'ColumnName', {'Pos X', 'Pos Y', 'Pos Z', 'Dir X', 'Dir Y', 'Dir Z'}, ...
                                              'RowName', {'T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7', 'T8'}, ...
                                              'ColumnEditable', true, 'ColumnWidth', 'auto', ...
                                              'Data', zeros(8, 6), ...
                                              'CellEditCallback', @(s, e) app.updateParameter('r_pos', s.Data, 'ThrusterConfigTable'));

            app.ControlPanel = uipanel(app.ParametersTab, 'Title', 'Control Parameters', ...
                                       'Position', [910 60 450 350], 'FontWeight', 'bold');
            uilabel(app.ControlPanel, 'Text', 'Pose PID Gains:', 'Position', [10 300 100 22], 'FontWeight', 'bold');
            app.GainsTable = uitable(app.ControlPanel, 'Units', 'normalized', 'Position', [0 0.55 1 0.35], ...
                                     'ColumnName', {'Kp', 'Ki', 'Kd'}, 'RowName', {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'}, ...
                                     'ColumnEditable', true, 'ColumnWidth', 'auto', ...
                                     'Data', zeros(6, 3), ...
                                     'CellEditCallback', @(s, e) app.updateParameter('gains', s.Data, 'GainsTable'));
            uilabel(app.ControlPanel, 'Text', 'Velocity PID Gains:', 'Position', [10 160 100 22], 'FontWeight', 'bold');
            app.VelocityGainsTable = uitable(app.ControlPanel, 'Units', 'normalized', 'Position', [0 0.3 1 0.25], ...
                                             'ColumnName', {'Kv_p', 'Kv_i', 'Kv_d'}, 'RowName', {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'}, ...
                                             'ColumnEditable', true, 'ColumnWidth', 'auto', ...
                                             'Data', zeros(6, 3), ...
                                             'CellEditCallback', @(s, e) app.updateParameter('gains', s.Data, 'VelocityGainsTable'));
            uilabel(app.ControlPanel, 'Text', 'Desired Pose:', 'Position', [10 80 100 22], 'FontWeight', 'bold');
            app.DesiredPoseTable = uitable(app.ControlPanel, 'Units', 'normalized', 'Position', [0 0 1 0.25], ...
                                           'ColumnName', {'X (m)', 'Y (m)', 'Z (m)', 'Roll (rad)', 'Pitch (rad)', 'Yaw (rad)'}, ...
                                           'ColumnEditable', true, 'Data', [20, 20, 20, 0, 0, 0], 'ColumnWidth', 'auto', ...
                                           'CellEditCallback', @(s, e) app.updateParameter('desired', s.Data, 'DesiredPoseTable'));

            % Simulation Tab Components
            app.UIAxes2D = uiaxes(app.SimulationTab, 'Position', [50 20 1280 720], 'Box', 'on');
            title(app.UIAxes2D, 'Real-time Simulation Results');
            xlabel(app.UIAxes2D, 'Time (s)');
            ylabel(app.UIAxes2D, 'State Variables');

            % Visualization Tab Components
            app.UIAxesThruster3D = uiaxes(app.VisualizationTab, 'Position', [20 400 420 350], 'Box', 'on');
            title(app.UIAxesThruster3D, '3D Thruster Forces');
            app.UIAxesTrajectory3D = uiaxes(app.VisualizationTab, 'Position', [460 400 420 350], 'Box', 'on');
            title(app.UIAxesTrajectory3D, '3D Trajectory');
            app.UIAxesHull3D = uiaxes(app.VisualizationTab, 'Position', [900 400 420 350], 'Box', 'on');
            title(app.UIAxesHull3D, '3D Hull Visualization');

            % Results Tab Components
            app.UIAxesError = uiaxes(app.ResultsTab, 'Position', [20 400 640 350], 'Box', 'on');
            title(app.UIAxesError, 'Error Over Time');
            xlabel(app.UIAxesError, 'Time (s)');
            ylabel(app.UIAxesError, 'Error');
            app.UIAxesControlEffort = uiaxes(app.ResultsTab, 'Position', [680 400 640 350], 'Box', 'on');
            title(app.UIAxesControlEffort, 'Control Effort (Thrust Magnitude)');
            xlabel(app.UIAxesControlEffort, 'Time (s)');
            ylabel(app.UIAxesControlEffort, 'Thrust (N)');
            app.ResultsTextArea = uitextarea(app.ResultsTab, 'Position', [20 20 1280 350], ...
                                             'FontSize', 12, 'Editable', 'off');

            % Drag Fitting Tab Components
            app.NumPointsLabel = uilabel(app.DragFittingTab, 'Text', 'Number of Points:', ...
                                         'Position', [20 750 120 22], 'FontWeight', 'bold');
            app.NumPointsEdit = uieditfield(app.DragFittingTab, 'numeric', 'Position', [150 750 100 22], ...
                                            'Value', 11, 'Limits', [6 Inf], 'ValueChangedFcn', @(s, e) app.updateDragTables());
            app.LinearDragTable = uitable(app.DragFittingTab, 'Position', [20 520 600 200], ...
                                          'ColumnName', {'A', 'B', 'C', 'D'}, ...
                                          'ColumnEditable', true, 'ColumnWidth', 'auto');
            app.RotationalDragTable = uitable(app.DragFittingTab, 'Position', [650 520 600 200], ...
                                              'ColumnName', {'A', 'B', 'C', 'D'}, ...
                                              'ColumnEditable', true, 'ColumnWidth', 'auto');
            app.UIAxesLinearDrag = uiaxes(app.DragFittingTab, 'Position', [20 270 600 200], 'Box', 'on');
            title(app.UIAxesLinearDrag, 'Linear Drag Fit');
            app.UIAxesRotationalDrag = uiaxes(app.DragFittingTab, 'Position', [650 270 600 200], 'Box', 'on');
            title(app.UIAxesRotationalDrag, 'Rotational Drag Fit');

            % PID Tuning Tab Components
            app.TuningGoalDropDown = uidropdown(app.PIDTuningTab, 'Items', {'Optimized Tuning'}, ...
                                                'Value', 'Optimized Tuning', 'Position', [50 650 200 22]);
            app.TunePIDButton = uibutton(app.PIDTuningTab, 'Text', 'Tune PID', 'Position', [50 610 100 25], ...
                                         'ButtonPushedFcn', @(s, e) app.TunePIDAutomatically(s, e));
        end
    end

    % --- Public Methods ---
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