
classdef ROVSimGUI < matlab.apps.AppBase
    % ROVSimGUI: GUI for 6-DOF ROV Simulation with .mat parameter storage, Cascade PID control, and OpenAI API integration

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
    PauseButton              matlab.ui.control.Button % New: Pause/Resume simulation
    AccelerateButton         matlab.ui.control.Button % New: Increase simulation speed
    DecelerateButton         matlab.ui.control.Button % New: Decrease simulation speed
    StopButton               matlab.ui.control.Button % New: Stop simulation
    FitDragButton            matlab.ui.control.Button
    ForceLoadMatButton       matlab.ui.control.Button
    ResetDefaultsButton      matlab.ui.control.Button
    ScenarioSwitch           matlab.ui.control.Switch % Switch for Control/Test scenarios
    
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
    
    COBPanel                 matlab.ui.container.Panel
    COBTable                 matlab.ui.control.Table
    
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
    UIAxesPath3D             matlab.ui.control.UIAxes % New: Replaces UIAxesHull3D for path animation
    
    % Results Tab Controls
    UIAxesError              matlab.ui.control.UIAxes
    UIAxesControlEffort      matlab.ui.control.UIAxes
    ResultsTextArea          matlab.ui.control.TextArea
    AIChatInputLabel         matlab.ui.control.Label
    AIChatInputTextArea      matlab.ui.control.TextArea
    AISendButton             matlab.ui.control.Button
    AIResponseTextArea       matlab.ui.control.TextArea

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
    TuningProgressLabel      matlab.ui.control.Label
    TuningResultsTextArea    matlab.ui.control.TextArea
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
    apiKey                   % OpenAI API key
    scenarioMode             % 'Control' or 'Test'
    maxSpeeds                % Maximum speeds for Test scenario (6x2)
    simControl               % New: Simulation control state (struct with isPaused, speedScale, isStopped)
end

    % === Public Methods ===
    methods (Access = public)
        % Constructor
        function app = ROVSimGUI
            % Initialize API key (replace with your OpenAI API key or use getenv)
            app.apiKey = ''; % TODO: Replace with your key
            app.scenarioMode = 'Control'; % Default to Control scenario
            
            % Create and configure components
            createComponents(app);
            
            % Initialize properties
            app.matFile = fullfile(fileparts(mfilename('fullpath')), 'rov_parameters.mat');
            startup(app);
            
            % Register the app with App Designer
            registerApp(app, app.UIFigure);
        end

        % App Deletion
        function delete(app)
            % Clean up when app is closed
            try
                delete(app.UIFigure);
            catch
                % Suppress errors during deletion
            end
        end
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
        
            % Initialize simulation control state
            app.simControl = struct('isPaused', false, 'speedScale', 1.0, 'isStopped', false);
        
            % Load parameters from .mat or use defaults
            app.forceLoadParameters();
        end
        function togglePause(app)
            app.simControl.isPaused = ~app.simControl.isPaused;
            if app.simControl.isPaused
                app.PauseButton.Text = 'Resume';
            else
                app.PauseButton.Text = 'Pause';
            end
        end
        
        function accelerateSim(app)
            app.simControl.speedScale = min(app.simControl.speedScale * 1.5, 5.0); % Cap at 5x speed
        end
        
        function decelerateSim(app)
            app.simControl.speedScale = max(app.simControl.speedScale / 1.5, 0.2); % Floor at 0.2x speed
        end
        
        function stopSim(app)
            app.simControl.isStopped = true;
end

        % --- Force Load Parameters from .mat ---
        function forceLoadParameters(app)
            % Force reload parameters from .mat file
            if isfile(app.matFile)
                try
                    loadedData = load(app.matFile, 'simParams');
                    simParams = loadedData.simParams;
                    % Define required fields
                    requiredFields = {'mass', 'Vdisp', 'maxThrust', 'com', 'cob', 'I', 'added', ...
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
                        simParams = app.simParams;
                        save(app.matFile, 'simParams');
                    end
                catch ME
                    uialert(app.UIFigure, sprintf('Error loading .mat: %s. Initializing defaults.', ME.message), 'Error', 'Icon', 'error');
                    app.initializeDefaultParameters();
                    simParams = app.simParams;
                    save(app.matFile, 'simParams');
                end
            else
                app.initializeDefaultParameters();
                simParams = app.simParams;
                save(app.matFile, 'simParams');
            end

            % Populate UI components with loaded or default parameters
            try
                app.MassEdit.Value = app.simParams.mass;
                app.VdispEdit.Value = app.simParams.Vdisp;
                app.MaxThrustEdit.Value = app.simParams.maxThrust;
                app.COMTable.Data = app.simParams.com';
                app.COBTable.Data = app.simParams.cob';
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
                app.TuningGoalDropDown.Items = {'Fast Response', 'Minimal Overshoot', 'Balanced Performance'};
                app.TuningGoalDropDown.Value = 'Balanced Performance';
                app.TuningResultsTextArea.Value = '';
                app.AIChatInputTextArea.Value = {'Enter your request (e.g., "Analyze high yaw error" or "Suggest PID tuning improvements").'};
                app.AIResponseTextArea.Value = {''};
                app.ScenarioSwitch.Value = app.scenarioMode;
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
            p.cob = [0; 0; 0.05]; % Center of Buoyancy (m)
            p.I = diag([1.990993, 15.350344, 15.774167]); % Inertia matrix (diagonal, kg·m²)
            p.added = diag([5, 5, 10, 1, 1, 1]); % Added mass matrix (diagonal)
            p.CdA = [550; 550; 550]; % Linear drag coefficients
            p.D_rot = diag([5, 5, 5]); % Rotational drag matrix (diagonal)
            p.r_pos = [0.2, 0.1, 0.1; 0.2, -0.1, 0.1; -0.2, -0.1, 0.1; -0.2, 0.1, 0.1;
                       0.2, 0.1, 0; 0.2, -0.1, 0; -0.2, -0.1, 0; -0.2, 0.1, 0]; % Thruster positions (m)
            deg = [nan, nan, nan, nan, -45, 45, 135, -135];
            p.dir_thr = [repmat([0, 0, 1], 4, 1); [cosd(deg(5:8))', sind(deg(5:8))', zeros(4, 1)]]; % Thruster directions
            p.Kp = diag([2, 2, 2, 0.5, 0.5, 0.5]); % Pose control
            p.Ki = diag([0.01, 0.01, 0.01, 0, 0, 0]);
            p.Kd = diag([1, 1, 1, 0.5, 0.5, 0.5]);
            p.Kv_p = diag([5, 5, 5, 2, 2, 2]); % Velocity control
            p.Kv_i = diag([0.05, 0.05, 0.05, 0, 0, 0]);
            p.Kv_d = diag([2, 2, 2, 1, 1, 1]);
            p.desired = [20; 20; 20; 0; 0; 0]; % Desired pose
            p.polyDragLin = [];
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
                uialert(app.UIFigure, sprintf('Failed to save parameters.mat: %s', ME.message), 'Error', 'Icon', 'error');
            end
        end

        % --- Reset to Default Parameters ---
        function resetToDefaults(app, ~, ~)
            % Reset parameters to defaults and update UI
            try
                app.initializeDefaultParameters();
                app.saveParametersToMat();
                app.forceLoadParameters();
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

        % --- PID Tuning ---
        function TunePIDAutomatically(app, ~, ~)
            % Automatically tune cascade PID gains based on user-selected goal
            try
                if strcmp(app.scenarioMode, 'Test')
                    uialert(app.UIFigure, 'PID tuning is only available in Control scenario.', 'Error', 'Icon', 'error');
                    return;
                end
                app.TabGroup.SelectedTab = app.PIDTuningTab;
                app.TuningProgressLabel.Text = 'Initializing PID tuning...';
                drawnow;

                p = app.simParams;
                
                lb_pose = [0.1 * ones(3,1); 0.05 * ones(3,1); zeros(6,1); 0.1 * ones(6,1)];
                ub_pose = [10 * ones(3,1); 2 * ones(3,1); 0.5 * ones(6,1); 5 * ones(6,1)];
                lb_vel = [0.5 * ones(3,1); 0.2 * ones(3,1); zeros(6,1); 0.5 * ones(6,1)];
                ub_vel = [20 * ones(3,1); 5 * ones(3,1); 1 * ones(6,1); 10 * ones(6,1)];
                lb = [lb_pose; lb_vel];
                ub = [ub_pose; ub_vel];

                x0 = [diag(p.Kp); diag(p.Ki); diag(p.Kd); diag(p.Kv_p); diag(p.Kv_i); diag(p.Kv_d)];

                switch app.TuningGoalDropDown.Value
                    case 'Fast Response'
                        w_settle = 0.6; w_overshoot = 0.2; w_iae = 0.2; w_energy = 0.1;
                    case 'Minimal Overshoot'
                        w_settle = 0.2; w_overshoot = 0.6; w_iae = 0.1; w_energy = 0.1;
                    case 'Balanced Performance'
                        w_settle = 0.35; w_overshoot = 0.35; w_iae = 0.2; w_energy = 0.1;
                    otherwise
                        w_settle = 0.35; w_overshoot = 0.35; w_iae = 0.2; w_energy = 0.1;
                end

                opts = optimoptions('fmincon', ...
                    'Display', 'iter', ...
                    'MaxIterations', 100, ...
                    'MaxFunctionEvaluations', 1000, ...
                    'Algorithm', 'interior-point', ...
                    'StepTolerance', 1e-8, ...
                    'ConstraintTolerance', 1e-8, ...
                    'OptimalityTolerance', 1e-8);

                objFun = @(x) evaluatePID(app, x, w_settle, w_overshoot, w_iae, w_energy);

                app.TuningProgressLabel.Text = 'Running PID tuning optimization...';
                drawnow;
                tic;
                [x_opt, fval, exitflag, output] = fmincon(objFun, x0, [], [], [], [], lb, ub, [], opts);
                elapsed = toc;

                app.simParams.Kp = diag(x_opt(1:6));
                app.simParams.Ki = diag(x_opt(7:12));
                app.simParams.Kd = diag(x_opt(13:18));
                app.simParams.Kv_p = diag(x_opt(19:24));
                app.simParams.Kv_i = diag(x_opt(25:30));
                app.simParams.Kv_d = diag(x_opt(31:36));

                app.GainsTable.Data = [diag(app.simParams.Kp), diag(app.simParams.Ki), diag(app.simParams.Kd)];
                app.VelocityGainsTable.Data = [diag(app.simParams.Kv_p), diag(app.simParams.Kv_i), diag(app.simParams.Kv_d)];
                app.saveParametersToMat();

                resultsStr = sprintf('PID Tuning Results:\n\n');
                resultsStr = [resultsStr, sprintf('Tuning Goal: %s\n', app.TuningGoalDropDown.Value)];
                resultsStr = [resultsStr, sprintf('Optimization Time: %.2f s\n', elapsed)];
                resultsStr = [resultsStr, sprintf('Objective Function Value: %.4f\n', fval)];
                resultsStr = [resultsStr, sprintf('Iterations: %d\n', output.iterations)];
                resultsStr = [resultsStr, sprintf('Exit Flag: %d (%s)\n', exitflag, output.message)];
                resultsStr = [resultsStr, sprintf('\nOptimized Pose PID Gains:\n')];
                coords = {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'};
                for i = 1:6
                    resultsStr = [resultsStr, sprintf('  %s: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', ...
                        coords{i}, x_opt(i), x_opt(i+6), x_opt(i+12))];
                end
                resultsStr = [resultsStr, sprintf('\nOptimized Velocity PID Gains:\n')];
                for i = 1:6
                    resultsStr = [resultsStr, sprintf('  %s: Kv_p=%.4f, Kv_i=%.4f, Kv_d=%.4f\n', ...
                        coords{i}, x_opt(i+18), x_opt(i+24), x_opt(i+30))];
                end
                app.TuningResultsTextArea.Value = resultsStr;
                app.TuningProgressLabel.Text = 'Tuning completed successfully.';
                uialert(app.UIFigure, 'PID tuning completed successfully.', 'Success', 'Icon', 'success');

                app.TuningProgressLabel.Text = 'Running simulation with optimized gains...';
                drawnow;
                app.RunSim([], []);

            catch ME
                app.TuningProgressLabel.Text = 'Tuning failed.';
                uialert(app.UIFigure, sprintf('PID tuning error: %s', ME.message), 'Error', 'Icon', 'error');
            end
        end

        % Helper function to evaluate PID performance
        function cost = evaluatePID(app, x, w_settle, w_overshoot, w_iae, w_energy)
            app.simParams.Kp = diag(x(1:6));
            app.simParams.Ki = diag(x(7:12));
            app.simParams.Kd = diag(x(13:18));
            app.simParams.Kv_p = diag(x(19:24));
            app.simParams.Kv_i = diag(x(25:30));
            app.simParams.Kv_d = diag(x(31:36));

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

                app.errorHistory(:, 1) = p.desired - state(1:6);

                for k = 2:N
                    phi = state(4); th = state(5); psi = state(6);
                    Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
                    Ry = [cos(th), 0, sin(th); 0, 1, 0; -sin(th), 0, cos(th)];
                    Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
                    R = Rz * Ry * Rx;

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
                    buoyancy_force = [0; 0; p.Fb];
                    weight_force = [0; 0; -p.mass * 9.81];
                    r_cob_world = R * p.cob;
                    r_com_world = R * p.com;
                    M_buoyancy = cross(R' * r_cob_world, R' * buoyancy_force);
                    F = F + R' * (buoyancy_force + weight_force);
                    M_t = M_t + TF(4:6) + M_buoyancy;

                    acc_lin = invM_lin * F;
                    acc_ang = invM_ang * M_t;
                    state(7:9) = state(7:9) + acc_lin * p.ts;
                    state(10:12) = state(10:12) + acc_ang * p.ts;
                    state(1:3) = state(1:3) + state(7:9) * p.ts + 0.5 * acc_lin * p.ts^2;
                    state(4:6) = state(4:6) + state(10:12) * p.ts + 0.5 * acc_ang * p.ts^2;
                    app.stateHistory(:, k) = state;
                end

                desired_norm = max(abs(p.desired), [0.1; 0.1; 0.1; 0.01; 0.01; 0.01]);
                thresholds = 0.05 * desired_norm;
                settling_times = zeros(6, 1);
                overshoots = max(abs(app.errorHistory), [], 2);
                iae = sum(abs(app.errorHistory), 2) * p.ts;
                energy = sum(sum(app.thrustHistory.^2)) * p.ts;

                for i = 1:6
                    err = abs(app.errorHistory(i, :));
                    idx = find(err < thresholds(i), 5, 'first');
                    if length(idx) == 5 && all(diff(idx) == 1) && idx(1) > 1
                        settling_times(i) = app.timeVec(idx(1));
                    else
                        settling_times(i) = p.t_final;
                    end
                end

                settle_cost = mean(settling_times / p.t_final);
                overshoot_cost = mean(overshoots ./ desired_norm);
                iae_cost = mean(iae ./ desired_norm);
                energy_cost = energy / (p.maxThrust^2 * p.t_final);

                cost = w_settle * settle_cost + w_overshoot * overshoot_cost + ...
                       w_iae * iae_cost + w_energy * energy_cost;

                if any(isnan(app.stateHistory(:)) | isinf(app.stateHistory(:)))
                    cost = cost + 1e6;
                end

                app.TuningProgressLabel.Text = sprintf('Evaluating: Cost = %.4f (Settle: %.2f, Overshoot: %.2f, IAE: %.2f, Energy: %.2f)', ...
                    cost, settle_cost, overshoot_cost, iae_cost, energy_cost);
                drawnow;

            catch ME
                warning('ROVSimGUI:SimFailed', 'Simulation failed: %s', ME.message);
                cost = 1e6;
            end
        end

        % --- Simulation Execution ---
        function RunSim(app, ~, ~)
    try
        app.simParams.mass = app.MassEdit.Value;
        app.simParams.Vdisp = app.VdispEdit.Value;
        app.simParams.maxThrust = app.MaxThrustEdit.Value;
        app.simParams.com = app.COMTable.Data';
        app.simParams.cob = app.COBTable.Data';
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

    % Reset simulation control state
    app.simControl.isPaused = false;
    app.simControl.speedScale = 1.0;
    app.simControl.isStopped = false;
    app.PauseButton.Text = 'Pause';

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
    title(ax2, sprintf('%s Scenario Results', app.scenarioMode)); legend(ax2, 'show'); hold(ax2, 'off');

    axT = app.UIAxesThruster3D; cla(axT); hold(axT, 'on');
    quiv = gobjects(8, 1);
    for i = 1:8
        quiv(i) = quiver3(axT, 0, 0, 0, 0, 0, 0, 'r', 'MaxHeadSize', 0.5, 'DisplayName', sprintf('Thruster %d', i));
    end
    quiv_buoy = quiver3(axT, 0, 0, 0, 0, 0, 0, 'b', 'MaxHeadSize', 0.5, 'DisplayName', 'Buoyancy');
    quiv_weight = quiver3(axT, 0, 0, 0, 0, 0, 0, 'k', 'MaxHeadSize', 0.5, 'DisplayName', 'Weight');
    grid(axT, 'on'); daspect(axT, [1 1 1]); title(axT, '3D Thruster Forces with Buoyancy and Weight'); legend(axT, 'show'); hold(axT, 'off');

    axTraj = app.UIAxesTrajectory3D; cla(axTraj); hold(axTraj, 'on');
    hTraj = animatedline(axTraj, 'Color', 'b', 'LineWidth', 1.5, 'DisplayName', 'Trajectory');
    if strcmp(app.scenarioMode, 'Control')
        plot3(axTraj, p.desired(1), p.desired(2), p.desired(3), 'rx', 'MarkerSize', 10, 'DisplayName', 'Desired Pose');
    end
    grid(axTraj, 'on'); daspect(axTraj, [1 1 1]); title(axTraj, 'Trajectory'); legend(axTraj, 'show'); hold(axTraj, 'off');

    axPath = app.UIAxesPath3D; cla(axPath); hold(axPath, 'on');
    app.tr = hgtransform('Parent', axPath);
    patch('Faces', app.faces, 'Vertices', app.vertices, 'FaceColor', 'cyan', 'FaceAlpha', 0.3, 'Parent', app.tr, 'DisplayName', 'ROV Hull');
    hPath = animatedline(axPath, 'Color', 'b', 'LineWidth', 1.5, 'DisplayName', 'Path');
    if strcmp(app.scenarioMode, 'Control')
        plot3(axPath, p.desired(1), p.desired(2), p.desired(3), 'rx', 'MarkerSize', 10, 'DisplayName', 'Desired Pose');
    end
    grid(axPath, 'on'); daspect(axPath, [1 1 1]); title(axPath, 'ROV Path Animation'); legend(axPath, 'show'); hold(axPath, 'off');

    state = zeros(12, 1);
    thr_e = zeros(8, 1);
    thr_a = zeros(8, 1);
    Tmax = p.maxThrust;
    tau_e = 0.05;
    tau_m = 0.15;
    alpha_e = p.ts / (tau_e + p.ts);
    alpha_m = p.ts / (tau_m + p.ts);
    arwScale = 0.1;
    forceScale = 0.001;

    if strcmp(app.scenarioMode, 'Control')
        intErr = zeros(6, 1);
        prevErr = p.desired;
        v_intErr = zeros(6, 1);
        v_prevErr = zeros(6, 1);
        app.errorHistory(:, 1) = p.desired - state(1:6);

        tic;
        try
            for k = 2:N
                if app.simControl.isStopped
                    uialert(app.UIFigure, 'Simulation stopped by user.', 'Info', 'Icon', 'info');
                    return;
                end
                while app.simControl.isPaused
                    pause(0.1); % Check every 0.1s for resume
                    if app.simControl.isStopped
                        uialert(app.UIFigure, 'Simulation stopped by user.', 'Info', 'Icon', 'info');
                        return;
                    end
                end

                phi = state(4); th = state(5); psi = state(6);
                Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
                Ry = [cos(th), 0, sin(th); 0, 1, 0; -sin(th), 0, cos(th)];
                Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
                R = Rz * Ry * Rx;

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
                buoyancy_force = [0; 0; p.Fb];
                weight_force = [0; 0; -p.mass * 9.81];
                r_cob_world = R * p.cob;
                r_com_world = R * p.com;
                M_buoyancy = cross(R' * r_cob_world, R' * buoyancy_force);
                F = F + R' * (buoyancy_force + weight_force);
                M_t = M_t + TF(4:6) + M_buoyancy;

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
                    pos = state(1:3) + R * p.r_pos(i, :)';
                    vec = R * p.dir_thr(i, :)' * thr_a(i) * arwScale;
                    set(quiv(i), 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), ...
                                 'UData', vec(1), 'VData', vec(2), 'WData', vec(3));
                end
                pos_buoy = state(1:3) + r_cob_world;
                vec_buoy = buoyancy_force * forceScale;
                set(quiv_buoy, 'XData', pos_buoy(1), 'YData', pos_buoy(2), 'ZData', pos_buoy(3), ...
                               'UData', vec_buoy(1), 'VData', vec_buoy(2), 'WData', vec_buoy(3));
                pos_weight = state(1:3) + r_com_world;
                vec_weight = weight_force * forceScale;
                set(quiv_weight, 'XData', pos_weight(1), 'YData', pos_weight(2), 'ZData', pos_weight(3), ...
                                 'UData', vec_weight(1), 'VData', vec_weight(2), 'WData', vec_weight(3));
                addpoints(hTraj, state(1), state(2), state(3));
                addpoints(hPath, state(1), state(2), state(3));

                T4 = eye(4); T4(1:3, 1:3) = R; T4(1:3, 4) = state(1:3);
                set(app.tr, 'Matrix', T4);

                drawnow limitrate;
                elapsed = toc;
                wait = (tnow - elapsed) / app.simControl.speedScale;
                if wait > 0, pause(wait); end
            end
            app.updateResultsTab();
            uialert(app.UIFigure, 'Control scenario simulation completed successfully.', 'Success', 'Icon', 'success');
        catch ME
            uialert(app.UIFigure, sprintf('Control scenario simulation error: %s', ME.message), 'Error', 'Icon', 'error');
        end
    else
        segment_duration = 5;
        num_segments = 12;
        segment_time = segment_duration / p.ts;
        N = round(num_segments * segment_duration / p.ts);
        app.timeVec = (0:N-1) * p.ts;
        app.stateHistory = zeros(12, N);
        app.thrustHistory = zeros(8, N);
        app.errorHistory = zeros(6, N);

        max_speeds = zeros(6, 2);
        segment_names = {'X+', 'X-', 'Y+', 'Y-', 'Z+', 'Z-', 'Roll+', 'Roll-', 'Pitch+', 'Pitch-', 'Yaw+', 'Yaw-'};

        tic;
        try
            k_start = 1;
            for seg = 1:num_segments
                tau = zeros(6, 1);
                if seg <= 6
                    axis = ceil(seg / 2);
                    direction = mod(seg-1, 2) * 2 - 1;
                    tau(axis) = direction * p.maxThrust * sum(abs(A(axis, :)));
                else
                    axis = ceil((seg-6) / 2) + 3;
                    direction = mod(seg-7, 2) * 2 - 1;
                    tau(axis) = direction * p.maxThrust * sum(abs(A(axis, :)));
                end

                for k = k_start:min(k_start + segment_time - 1, N)
                    if app.simControl.isStopped
                        uialert(app.UIFigure, 'Simulation stopped by user.', 'Info', 'Icon', 'info');
                        return;
                    end
                    while app.simControl.isPaused
                        pause(0.1); % Check every 0.1s for resume
                        if app.simControl.isStopped
                            uialert(app.UIFigure, 'Simulation stopped by user.', 'Info', 'Icon', 'info');
                            return;
                        end
                    end

                    phi = state(4); th = state(5); psi = state(6);
                    Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
                    Ry = [cos(th), 0, sin(th); 0, 1, 0; -sin(th), 0, cos(th)];
                    Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
                    R = Rz * Ry * Rx;

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
                    buoyancy_force = [0; 0; p.Fb];
                    weight_force = [0; 0; -p.mass * 9.81];
                    r_cob_world = R * p.cob;
                    r_com_world = R * p.com;
                    M_buoyancy = cross(R' * r_cob_world, R' * buoyancy_force);
                    F = F + R' * (buoyancy_force + weight_force);
                    M_t = M_t + TF(4:6) + M_buoyancy;

                    acc_lin = invM_lin * F;
                    acc_ang = invM_ang * M_t;
                    state(7:9) = state(7:9) + acc_lin * p.ts;
                    state(10:12) = state(10:12) + acc_ang * p.ts;
                    state(1:3) = state(1:3) + state(7:9) * p.ts + 0.5 * acc_lin * p.ts^2;
                    state(4:6) = state(4:6) + state(10:12) * p.ts + 0.5 * acc_ang * p.ts^2;
                    app.stateHistory(:, k) = state;

                    if seg <= 6
                        axis = ceil(seg / 2);
                        col = mod(seg-1, 2) + 1;
                        max_speeds(axis, col) = max(max_speeds(axis, col), abs(state(6 + axis)));
                    else
                        axis = ceil((seg-6) / 2) + 3;
                        col = mod(seg-7, 2) + 1;
                        max_speeds(axis, col) = max(max_speeds(axis, col), abs(state(6 + axis)));
                    end

                    tnow = app.timeVec(k);
                    for i = 1:6
                        addpoints(hPos.(coords{i}), tnow, state(i));
                    end
                    for i = 1:3
                        addpoints(hVel.(vcoords{i}), tnow, state(6+i));
                        addpoints(hAng.(acoords{i}), tnow, state(9+i));
                    end
                    for i = 1:8
                        pos = state(1:3) + R * p.r_pos(i, :)';
                        vec = R * p.dir_thr(i, :)' * thr_a(i) * arwScale;
                        set(quiv(i), 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3), ...
                                     'UData', vec(1), 'VData', vec(2), 'WData', vec(3));
                    end
                    pos_buoy = state(1:3) + r_cob_world;
                    vec_buoy = buoyancy_force * forceScale;
                    set(quiv_buoy, 'XData', pos_buoy(1), 'YData', pos_buoy(2), 'ZData', pos_buoy(3), ...
                                   'UData', vec_buoy(1), 'VData', vec_buoy(2), 'WData', vec_buoy(3));
                    pos_weight = state(1:3) + r_com_world;
                    vec_weight = weight_force * forceScale;
                    set(quiv_weight, 'XData', pos_weight(1), 'YData', pos_weight(2), 'ZData', pos_weight(3), ...
                                     'UData', vec_weight(1), 'VData', vec_weight(2), 'WData', vec_weight(3));
                    addpoints(hTraj, state(1), state(2), state(3));
                    addpoints(hPath, state(1), state(2), state(3));

                    T4 = eye(4); T4(1:3, 1:3) = R; T4(1:3, 4) = state(1:3);
                    set(app.tr, 'Matrix', T4);

                    drawnow limitrate;
                    elapsed = toc;
                    wait = (tnow - elapsed) / app.simControl.speedScale;
                    if wait > 0, pause(wait); end
                end
                k_start = k_start + segment_time;
            end
            app.maxSpeeds = max_speeds;
            app.updateResultsTab();
            uialert(app.UIFigure, 'Test scenario simulation completed successfully.', 'Success', 'Icon', 'success');
        catch ME
            uialert(app.UIFigure, sprintf('Test scenario simulation error: %s', ME.message), 'Error', 'Icon', 'error');
        end
    end
end

        % --- Results Visualization ---
        function updateResultsTab(app)
            cla(app.UIAxesError);
            cla(app.UIAxesControlEffort);

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

            if strcmp(app.scenarioMode, 'Control')
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

                resultsStr = sprintf('Control Scenario Results:\n\n');
                desired_norm = max(abs(app.simParams.desired), [0.1; 0.1; 0.1; 0.01; 0.01; 0.01]);
                thresholds = 0.05 * desired_norm;
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
                    resultsStr = [resultsStr, sprintf('\nWarning: Orientation (Roll, Pitch, Yaw) may be unstable. Consider adjusting thruster configuration.\n')];
                end
            else
                axVel = app.UIAxesError;
                hold(axVel, 'on');
                coords = {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'};
                cols = {'r', 'g', 'b', 'm', 'c', 'k'};
                for i = 1:3
                    plot(axVel, app.timeVec, app.stateHistory(6+i, :), 'Color', cols{i}, 'DisplayName', sprintf('%s Velocity', coords{i}));
                end
                for i = 4:6
                    plot(axVel, app.timeVec, app.stateHistory(6+i, :), 'Color', cols{i}, 'DisplayName', sprintf('%s Ang Velocity', coords{i}));
                end
                title(axVel, 'Velocities Over Time');
                xlabel(axVel, 'Time (s)');
                ylabel(axVel, 'Velocity (m/s or rad/s)');
                legend(axVel, 'show');
                grid(axVel, 'on');
                hold(axVel, 'off');

                resultsStr = sprintf('Test Scenario Results:\n\n');
                resultsStr = [resultsStr, sprintf('Maximum Translational Speeds (m/s):\n')];
                for i = 1:3
                    resultsStr = [resultsStr, sprintf('  %s: Positive = %.4f, Negative = %.4f\n', ...
                        coords{i}, app.maxSpeeds(i, 1), app.maxSpeeds(i, 2))];
                end
                resultsStr = [resultsStr, sprintf('\nMaximum Rotational Speeds (rad/s):\n')];
                for i = 4:6
                    resultsStr = [resultsStr, sprintf('  %s: Positive = %.4f, Negative = %.4f\n', ...
                        coords{i}, app.maxSpeeds(i, 1), app.maxSpeeds(i, 2))];
                end
                energy = sum(sum(app.thrustHistory.^2)) * app.simParams.ts;
                resultsStr = [resultsStr, sprintf('\nTotal Control Effort: %.2f N²s\n', energy)];
            end

            app.ResultsTextArea.Value = {resultsStr};
        end

        % --- Send AI Request ---
        function sendAIRequest(app)
            try
                if isempty(app.timeVec)
                    uialert(app.UIFigure, 'Run a simulation first to generate data.', 'Error', 'Icon', 'error');
                    return;
                end

                coords = {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'};
                simData = sprintf('%s Scenario Results:\n', app.scenarioMode);
                if strcmp(app.scenarioMode, 'Control')
                    desired_norm = max(abs(app.simParams.desired), [0.1; 0.1; 0.1; 0.01; 0.01; 0.01]);
                    thresholds = 0.05 * desired_norm;
                    arrival_times = zeros(6, 1);
                    final_err = app.errorHistory(:, end);
                    max_overshoot = max(abs(app.errorHistory), [], 2);
                    mean_thrust = mean(abs(app.thrustHistory), 2);
                    energy = sum(sum(app.thrustHistory.^2)) * app.simParams.ts;

                    for i = 1:6
                        err = abs(app.errorHistory(i, :));
                        idx = find(err < thresholds(i), 5, 'first');
                        if length(idx) == 5 && all(diff(idx) == 1) && idx(1) > 1
                            arrival_times(i) = app.timeVec(idx(1));
                        else
                            arrival_times(i) = Inf;
                        end
                    end

                    simData = [simData, sprintf('Time to Arrival (within 5%% of desired pose):\n')];
                    for i = 1:6
                        if isinf(arrival_times(i))
                            simData = [simData, sprintf('  %s: Did not reach threshold\n', coords{i})];
                        else
                            simData = [simData, sprintf('  %s: %.2f s\n', coords{i}, arrival_times(i))];
                        end
                    end
                    simData = [simData, sprintf('\nFinal Steady-State Error:\n')];
                    for i = 1:6
                        simData = [simData, sprintf('  %s: %.4f\n', coords{i}, final_err(i))];
                    end
                    simData = [simData, sprintf('\nMaximum Overshoot:\n')];
                    for i = 1:6
                        simData = [simData, sprintf('  %s: %.4f\n', coords{i}, max_overshoot(i))];
                    end
                    simData = [simData, sprintf('\nMean Thrust (N):\n')];
                    for i = 1:8
                        simData = [simData, sprintf('  Thruster %d: %.2f\n', i, mean_thrust(i))];
                    end
                    simData = [simData, sprintf('\nTotal Control Effort: %.2f N²s\n', energy)];
                else
                    simData = [simData, sprintf('Maximum Translational Speeds (m/s):\n')];
                    for i = 1:3
                        simData = [simData, sprintf('  %s: Positive = %.4f, Negative = %.4f\n', ...
                            coords{i}, app.maxSpeeds(i, 1), app.maxSpeeds(i, 2))];
                    end
                    simData = [simData, sprintf('\nMaximum Rotational Speeds (rad/s):\n')];
                    for i = 4:6
                        simData = [simData, sprintf('  %s: Positive = %.4f, Negative = %.4f\n', ...
                            coords{i}, app.maxSpeeds(i, 1), app.maxSpeeds(i, 2))];
                    end
                    energy = sum(sum(app.thrustHistory.^2)) * app.simParams.ts;
                    simData = [simData, sprintf('\nTotal Control Effort: %.2f N²s\n', energy)];
                end

                p = app.simParams;
                paramData = sprintf('System Parameters:\n');
                paramData = [paramData, sprintf('Mass: %.4f kg\n', p.mass)];
                paramData = [paramData, sprintf('Displaced Volume: %.6f m³\n', p.Vdisp)];
                paramData = [paramData, sprintf('Buoyancy Force: %.2f N\n', p.Fb)];
                paramData = [paramData, sprintf('Maximum Thrust per Thruster: %.2f N\n', p.maxThrust)];
                paramData = [paramData, sprintf('Time Step: %.2f s\n', p.ts)];
                paramData = [paramData, sprintf('Simulation Duration: %.1f s\n', p.t_final)];
                paramData = [paramData, sprintf('Center of Mass (m): [%.4f, %.4f, %.4f]\n', p.com)];
                paramData = [paramData, sprintf('Center of Buoyancy (m): [%.4f, %.4f, %.4f]\n', p.cob)];
                paramData = [paramData, sprintf('Inertia Matrix Diagonal (kg·m²): [%.4f, %.4f, %.4f]\n', diag(p.I))];
                paramData = [paramData, sprintf('Added Mass Diagonal: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', diag(p.added))];
                paramData = [paramData, sprintf('Linear Drag Coefficients (CdA): [%.2f, %.2f, %.2f]\n', p.CdA)];
                paramData = [paramData, sprintf('Rotational Drag Coefficients Diagonal: [%.2f, %.2f, %.2f]\n', diag(p.D_rot))];
                paramData = [paramData, sprintf('Thruster Positions (m):\n')];
                for i = 1:8
                    paramData = [paramData, sprintf('  Thruster %d: [%.4f, %.4f, %.4f]\n', i, p.r_pos(i, :))];
                end
                paramData = [paramData, sprintf('Thruster Directions:\n')];
                for i = 1:8
                    paramData = [paramData, sprintf('  Thruster %d: [%.4f, %.4f, %.4f]\n', i, p.dir_thr(i, :))];
                end
                paramData = [paramData, sprintf('Pose PID Gains:\n')];
                for i = 1:6
                    paramData = [paramData, sprintf('  %s: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', ...
                        coords{i}, p.Kp(i,i), p.Ki(i,i), p.Kd(i,i))];
                end
                paramData = [paramData, sprintf('Velocity PID Gains:\n')];
                for i = 1:6
                    paramData = [paramData, sprintf('  %s: Kv_p=%.4f, Kv_i=%.4f, Kv_d=%.4f\n', ...
                        coords{i}, p.Kv_p(i,i), p.Kv_i(i,i), p.Kv_d(i,i))];
                end
                paramData = [paramData, sprintf('Desired Pose: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', p.desired)];

                userInput = strjoin(app.AIChatInputTextArea.Value, ' ');
                prompt = sprintf('You are an expert in underwater vehicle dynamics and control. Below are the simulation results and system parameters for a 6-DOF ROV simulation. Analyze the data and respond to the following user request: "%s"\n\n%s\n\n%s', ...
                    userInput, simData, paramData);

                apiUrl = 'https://api.openai.com/v1/chat/completions';
                headers = {'Content-Type', 'application/json'; 'Authorization', ['Bearer ', app.apiKey]};
                body = struct(...
                    'model', 'gpt-4o', ...
                    'messages', {{struct('role', 'user', 'content', prompt)}},...
                    'max_tokens', 1000,...
                    'temperature', 0.7);

                try
                    response = webwrite(apiUrl, body, ...
                        weboptions('RequestMethod', 'post', 'MediaType', 'application/json', 'HeaderFields', headers, 'Timeout', 30));
                    responseText = response.choices(1).message.content;
                    app.AIResponseTextArea.Value = splitlines(responseText);
                    uialert(app.UIFigure, 'AI response received successfully.', 'Success', 'Icon', 'success');
                catch ME
                    if contains(ME.message, '401') || contains(ME.message, '403')
                        uialert(app.UIFigure, 'Invalid or unauthorized API key. Please check your OpenAI API key.', 'API Error', 'Icon', 'error');
                    else
                        uialert(app.UIFigure, sprintf('Error contacting OpenAI API: %s', ME.message), 'API Error', 'Icon', 'error');
                    end
                    app.AIResponseTextArea.Value = {'Error: Unable to retrieve AI response.'};
                end
            catch ME
                uialert(app.UIFigure, sprintf('Error preparing AI request: %s', ME.message), 'Error', 'Icon', 'error');
                app.AIResponseTextArea.Value = {'Error: Unable to prepare AI request.'};
            end
        end

        function createComponents(app)
    app.UIFigure = uifigure('Name', 'ROV Simulation GUI', ...
        'Position', [100, 100, 1200, 800], ...
        'CloseRequestFcn', @(src, event) delete(app));

    app.TabGroup = uitabgroup(app.UIFigure, 'Position', [0, 0, 1200, 800]);

    app.ParametersTab = uitab(app.TabGroup, 'Title', 'Parameters');

    app.BasicPropsPanel = uipanel(app.ParametersTab, ...
        'Title', 'Basic Properties', ...
        'Position', [20, 550, 300, 200]);
    app.MassLabel = uilabel(app.BasicPropsPanel, ...
        'Text', 'Mass (kg):', 'Position', [10, 140, 100, 22]);
    app.MassEdit = uieditfield(app.BasicPropsPanel, 'numeric', ...
        'Position', [120, 140, 100, 22], 'Value', 44.05438);
    app.VdispLabel = uilabel(app.BasicPropsPanel, ...
        'Text', 'Displaced Volume (m³):', 'Position', [10, 100, 100, 22]);
    app.VdispEdit = uieditfield(app.BasicPropsPanel, 'numeric', ...
        'Position', [120, 100, 100, 22], 'Value', 4.4054e-2);
    app.MaxThrustLabel = uilabel(app.BasicPropsPanel, ...
        'Text', 'Max Thrust (N):', 'Position', [10, 60, 100, 22]);
    app.MaxThrustEdit = uieditfield(app.BasicPropsPanel, 'numeric', ...
        'Position', [120, 60, 100, 22], 'Value', 40);

    app.COMPanel = uipanel(app.ParametersTab, ...
        'Title', 'Center of Mass (m)', ...
        'Position', [20, 350, 300, 150]);
    app.COMTable = uitable(app.COMPanel, ...
        'Position', [10, 10, 280, 120], ...
        'ColumnName', {'X', 'Y', 'Z'}, ...
        'RowName', {}, ...
        'Data', [-0.42049; 0.00072; -0.02683]', ...
        'ColumnEditable', true);

    app.COBPanel = uipanel(app.ParametersTab, ...
        'Title', 'Center of Buoyancy (m)', ...
        'Position', [20, 150, 300, 150]);
    app.COBTable = uitable(app.COBPanel, ...
        'Position', [10, 10, 280, 120], ...
        'ColumnName', {'X', 'Y', 'Z'}, ...
        'RowName', {}, ...
        'Data', [0; 0; 0.05]', ...
        'ColumnEditable', true);

    app.InertiaPanel = uipanel(app.ParametersTab, ...
        'Title', 'Inertia Matrix Diagonal (kg·m²)', ...
        'Position', [340, 550, 300, 150]);
    app.InertiaTable = uitable(app.InertiaPanel, ...
        'Position', [10, 10, 280, 120], ...
        'ColumnName', {'Ixx', 'Iyy', 'Izz'}, ...
        'RowName', {}, ...
        'Data', [1.990993, 15.350344, 15.774167], ...
        'ColumnEditable', true);

    app.AddedMassPanel = uipanel(app.ParametersTab, ...
        'Title', 'Added Mass Diagonal', ...
        'Position', [340, 350, 300, 150]);
    app.AddedMassTable = uitable(app.AddedMassPanel, ...
        'Position', [10, 10, 280, 120], ...
        'ColumnName', {'m_x', 'm_y', 'm_z', 'I_x', 'I_y', 'I_z'}, ...
        'RowName', {}, ...
        'Data', [5, 5, 10, 1, 1, 1], ...
        'ColumnEditable', true);

    app.DragPanel = uipanel(app.ParametersTab, ...
        'Title', 'Drag Coefficients', ...
        'Position', [340, 150, 300, 150]);
    app.CdATable = uitable(app.DragPanel, ...
        'Position', [10, 60, 280, 80], ...
        'ColumnName', {'X', 'Y', 'Z'}, ...
        'RowName', {'CdA'}, ...
        'Data', [550; 550; 550]', ...
        'ColumnEditable', true);
    app.DrotTable = uitable(app.DragPanel, ...
        'Position', [10, 10, 280, 40], ...
        'ColumnName', {'Roll', 'Pitch', 'Yaw'}, ...
        'RowName', {'D_rot'}, ...
        'Data', [5, 5, 5], ...
        'ColumnEditable', true);

    app.ThrusterPanel = uipanel(app.ParametersTab, ...
        'Title', 'Thruster and Control Configuration', ...
        'Position', [660, 20, 500, 730]);
    app.ThrusterConfigTable = uitable(app.ThrusterPanel, ...
        'Position', [10, 350, 480, 360], ...
        'ColumnName', {'X (m)', 'Y (m)', 'Z (m)', 'Dir X', 'Dir Y', 'Dir Z'}, ...
        'RowName', {'T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7', 'T8'}, ...
        'Data', [0.2, 0.1, 0.1, 0, 0, 1; ...
                 0.2, -0.1, 0.1, 0, 0, 1; ...
                 -0.2, -0.1, 0.1, 0, 0, 1; ...
                 -0.2, 0.1, 0.1, 0, 0, 1; ...
                 0.2, 0.1, 0, cosd(-45), sind(-45), 0; ...
                 0.2, -0.1, 0, cosd(45), sind(45), 0; ...
                 -0.2, -0.1, 0, cosd(135), sind(135), 0; ...
                 -0.2, 0.1, 0, cosd(-135), sind(-135), 0], ...
        'ColumnEditable', true);
    app.GainsTable = uitable(app.ThrusterPanel, ...
        'Position', [10, 210, 480, 130], ...
        'ColumnName', {'Kp', 'Ki', 'Kd'}, ...
        'RowName', {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'}, ...
        'Data', [2, 0.01, 1; 2, 0.01, 1; 2, 0.01, 1; 0.5, 0, 0.5; 0.5, 0, 0.5; 0.5, 0, 0.5], ...
        'ColumnEditable', true);
    app.VelocityGainsTable = uitable(app.ThrusterPanel, ...
        'Position', [10, 80, 480, 130], ...
        'ColumnName', {'Kv_p', 'Kv_i', 'Kv_d'}, ...
        'RowName', {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'}, ...
        'Data', [5, 0.05, 2; 5, 0.05, 2; 5, 0.05, 2; 2, 0, 1; 2, 0, 1; 2, 0, 1], ...
        'ColumnEditable', true);
    app.DesiredPoseTable = uitable(app.ThrusterPanel, ...
        'Position', [10, 10, 480, 60], ...
        'ColumnName', {'X (m)', 'Y (m)', 'Z (m)', 'Roll (rad)', 'Pitch (rad)', 'Yaw (rad)'}, ...
        'RowName', {'Desired'}, ...
        'Data', [20, 20, 20, 0, 0, 0], ...
        'ColumnEditable', true);

    app.SimulationTab = uitab(app.TabGroup, 'Title', 'Simulation');
    app.UIAxes2D = uiaxes(app.SimulationTab, ...
        'Position', [20, 20, 1160, 740], ...
        'XGrid', 'on', 'YGrid', 'on');

    app.VisualizationTab = uitab(app.TabGroup, 'Title', 'Visualization');
    app.UIAxesThruster3D = uiaxes(app.VisualizationTab, ...
        'Position', [20, 410, 560, 350], ...
        'XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on');
    app.UIAxesTrajectory3D = uiaxes(app.VisualizationTab, ...
        'Position', [620, 410, 560, 350], ...
        'XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on');
    app.UIAxesPath3D = uiaxes(app.VisualizationTab, ...
        'Position', [20, 20, 1160, 350], ...
        'XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on');

    app.ResultsTab = uitab(app.TabGroup, 'Title', 'Results');
    app.UIAxesError = uiaxes(app.ResultsTab, ...
        'Position', [20, 410, 560, 350], ...
        'XGrid', 'on', 'YGrid', 'on');
    app.UIAxesControlEffort = uiaxes(app.ResultsTab, ...
        'Position', [620, 410, 560, 350], ...
        'XGrid', 'on', 'YGrid', 'on');
    app.ResultsTextArea = uitextarea(app.ResultsTab, ...
        'Position', [20, 20, 560, 350]);
    app.AIChatInputLabel = uilabel(app.ResultsTab, ...
        'Text', 'AI Chat Input:', ...
        'Position', [620, 350, 100, 22]);
    app.AIChatInputTextArea = uitextarea(app.ResultsTab, ...
        'Position', [620, 200, 560, 140], ...
        'Value', {'Enter your request (e.g., "Analyze high yaw error" or "Suggest PID tuning improvements").'});
    app.AISendButton = uibutton(app.ResultsTab, ...
        'Text', 'Send to AI', ...
        'Position', [620, 150, 100, 22], ...
        'ButtonPushedFcn', @(src, event) sendAIRequest(app));
    app.AIResponseTextArea = uitextarea(app.ResultsTab, ...
        'Position', [620, 20, 560, 120]);

    app.DragFittingTab = uitab(app.TabGroup, 'Title', 'Drag Fitting');
    app.NumPointsLabel = uilabel(app.DragFittingTab, ...
        'Text', 'Number of Points:', ...
        'Position', [20, 740, 100, 22]);
    app.NumPointsEdit = uieditfield(app.DragFittingTab, 'numeric', ...
        'Position', [120, 740, 100, 22], ...
        'Value', 11, ...
        'ValueChangedFcn', @(src, event) updateDragTables(app));
    app.LinearDragTable = uitable(app.DragFittingTab, ...
        'Position', [20, 350, 560, 350], ...
        'ColumnName', {'Velocity (m/s)', 'X Drag (N)', 'Y Drag (N)', 'Z Drag (N)'}, ...
        'RowName', {}, ...
        'ColumnEditable', [true, true, true, true]);
    app.RotationalDragTable = uitable(app.DragFittingTab, ...
        'Position', [620, 350, 560, 350], ...
        'ColumnName', {'Ang Velocity (rad/s)', 'Roll Drag (N·m)', 'Pitch Drag (N·m)', 'Yaw Drag (N·m)'}, ...
        'RowName', {}, ...
        'ColumnEditable', [true, true, true, true]);
    app.UIAxesLinearDrag = uiaxes(app.DragFittingTab, ...
        'Position', [20, 20, 560, 300], ...
        'XGrid', 'on', 'YGrid', 'on');
    app.UIAxesRotationalDrag = uiaxes(app.DragFittingTab, ...
        'Position', [620, 20, 560, 300], ...
        'XGrid', 'on', 'YGrid', 'on');

    app.PIDTuningTab = uitab(app.TabGroup, 'Title', 'PID Tuning');
    app.TuningGoalDropDown = uidropdown(app.PIDTuningTab, ...
        'Position', [20, 740, 200, 22], ...
        'Items', {'Fast Response', 'Minimal Overshoot', 'Balanced Performance'}, ...
        'Value', 'Balanced Performance');
    app.TunePIDButton = uibutton(app.PIDTuningTab, ...
        'Text', 'Tune PID', ...
        'Position', [240, 740, 100, 22], ...
        'ButtonPushedFcn', @(src, event) TunePIDAutomatically(app));
    app.TuningProgressLabel = uilabel(app.PIDTuningTab, ...
        'Text', 'Tuning Progress:', ...
        'Position', [20, 700, 300, 22]);
    app.TuningResultsTextArea = uitextarea(app.PIDTuningTab, ...
        'Position', [20, 20, 1160, 660]);

    app.LoadDragButton = uibutton(app.DragFittingTab, ...
        'Text', 'Load Drag from CSV', ...
        'Position', [240, 740, 150, 22], ...
        'ButtonPushedFcn', @(src, event) LoadDragFromCSV(app));
    app.FitDragButton = uibutton(app.DragFittingTab, ...
        'Text', 'Fit Drag Polynomials', ...
        'Position', [410, 740, 150, 22], ...
        'ButtonPushedFcn', @(src, event) FitDragPolynomials(app));
    app.RunSimButton = uibutton(app.SimulationTab, ...
        'Text', 'Run Simulation', ...
        'Position', [20, 740, 100, 22], ...
        'ButtonPushedFcn', @(src, event) RunSim(app, [], []));
    app.PauseButton = uibutton(app.SimulationTab, ...
        'Text', 'Pause', ...
        'Position', [140, 740, 100, 22], ...
        'ButtonPushedFcn', @(src, event) togglePause(app));
    app.AccelerateButton = uibutton(app.SimulationTab, ...
        'Text', 'Accelerate', ...
        'Position', [260, 740, 100, 22], ...
        'ButtonPushedFcn', @(src, event) accelerateSim(app));
    app.DecelerateButton = uibutton(app.SimulationTab, ...
        'Text', 'Decelerate', ...
        'Position', [380, 740, 100, 22], ...
        'ButtonPushedFcn', @(src, event) decelerateSim(app));
    app.StopButton = uibutton(app.SimulationTab, ...
        'Text', 'Stop', ...
        'Position', [500, 740, 100, 22], ...
        'ButtonPushedFcn', @(src, event) stopSim(app));
    app.ForceLoadMatButton = uibutton(app.ParametersTab, ...
        'Text', 'Force Load .mat', ...
        'Position', [20, 20, 100, 22], ...
        'ButtonPushedFcn', @(src, event) forceLoadParameters(app));
    app.ResetDefaultsButton = uibutton(app.ParametersTab, ...
        'Text', 'Reset Defaults', ...
        'Position', [140, 20, 100, 22], ...
        'ButtonPushedFcn', @(src, event) resetToDefaults(app, [], []));
    app.ScenarioSwitch = uiswitch(app.SimulationTab, ...
        'Position', [620, 740, 60, 20], ...
        'Items', {'Control', 'Test'}, ...
        'Value', 'Control', ...
        'ValueChangedFcn', @(src, event) updateScenarioMode(app, src));
end
        % --- Scenario Mode Update ---
        function updateScenarioMode(app, src)
            app.scenarioMode = src.Value;
            if strcmp(app.scenarioMode, 'Control')
                app.DesiredPoseTable.Enable = 'on';
                app.GainsTable.Enable = 'on';
                app.VelocityGainsTable.Enable = 'on';
                app.TunePIDButton.Enable = 'on';
            else
                app.DesiredPoseTable.Enable = 'off';
                app.GainsTable.Enable = 'off';
                app.VelocityGainsTable.Enable = 'off';
                app.TunePIDButton.Enable = 'off';
            end
            app.saveParametersToMat();
        end
    end
end