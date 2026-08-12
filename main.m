%% main.m
% 2D SLAM Simulation - Project Entry Point
%
% FULL PIPELINE:
%   Phase 3A: Navigation (Controller → Midpoint Motion → Collision → Recovery)
%   Phase 3B: Virtual LiDAR (ray casting against ground truth map)
%   Phase 4:  SLAM (lidarSLAM → pose graph → occupancy map → results)
%
% Modules:
%   createEnvironment.m       - Build the ground-truth occupancy map
%   createRobot.m             - Initialize the robot struct
%   generateWaypoints.m       - Wavefront-based coverage waypoints
%   simulateRobotStep.m       - Motion pipeline (controller/motion/collision)
%   moveRobot.m               - Backward-compatible wrapper
%   simulateLiDAR.m           - Virtual 2D LiDAR sensor (ray casting)
%   initializeSLAM.m          - Configure the lidarSLAM object
%   updateSLAM.m              - Add scans, detect loop closures
%   buildOccupancyMap.m       - Build map from SLAM scans + poses
%   updateRobotGraphics.m     - Robot, waypoints, trajectory visualization
%   updateLiDARGraphics.m     - LiDAR ray and hit-point overlay
%   createStatusPanel.m       - Dashboard UI creation
%   updateStatusPanel.m       - Dashboard UI update
%   loggerInit.m              - Preallocate simulation logger
%   loggerUpdate.m            - Record one simulation step
%   loggerExport.m            - Export logged data to MAT file
%   helperFunctions.m         - Shared static utilities
%   plotSLAMResults.m         - Final multi-panel results figure
%
% Required toolboxes: Robotics System Toolbox, Navigation Toolbox
% No Simulink, no ROS, no Gazebo, no external datasets/images required.

clear;
close all;
clc;

%% ====================================================================
%  SIMULATION CONFIGURATION
%  ====================================================================

% --- Environment ---
config.mapWidth          = 20;      % meters
config.mapHeight         = 15;      % meters
config.resolution        = 20;      % cells per meter

% --- Robot ---
config.robotStartPose    = [1.2, 1.2, 0];   % [x y theta], meters/rad
config.waypointSpacing   = 1.5;     % meters, exploration grid spacing (wider = fewer, more reachable points)
config.waypointClearance = 0.50;    % meters, min distance from walls (increased for safer waypoints)

% --- Simulation ---
config.dt                = 0.05;    % seconds per step (50 ms)
config.waypointTolerance = 0.15;    % meters
config.maxSimTime        = 300;     % seconds, safety cap
config.collisionThreshold = 80;     % steps before waypoint skip (increased for harder obstacles)

% --- LiDAR ---
config.lidar.maxRange          = 8.0;     % meters
config.lidar.minRange          = 0.12;    % meters
config.lidar.fov               = 2*pi;    % radians (360-degree scan)
config.lidar.angularResolution = deg2rad(1); % 1-degree steps → 360 rays
config.lidar.rangeNoise        = 0.01;    % std dev of Gaussian noise (m)

% --- SLAM ---
config.slam.mapResolution         = 20;    % cells per meter
config.slam.maxRange               = config.lidar.maxRange;
config.slam.loopClosureThreshold   = 200;
config.slam.loopClosureSearchRadius = 3.0; % meters
% MovementThreshold: [translation(m) rotation(rad)].
%   Robot at dt=0.05s, ~0.3 m/s covers ~0.015 m/step.
%   0.5 m translation ≈ every ~33 steps ≈ one scan per 1.7 s.
config.slam.movementThreshold      = [0.20, deg2rad(8)];
% OptimizationInterval: number of loop closures before optimization.
config.slam.optimizationInterval   = 5;

% --- Visualization ---
config.showLiDARRays       = true;   % toggle live LiDAR ray overlay
config.slamMapUpdateRate   = 20;     % rebuild SLAM map every N accepted scans
config.fpsSmoothing        = 0.9;    % exponential smoothing factor for FPS

% --- Recovery (analyzeRecovery parameters) ---
% Turn-angle bounds (radians)
config.recovery.minTurnAngle          = deg2rad(20);  % smallest correction
config.recovery.mediumTurnAngle       = deg2rad(45);  % wall-mode upper bound
config.recovery.maxTurnAngle          = deg2rad(90);  % maximum escape rotation
% Smooth angle mapping: score diff that drives the smooth-step to 1.0
%   (replaces the old smallDiffThreshold / largeDiffThreshold hard steps)
config.recovery.maxDiffForMaxAngle    = 4.0;  % combined-score diff (m) → maxTurnAngle
% Dead-end detection
config.recovery.deadEndScoreThreshold = 0.6;  % both sides below this → reverse (m)
config.recovery.deadEndPersistenceCount = 3;  % consecutive readings before committing
% LiDAR sector geometry (radians, robot frame)
config.recovery.lidarMaxRange         = config.lidar.maxRange;
config.recovery.frontSectorHalfAngle  = deg2rad(30);  % ±30° front sector
config.recovery.sideInnerAngle        = deg2rad(30);  % inner edge of side sectors
config.recovery.sideOuterAngle        = deg2rad(90);  % outer edge of side sectors
% Map sampling geometry (meters)
config.recovery.wallDetectionRadius   = 1.5;   % forward/lateral probe extent
config.recovery.minProbeDistance      = 0.15;  % nearest forward sample point
% Environment classification thresholds
config.recovery.wallLengthThreshold      = 0.8;   % min PCA major span for wall (m)
config.recovery.wallAspectThreshold      = 2.5;   % PCA major/minor span ratio
config.recovery.wallContinuityThreshold  = 0.55;  % min gap-continuity fraction
config.recovery.wallSideThreshold        = 0.3;   % |y| boundary for sub-sectors (m)
config.recovery.objectSizeThreshold      = 0.7;   % max PCA span for object (m)
config.recovery.deadEndSideRatio         = 0.40;  % (leftOcc+rightOcc)/total → DEADEND
% Direction confidence (Issue 4)
config.recovery.directionConfidenceMin   = 0.12;  % below this → scale down angle / alternate
% Anti-oscillation / recovery memory (Issue 5)
config.recovery.oscillationThreshold     = 2;     % failures before flipping direction
config.recovery.angleEscalationFactor    = 0.35;  % angle growth per failed attempt
% Waypoint-aware direction (optional feature)
config.recovery.waypointWeight           = 0.25;  % goal-heading influence weight
config.recovery.clearanceWeight          = 1.0;   % LiDAR clearance influence weight
% Continuous re-evaluation
config.recovery.maxRotateSteps           = 60;    % steps before left→right switch

% --- Local Planner (localPlanner parameters) ---
% Path feasibility check
config.localPlanner.pathCheckResolution = 0.1;    % sample spacing along path (m)
config.localPlanner.pathCheckInflation  = 0.1;    % extra inflation for check (m)
% Candidate sampling
config.localPlanner.candidateRadius     = 2.0;    % max radius for candidates (m)
config.localPlanner.numCandidates       = 32;     % number of candidate points
config.localPlanner.forwardBias         = 0.7;    % fraction in front 180 deg
% Scoring weights: [goalProgress, clearance, smoothness]
config.localPlanner.scoreWeights        = [1.0, 0.5, 0.3];
% Clearance requirements
config.localPlanner.minClearance        = 0.3;    % minimum clearance from obstacles (m)
config.localPlanner.hysteresisRadius    = 0.5;    % reuse prev temp wp if within (m)
% Environment-specific behavior
config.localPlanner.wallFollowOffset    = 0.5;    % lateral offset for wall-following (m)
config.localPlanner.objectClearance     = 0.4;    % extra clearance for objects (m)
config.localPlanner.maxHeadingChange    = pi/2;   % max heading change per step (rad)
config.localPlanner.replanInterval      = 5;      % steps between full replans
% Temporary waypoint tolerance
config.localPlanner.waypointTolerance   = 0.25;   % temp waypoint reach tolerance (m)
% LiDAR config reference (for sector scoring)
config.localPlanner.lidarMaxRange       = config.lidar.maxRange;
config.localPlanner.frontSectorHalfAngle = deg2rad(30);
config.localPlanner.sideInnerAngle      = deg2rad(30);
config.localPlanner.sideOuterAngle      = deg2rad(90);

% --- Perception (RANSAC + Map Filtering + Opening Detection) ---
% RANSAC wall extraction
config.perception.ransac.distanceThreshold  = 0.05;   % m (5x LiDAR noise sigma)
config.perception.ransac.maxIterations      = 200;
config.perception.ransac.minInliers         = 8;
config.perception.ransac.minSegmentLength   = 0.5;    % m
config.perception.ransac.maxSegments        = 15;
config.perception.ransac.windowSize         = 10;     % aggregate last N scans
config.perception.ransac.maxRange           = config.lidar.maxRange;
config.perception.ransac.minRange           = config.lidar.minRange;
% Conservative map filtering
config.perception.filter.behindWallTolerance     = 0.10;  % m
config.perception.filter.wallProximityThreshold  = 0.15;  % m
config.perception.filter.minOccupiedObservations = 2;
config.perception.filter.minNeighbourOccupied    = 2;
config.perception.filter.suspicionThreshold      = 3;
config.perception.filter.cornerPreservationRadius = 0.3;  % m
config.perception.filter.preservationInlierCount = 6;
% Opening detection
config.perception.opening.collinearAngleThreshold  = deg2rad(15);
config.perception.opening.collinearOffsetThreshold = 0.3;    % m
config.perception.opening.minOpeningWidth          = 0.55;   % m (2.5x safetyRadius)
config.perception.opening.maxOpeningWidth          = 3.0;    % m
config.perception.opening.minWallSupport           = 8;      % inliers per side
config.perception.opening.minWallLength            = 0.8;    % m
% Robot clearance reference
config.perception.robotSafetyRadius = 0.22;  % from createRobot defaults

%% ====================================================================
%  STEP 1: GENERATE GROUND-TRUTH ENVIRONMENT
%  ====================================================================
fprintf('Generating environment (%.1f m x %.1f m, %d cells/m)...\n', ...
    config.mapWidth, config.mapHeight, config.resolution);

[groundTruthMap, envInfo] = createEnvironment( ...
    config.mapWidth, config.mapHeight, config.resolution);

if envInfo.isFullyConnected
    fprintf('Environment generated: free space is fully connected.\n');
else
    fprintf(['Environment generated: WARNING — only %.1f%% of free space ', ...
        'is reachable from the seed cell.\n'], 100 * envInfo.freeSpaceCoverage);
end

%% ====================================================================
%  STEP 2: CREATE THE ROBOT
%  ====================================================================
robot = createRobot(groundTruthMap, config.robotStartPose, 'MaxLinearVel', 1.5);
fprintf('Robot created at pose [%.2f, %.2f, %.2f rad].\n', robot.pose);

%% ====================================================================
%  STEP 3: GENERATE EXPLORATION WAYPOINTS (WAVEFRONT)
%  ====================================================================
waypoints = generateWaypoints(groundTruthMap, robot.pose(1:2), ...
    'Spacing', config.waypointSpacing, ...
    'Clearance', config.waypointClearance, ...
    'Visualize', false);

if isempty(waypoints)
    error('main:NoWaypoints', ...
        'No exploration waypoints could be generated; check Spacing/Clearance settings.');
end
numWaypoints = size(waypoints, 1);
fprintf('Generated %d exploration waypoints (wavefront ordering).\n', numWaypoints);

%% ====================================================================
%  STEP 4: INITIALIZE SLAM
%  ====================================================================
slamState = initializeSLAM(config.slam);
% Attach perception config to slamState so buildOccupancyMap can access it.
slamState.perceptionConfig = config.perception;
% Initialize slamStats with the same struct shape as updateSLAM output,
% so the logger/dashboard always receive valid data — even on sim steps
% where the movement gate skips LiDAR/SLAM.
slamStats.isAccepted           = false;
slamStats.loopClosureDetected  = false;
slamStats.estimatedPose        = [0, 0, 0];
slamStats.localizationError    = NaN;
slamStats.totalAccepted        = 0;
slamStats.totalRejected        = 0;
slamStats.totalLoopClosures    = 0;

%% ====================================================================
%  STEP 5: INITIALIZE LOGGER
%  ====================================================================
maxExpectedSteps = ceil(config.maxSimTime / config.dt) + 1;
logger = loggerInit(maxExpectedSteps);

%% ====================================================================
%  STEP 6: SET UP THE LIVE DISPLAY
%  ====================================================================
figHandle = figure('Name', '2D SLAM Simulation', 'NumberTitle', 'off', ...
    'Position', [50, 50, 1400, 800]);

% Map axes — leave room on the right for the status panel.
axesHandle = axes(figHandle, 'Units', 'normalized', ...
    'Position', [0.03 0.05 0.53 0.90]);
show(groundTruthMap, 'Parent', axesHandle);

% The Navigation Toolbox show() calls newplot() internally which can
% replace the axes object, invalidating axesHandle.  Re-acquire the
% live axes from the figure so the simulation loop always holds a
% valid handle.
if ~isgraphics(axesHandle, 'axes')
    % Find the main map axes (exclude any legend pseudo-axes).
    allAx = findobj(figHandle, 'Type', 'axes');
    axesHandle = allAx(1);   % show() leaves its axes as the most recent
end

title(axesHandle, 'Robot Navigation (Ground Truth)');
xlabel(axesHandle, 'X (meters)');
ylabel(axesHandle, 'Y (meters)');
axis(axesHandle, 'equal');

% SLAM map axes — secondary panel.
slamAxes = axes(figHandle, 'Units', 'normalized', ...
    'Position', [0.58 0.05 0.18 0.40]);
title(slamAxes, 'SLAM Map');
xlabel(slamAxes, 'X (m)');
ylabel(slamAxes, 'Y (m)');
axis(slamAxes, 'equal');
grid(slamAxes, 'on');

% Status panel.
panelHandles = createStatusPanel(figHandle);

graphicsHandles = [];  % triggers one-time creation on first call
lidarGraphics   = [];  % triggers one-time creation on first call

% Preallocate trajectory buffers.
trajectoryBuffer    = nan(maxExpectedSteps, 2);
estTrajectoryBuffer = nan(maxExpectedSteps, 2);
trajectoryCount     = 1;
estTrajectoryCount  = 0;
trajectoryBuffer(1, :) = robot.pose(1:2);

%% ====================================================================
%  STEP 7: SIMULATION LOOP
%  ====================================================================
%  Per-iteration pipeline:
%    1. Controller -> Motion -> Collision -> Recovery   [simulateRobotStep]
%    2. Movement gate check
%    3. LiDAR ray casting           (only if gate passes) [simulateLiDAR]
%    4. SLAM scan matching          (only if gate passes) [updateSLAM]
%    5. Logger                                           [loggerUpdate]
%    6. Visualization                                    [update*Graphics]
%    7. Dashboard                                        [updateStatusPanel]
%  ====================================================================
controllerState      = struct('currentWaypointIdx', 1);
isPathComplete       = false;
simulationTime       = 0;
totalDistanceTraveled = 0;
previousPosition     = robot.pose(1:2);
lastAcceptedForMap   = 0;   % track when to rebuild the SLAM map
smoothedFPS          = 0;
slamMap              = [];  % built lazily
perceptionResults    = initPerceptionResults(); % empty valid struct (updated on map rebuild)

% Movement gating: use the same translation threshold as lidarSLAM's
% MovementThreshold so robot-side gating and SLAM-side gating agree.
movementGateThreshold = slamState.movementThreshold;  % meters
lastScanPosition      = robot.pose(1:2);  % position at last LiDAR+SLAM
lastScan              = [];               % latest lidarScan for display

fprintf('Starting simulation loop...\n');
tic;

while ~isPathComplete && simulationTime < config.maxSimTime

    stepTic = tic;

    % --- Stage 1: Navigation pipeline ---
    [robot, controllerState, isPathComplete] = simulateRobotStep( ...
        robot, waypoints, controllerState, config.dt, ...
        config.waypointTolerance, groundTruthMap, config.collisionThreshold, ...
        lastScan, config.recovery, config.localPlanner);

    simulationTime = simulationTime + config.dt;

    % --- Distance tracking ---
    currentPosition = robot.pose(1:2);
    stepDistance = norm(currentPosition - previousPosition);
    totalDistanceTraveled = totalDistanceTraveled + stepDistance;
    previousPosition = currentPosition;

    % --- Stage 2: Movement gate ---
    %   Only acquire a LiDAR scan and run SLAM when the robot has
    %   moved at least movementGateThreshold since the last scan.
    %   This eliminates scan oversampling at the source, improving
    %   FPS and producing meaningful accept/reject/loop-closure stats.
    distSinceLastScan = norm(currentPosition - lastScanPosition);
    runSLAM = distSinceLastScan >= movementGateThreshold;

    if runSLAM
        % --- Stage 3: LiDAR ---
        scan = simulateLiDAR(groundTruthMap, robot.pose, config.lidar);
        lastScan = scan;
        lastScanPosition = currentPosition;

        % --- Stage 4: SLAM ---
        [slamState, slamStats] = updateSLAM(slamState, scan, robot.pose);
    end
    %   When the gate does NOT fire, slamStats retains its value from
    %   the previous iteration (or the default initialised above the
    %   loop).  The logger and dashboard still receive valid data.

    % --- Stage 5: Logger ---
    logger = loggerUpdate(logger, simulationTime, robot, slamStats, lastScan);

    % --- Trajectory buffers ---
    trajectoryCount = trajectoryCount + 1;
    if trajectoryCount <= size(trajectoryBuffer, 1)
        trajectoryBuffer(trajectoryCount, :) = currentPosition;
    end

    if runSLAM && slamStats.isAccepted && ~any(isnan(slamStats.estimatedPose))
        estTrajectoryCount = estTrajectoryCount + 1;
        if estTrajectoryCount <= size(estTrajectoryBuffer, 1)
            estTrajectoryBuffer(estTrajectoryCount, :) = slamStats.estimatedPose(1:2);
        end
    end

    % --- Stage 6: Visualization ---
    graphicsHandles = updateRobotGraphics(axesHandle, graphicsHandles, ...
        robot, waypoints, controllerState.currentWaypointIdx, ...
        trajectoryBuffer, trajectoryCount, ...
        estTrajectoryBuffer, estTrajectoryCount);

    if ~isempty(lastScan)
        lidarGraphics = updateLiDARGraphics(axesHandle, lidarGraphics, ...
            robot.pose, lastScan, config.lidar, config.showLiDARRays);
    end

    % Rebuild SLAM map periodically (every slamMapUpdateRate accepted
    % scans) or immediately after a loop closure.
    needMapRebuild = false;
    if slamState.acceptedCount > lastAcceptedForMap
        if mod(slamState.acceptedCount, config.slamMapUpdateRate) == 0
            needMapRebuild = true;
        end
        if runSLAM && slamStats.loopClosureDetected
            needMapRebuild = true;
        end
    end
    if needMapRebuild
        [slamMap, perceptionResults] = buildOccupancyMap(slamState, config.mapWidth, ...
            config.mapHeight, config.slam.mapResolution, config.slam.maxRange);
        lastAcceptedForMap = slamState.acceptedCount;

        % Refresh SLAM map panel.
        cla(slamAxes);
        show(slamMap, 'Parent', slamAxes);
        title(slamAxes, sprintf('SLAM Map (%d scans)', slamState.acceptedCount));

        % Log perception metrics.
        if ~isempty(perceptionResults)
            logger = loggerUpdatePerception(logger, ...
                size(perceptionResults.wallSegments, 1), ...
                numel(perceptionResults.openings), ...
                perceptionResults.filteredCellCount);
        end
    end

    % --- Stage 7: Dashboard ---
    stepDuration = toc(stepTic);
    currentFPS = 1 / max(stepDuration, 1e-6);
    smoothedFPS = config.fpsSmoothing * smoothedFPS + ...
        (1 - config.fpsSmoothing) * currentFPS;

    updateStatusPanel(panelHandles, robot, controllerState, numWaypoints, ...
        simulationTime, totalDistanceTraveled, slamStats, smoothedFPS);

    % Update map size in the panel (needs the slamMap object).
    if ~isempty(slamMap)
        [mapRows, mapCols] = size(getOccupancy(slamMap));
        set(panelHandles.mapSize, 'String', sprintf('%dx%d', mapRows, mapCols));
    end

    drawnow limitrate;
end

elapsedWall = toc;

%% ====================================================================
%  STEP 8: FINAL SLAM MAP BUILD
%  ====================================================================
fprintf('Building final SLAM occupancy map...\n');
[slamMap, perceptionResults] = buildOccupancyMap(slamState, config.mapWidth, ...
    config.mapHeight, config.slam.mapResolution, config.slam.maxRange);

% Refresh the SLAM axes with the final map.
cla(slamAxes);
show(slamMap, 'Parent', slamAxes);
title(slamAxes, sprintf('SLAM Map (Final, %d scans)', slamState.acceptedCount));

%% ====================================================================
%  STEP 9: EXPORT LOGGER DATA
%  ====================================================================
loggerExport(logger, 'slam_simulation_log.mat');

%% ====================================================================
%  STEP 10: RESULTS DASHBOARD
%  ====================================================================
plotSLAMResults(logger, slamState, groundTruthMap, slamMap, config, perceptionResults);

%% ====================================================================
%  CONSOLE SUMMARY
%  ====================================================================
if isPathComplete
    fprintf('\nNavigation complete: all waypoints reached.\n');
else
    fprintf('\nNavigation stopped: maxSimTime (%.1f s) before finishing.\n', ...
        config.maxSimTime);
end

fprintf('Wall-clock time:        %.1f s\n', elapsedWall);
fprintf('Simulated time:         %.1f s\n', simulationTime);
fprintf('Total distance:         %.2f m\n', totalDistanceTraveled);
fprintf('Final GT pose:          %s\n', helperFunctions.formatPose(robot.pose));
fprintf('Final est pose:         %s\n', helperFunctions.formatPose(slamState.estimatedPose));
fprintf('SLAM scans accepted:    %d / %d\n', slamState.acceptedCount, slamState.scanCount);
fprintf('Loop closures:          %d\n', slamState.loopClosureCount);

% Compute final localization metrics.
validErr = logger.localizationError(1:logger.count);
validErr = validErr(~isnan(validErr));
if ~isempty(validErr)
    fprintf('Localization RMSE:      %.4f m\n', sqrt(mean(validErr.^2)));
    fprintf('Localization max error: %.4f m\n', max(validErr));
    fprintf('Localization mean:      %.4f m\n', mean(validErr));
end

% Perception layer summary.
if ~isempty(perceptionResults)
    fprintf('\n--- Perception Layer ---\n');
    fprintf('Wall segments detected: %d\n', size(perceptionResults.wallSegments, 1));
    fprintf('Openings detected:      %d\n', numel(perceptionResults.openings));
    fprintf('Filtered cells:         %d\n', perceptionResults.filteredCellCount);
    if numel(perceptionResults.openings) > 0
        for oi = 1:numel(perceptionResults.openings)
            op = perceptionResults.openings(oi);
            fprintf('  Opening %d: center=[%.2f, %.2f], width=%.2f m, confidence=%.2f\n', ...
                oi, op.center(1), op.center(2), op.width, op.confidence);
        end
    end
end


% -----------------------------------------------------------------------
%  Local helper: return empty-but-valid perceptionResults struct.
% -----------------------------------------------------------------------
function pr = initPerceptionResults()
    pr = struct( ...
        'wallSegments', zeros(0, 4), ...
        'segmentInfo', struct('numInliers', {}, 'orientation', {}, ...
            'length', {}, 'lineParams', {}, 'inlierPoints', {}), ...
        'openings', struct('center', {}, 'width', {}, 'normal', {}, ...
            'direction', {}, 'endpoint1', {}, 'endpoint2', {}, ...
            'segmentIdx', {}, 'confidence', {}, 'wallSupport', {}), ...
        'filteredCellCount', 0);
end
