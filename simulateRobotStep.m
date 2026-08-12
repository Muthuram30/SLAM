function [robot, controllerState, isPathComplete] = simulateRobotStep(robot, waypoints, controllerState, dt, waypointTolerance, map, collisionThreshold, lastScan, recoveryConfig, localPlannerConfig)
%SIMULATEROBOTSTEP Advance the robot one time step through a decomposed
%pipeline: Controller → Velocity → Motion Model → Collision Check → State.
%
%   [robot, controllerState, isPathComplete] = SIMULATEROBOTSTEP(robot, ...
%       waypoints, controllerState, dt, waypointTolerance, map, ...
%       collisionThreshold)
%
%   [robot, controllerState, isPathComplete] = SIMULATEROBOTSTEP(..., ...
%       lastScan, recoveryConfig, localPlannerConfig)
%
%   Pipeline stages:
%       1. Controller         - compute velocity commands toward current waypoint
%       2. Motion Model       - midpoint integration of the unicycle model
%       3. Collision          - check new pose (dynamic ring sampling)
%       4. Local Planner      - check path feasibility, generate temp waypoints
%       5. Recovery           - map-aware, LiDAR-assisted (analyzeRecovery) or
%                               fixed-angle fallback when args 8-9 are absent
%       6. State Update       - update robot struct with pose + diagnostics
%
%   Inputs:
%       robot               - robot struct from createRobot.m
%       waypoints           - Nx2 matrix of [x y] waypoints to follow
%       controllerState     - struct with field .currentWaypointIdx
%       dt                  - time step, seconds (scalar, > 0)
%       waypointTolerance   - distance within which a waypoint is reached
%       map                 - binaryOccupancyMap for collision checking
%       collisionThreshold  - (optional, default 50) recovery cycles before
%                             the current waypoint is skipped
%       lastScan            - (optional) lidarScan from simulateLiDAR;
%                             pass [] to use map-only recovery analysis
%       recoveryConfig      - (optional) config.recovery struct from main.m;
%                             when absent, falls back to fixed 25° turns
%       localPlannerConfig  - (optional) config.localPlanner struct from main.m;
%                             when absent, local planner is disabled
%
%   Outputs:
%       robot           - updated robot struct with new pose and diagnostics
%       controllerState - updated struct, .currentWaypointIdx advanced
%       isPathComplete  - true once every waypoint has been reached

    % ---------------------------------------------------------------
    % Defaults and input validation
    % ---------------------------------------------------------------
    if nargin < 7 || isempty(collisionThreshold)
        collisionThreshold = 50;
    end
    if nargin < 8;  lastScan           = [];  end
    if nargin < 9;  recoveryConfig     = [];  end
    if nargin < 10; localPlannerConfig = [];  end

    % hasAnalyzer: true when both the scan and recovery config are present.
    % When false every decision point falls back to the original fixed-angle
    % behaviour, preserving full backward compatibility.
    hasAnalyzer = ~isempty(lastScan) && ~isempty(recoveryConfig);

    % hasLocalPlanner: true when local planner config is provided
    hasLocalPlanner = ~isempty(localPlannerConfig);

    validateattributes(waypoints, {'double'}, {'2d', 'ncols', 2});
    validateattributes(dt, {'double'}, {'scalar', 'positive', 'finite'});
    validateattributes(waypointTolerance, {'double'}, {'scalar', 'positive', 'finite'});

    numWaypoints = size(waypoints, 1);

    % Nothing to do if the path is empty or already finished.
    if numWaypoints == 0 || controllerState.currentWaypointIdx > numWaypoints
        robot.linearVel      = 0;
        robot.angularVel     = 0;
        robot.targetWaypoint = [NaN, NaN];
        robot.distanceToGoal = NaN;
        robot.headingError   = NaN;
        robot.navigationState= 'COMPLETED';
        isPathComplete       = true;
        return;
    end

    isPathComplete = false;

    % ---------------------------------------------------------------
    % Exact Flowchart Implementation: LiDAR Scan Coverage & Waypoint Skipping
    % Flowchart:
    %   A: LiDAR Scans Environment
    %   B: Calculate Distance from LiDAR to Remaining Waypoints
    %   C: Is Waypoint within LiDAR Range & FOV?
    %   D: Yes -> Increment waypointScanCount(k)
    %   E: No -> Keep Current Scan Count
    %   F: Is waypointScanCount(k) >= 2?
    %   G & H: Yes -> Mark Waypoint as EXPLORED (Green Circle) & Advance Gold Star Target
    %   I: No -> Continue Driving Toward Current Gold Star Target
    % ---------------------------------------------------------------
    if ~isfield(controllerState, 'waypointScanCount') || numel(controllerState.waypointScanCount) ~= numWaypoints
        controllerState.waypointScanCount = zeros(numWaypoints, 1);
    end

    % Adaptive local scan radius: prevent long-range doorway ray leakage
    % from prematurely skipping waypoints in unvisited rooms/wings.
    lidarMaxRange = 3.5; % meters (local adaptive scan radius for waypoint coverage)
    lidarFov = 2*pi;     % default 360 degrees
    if ~isempty(localPlannerConfig) && isfield(localPlannerConfig, 'lidarMaxRange')
        lidarMaxRange = min(localPlannerConfig.lidarMaxRange, 3.5);
    end

    % Step B & C: Check each remaining waypoint
    for k = controllerState.currentWaypointIdx:numWaypoints
        wpPos = waypoints(k, :);
        delta = wpPos - robot.pose(1:2);
        distToWp = norm(delta);
        
        % Check LiDAR Range
        if distToWp <= lidarMaxRange
            inFOV = true;
            % Check FOV angle if lastScan provides LiDAR geometry
            if ~isempty(lastScan) && isprop(lastScan, 'Angles') && ~isempty(lastScan.Angles)
                angleToWpWorld = atan2(delta(2), delta(1));
                angleToWpBody = wrapAngleToPi(angleToWpWorld - robot.pose(3));
                minAngle = min(lastScan.Angles);
                maxAngle = max(lastScan.Angles);
                inFOV = (angleToWpBody >= minAngle) && (angleToWpBody <= maxAngle);
            end

            if inFOV
                % Line-of-Sight obstacle check (is path to waypoint unoccluded)
                if ~isPathBlocked(map, robot.pose, wpPos, robot.radius, localPlannerConfig)
                    % Step D: Increment scan count
                    controllerState.waypointScanCount(k) = controllerState.waypointScanCount(k) + 1;
                end
            end
        end
    end

    % Step F, G, H: Auto-skip waypoints covered at least 2 times by LiDAR
    % Advancing currentWaypointIdx automatically turns skipped points green
    % and advances the Gold Star target to the next unexplored point.
    while controllerState.currentWaypointIdx <= numWaypoints && ...
          controllerState.waypointScanCount(controllerState.currentWaypointIdx) >= 2
        controllerState.currentWaypointIdx = controllerState.currentWaypointIdx + 1;
    end

    if controllerState.currentWaypointIdx > numWaypoints
        robot.linearVel      = 0;
        robot.angularVel     = 0;
        robot.targetWaypoint = [NaN, NaN];
        robot.distanceToGoal = NaN;
        robot.headingError   = NaN;
        robot.navigationState= 'COMPLETED';
        isPathComplete       = true;
        return;
    end

    % ---------------------------------------------------------------
    % Stage 1: Controller — compute velocity commands
    % ---------------------------------------------------------------
    % ==========================================================
    % Navigation Finite State Machine
    % ==========================================================

    switch robot.navigationState

        case "NAVIGATING"

            % Normal controller execution.
            targetWaypoint = waypoints(controllerState.currentWaypointIdx,:);

            % --- Local Planner: check if direct path is blocked ---
            if hasLocalPlanner
                % Initialize local planner state if needed
                if ~isfield(robot, 'localPlannerState')
                    robot.localPlannerState = struct( ...
                        'active', false, ...
                        'tempWaypoint', [], ...
                        'originalWaypoint', [], ...
                        'replanCounter', 0, ...
                        'prevTempWaypoint', []);
                end

                lpState = robot.localPlannerState;

                % Check if we have an active temporary waypoint
                if lpState.active
                    % Verify temp waypoint is still valid and making progress
                    tempWp = lpState.tempWaypoint;
                    origWp = lpState.originalWaypoint;
                    
                    distToTemp = norm(robot.pose(1:2) - tempWp);
                    distToOrig = norm(robot.pose(1:2) - origWp);
                    tempToOrig = norm(tempWp - origWp);
                    
                    % Check if temp waypoint reached
                    tempTolerance = localPlannerConfig.waypointTolerance;
                    if isempty(tempTolerance)
                        tempTolerance = waypointTolerance * 1.5;
                    end
                    
                    if distToTemp <= tempTolerance
                        % Temp waypoint reached - clear and resume original
                        lpState.active = false;
                        lpState.tempWaypoint = [];
                        lpState.originalWaypoint = [];
                        lpState.replanCounter = 0;
                    elseif distToOrig < tempToOrig - 0.1
                        % We've passed the original waypoint or are closer to it than temp
                        lpState.active = false;
                        lpState.tempWaypoint = [];
                        lpState.originalWaypoint = [];
                        lpState.replanCounter = 0;
                    elseif ~checkCollision(map, [tempWp, robot.pose(3)], robot.safetyRadius + localPlannerConfig.minClearance)
                        % Temp waypoint no longer valid - need replan
                        lpState.active = false;
                    else
                        % Continue following temp waypoint
                        targetWaypoint = tempWp;
                        lpState.replanCounter = lpState.replanCounter + 1;
                    end
                else
                    % No active temp waypoint - check if direct path is blocked
                    pathBlocked = isPathBlocked(map, robot.pose, targetWaypoint, ...
                        robot.safetyRadius, localPlannerConfig);
                    
                    if pathBlocked
                        % Invoke local planner
                        [tempWaypoint, ~] = localPlanner( ...
                            map, robot.pose, targetWaypoint, robot.safetyRadius, ...
                            localPlannerConfig, lpState.prevTempWaypoint, lastScan, robot.recoveryAnalysis);
                        
                        if ~isempty(tempWaypoint)
                            % Activate temporary waypoint
                            lpState.active = true;
                            lpState.tempWaypoint = tempWaypoint;
                            lpState.originalWaypoint = targetWaypoint;
                            lpState.replanCounter = 0;
                            lpState.prevTempWaypoint = tempWaypoint;
                            targetWaypoint = tempWaypoint;
                        end
                    end
                end
                
                robot.localPlannerState = lpState;
            end

            [linearVel,angularVel,distanceToTarget,headingError] = ...
                computeControlCommands(robot,targetWaypoint);

            newPose = applyMotionModel(robot.pose,...
                                    linearVel,...
                                    angularVel,...
                                    dt);

            isSafe = checkCollision(map,...
                                    newPose,...
                                    robot.safetyRadius);
            if isSafe

                robot.pose = newPose;

                robot.collisionCounter = 0;

                robot.navigationState = "NAVIGATING";

                distanceToTarget = norm(robot.pose(1:2) - targetWaypoint);

                if distanceToTarget <= waypointTolerance

                    controllerState.currentWaypointIdx = ...
                        controllerState.currentWaypointIdx + 1;

                    if controllerState.currentWaypointIdx > numWaypoints
                        isPathComplete = true;
                    end

                end

            else

                robot.recoveryTimer = 0;

                % Increment once per recovery cycle so collisionThreshold
                % correctly counts full recovery attempts, not just
                % individual BACKUP-blocked steps.
                robot.collisionCounter = robot.collisionCounter + 1;

                robot.recoveryStartHeading = robot.pose(3);

                % --- Map-aware, LiDAR-assisted recovery decision ---
                if hasAnalyzer
                    recov = analyzeRecovery(map, lastScan, robot.pose, recoveryConfig, ...
                        robot.recoveryAnalysis, targetWaypoint);
                    robot.recoveryAnalysis = recov;

                    if recov.direction == 0
                        % Dead-end detected: skip directly to BACKUP.
                        robot.navigationState = "BACKUP";
                        robot.backupDistance  = 0;

                    elseif recov.direction > 0
                        % Left sector more open.
                        robot.navigationState   = "ROTATE_LEFT";
                        robot.recoveryDirection = 1;
                        robot.targetRecoveryHeading = ...
                            wrapAngleToPi(robot.pose(3) + recov.angle);

                    else
                        % Right sector more open: skip ROTATE_LEFT entirely.
                        robot.navigationState   = "ROTATE_RIGHT";
                        robot.recoveryDirection = -1;
                        robot.targetRecoveryHeading = ...
                            wrapAngleToPi(robot.pose(3) - recov.angle);
                    end

                else
                    % Fallback: original fixed left-turn behaviour.
                    robot.navigationState   = "ROTATE_LEFT";
                    robot.recoveryDirection = 1;
                    robot.targetRecoveryHeading = ...
                        wrapAngleToPi(robot.pose(3) + deg2rad(25));
                end

                linearVel  = 0;
                angularVel = 0;

            end                                                                  
        case "ROTATE_LEFT"

            linearVel = 0;

            angularVel = 0.6 * robot.maxAngularVel;

            newPose = applyMotionModel(robot.pose,...
                                    linearVel,...
                                    angularVel,...
                                    dt);

            if checkCollision(map,newPose,robot.safetyRadius)

                robot.pose = newPose;

                robot.recoveryTimer = robot.recoveryTimer + 1;

                angleRemaining = wrapAngleToPi( ...
                    robot.targetRecoveryHeading - robot.pose(3));

                if abs(angleRemaining) < deg2rad(3)

                    if hasAnalyzer
                        % Continuous re-evaluation: probe forward before
                        % committing to TRY_FORWARD.  Only proceed when
                        % the path ahead is actually clear.
                        probePose = applyMotionModel(robot.pose, ...
                            0.2 * robot.maxLinearVel, 0, dt);

                        if checkCollision(map, probePose, robot.safetyRadius)
                            % Forward clear — proceed.
                            robot.navigationState = "TRY_FORWARD";
                            robot.recoveryTimer   = 0;

                        elseif robot.recoveryTimer > recoveryConfig.maxRotateSteps
                            % Spent too long rotating left; switch direction.
                            newRecov = analyzeRecovery(map, lastScan, ...
                                robot.pose, recoveryConfig, ...
                                robot.recoveryAnalysis, waypoints(controllerState.currentWaypointIdx,:));
                            robot.navigationState = "ROTATE_RIGHT";
                            robot.targetRecoveryHeading = ...
                                wrapAngleToPi(robot.pose(3) - newRecov.angle);
                            robot.recoveryTimer = 0;

                        else
                            % Still blocked: re-analyse and continue rotating.
                            newRecov = analyzeRecovery(map, lastScan, ...
                                robot.pose, recoveryConfig, ...
                                robot.recoveryAnalysis, waypoints(controllerState.currentWaypointIdx,:));
                            if newRecov.direction >= 0
                                % Continue left with a fresh adaptive target.
                                robot.targetRecoveryHeading = ...
                                    wrapAngleToPi(robot.pose(3) + newRecov.angle);
                                % State stays ROTATE_LEFT.
                            else
                                % Sensor now says right is better.
                                robot.navigationState = "ROTATE_RIGHT";
                                robot.targetRecoveryHeading = ...
                                    wrapAngleToPi(robot.pose(3) - newRecov.angle);
                                robot.recoveryTimer = 0;
                            end
                        end

                    else
                        % Fallback: original behaviour.
                        robot.navigationState = "TRY_FORWARD";
                        robot.recoveryTimer   = 0;
                    end

                end

            else

                % Rotation itself blocked: move to ROTATE_RIGHT with
                % an adaptive angle from analyzeRecovery if available.
                robot.recoveryTimer = 0;

                if hasAnalyzer
                    newRecov = analyzeRecovery(map, lastScan, robot.pose, recoveryConfig, ...
                        robot.recoveryAnalysis, waypoints(controllerState.currentWaypointIdx,:));
                    robot.targetRecoveryHeading = ...
                        wrapAngleToPi(robot.pose(3) - newRecov.angle);
                else
                    % Use current heading as the baseline for the right-turn
                    % target so that the angle remaining to cover is always
                    % a fresh 25° sweep from wherever the robot ended up.
                    robot.targetRecoveryHeading = ...
                        wrapAngleToPi(robot.pose(3) - deg2rad(25));
                end

                robot.navigationState = "ROTATE_RIGHT";

            end

            distanceToTarget = robot.distanceToGoal;

            headingError = robot.headingError;

            targetWaypoint = waypoints(controllerState.currentWaypointIdx,:);

        case "TRY_FORWARD"

            targetWaypoint = waypoints(controllerState.currentWaypointIdx,:);

            [~, ~, ~, ~] = ...
                computeControlCommands(robot, targetWaypoint);

            % Drive slowly while testing if the new heading is clear.
            linearVel = 0.35 * robot.maxLinearVel;

            angularVel = 0;

            newPose = applyMotionModel(robot.pose,...
                                    linearVel,...
                                    angularVel,...
                                    dt);

            if checkCollision(map,newPose,robot.safetyRadius)

                robot.pose = newPose;

                robot.navigationState = "NAVIGATING";

                robot.collisionCounter = 0;

                % TRY_FORWARD succeeded: clear recovery memory so the
                % next collision starts fresh without stale history.
                if hasAnalyzer
                    robot.recoveryAnalysis.failedAttempts = 0;
                    robot.recoveryAnalysis.deadEndCount   = 0;
                    robot.recoveryAnalysis.lastDirection  = 0;
                    robot.recoveryAnalysis.lastEnvType    = 'UNKNOWN';
                end

                distanceToTarget = norm(robot.pose(1:2)-targetWaypoint);

                if distanceToTarget <= waypointTolerance

                    controllerState.currentWaypointIdx = ...
                        controllerState.currentWaypointIdx + 1;

                    if controllerState.currentWaypointIdx > numWaypoints
                        isPathComplete = true;
                    end

                end

            else

                % Forward attempt blocked: re-analyse to pick the best
                % rotation direction rather than always defaulting right.
                robot.recoveryTimer = 0;

                if hasAnalyzer
                    % Increment failed attempts BEFORE calling so the
                    % analyzer can apply anti-oscillation logic.
                    robot.recoveryAnalysis.failedAttempts = ...
                        robot.recoveryAnalysis.failedAttempts + 1;
                    newRecov = analyzeRecovery(map, lastScan, robot.pose, recoveryConfig, ...
                        robot.recoveryAnalysis, waypoints(controllerState.currentWaypointIdx,:));
                    robot.recoveryAnalysis = newRecov;
                    if newRecov.direction >= 0
                        robot.navigationState = "ROTATE_LEFT";
                        robot.targetRecoveryHeading = ...
                            wrapAngleToPi(robot.pose(3) + newRecov.angle);
                    else
                        robot.navigationState = "ROTATE_RIGHT";
                        robot.targetRecoveryHeading = ...
                            wrapAngleToPi(robot.pose(3) - newRecov.angle);
                    end
                else
                    % Use current heading as the baseline so the right-turn
                    % arc is always a fresh 25° sweep from the current pose.
                    robot.navigationState = "ROTATE_RIGHT";
                    robot.targetRecoveryHeading = ...
                        wrapAngleToPi(robot.pose(3) - deg2rad(25));
                end

                linearVel  = 0;
                angularVel = 0;

            end

        case "ROTATE_RIGHT"

            linearVel = 0;

            angularVel = -0.6 * robot.maxAngularVel;

            newPose = applyMotionModel(robot.pose,...
                                    linearVel,...
                                    angularVel,...
                                    dt);

            if checkCollision(map,newPose,robot.safetyRadius)

                robot.pose = newPose;

                robot.recoveryTimer = robot.recoveryTimer + 1;

                angleRemaining = wrapAngleToPi( ...
                    robot.targetRecoveryHeading - robot.pose(3));

                if abs(angleRemaining) < deg2rad(3)

                    % Continuous re-evaluation: probe forward.  If the
                    % right rotation opened a path, skip BACKUP entirely.
                    if hasAnalyzer
                        probePose = applyMotionModel(robot.pose, ...
                            0.2 * robot.maxLinearVel, 0, dt);
                        if checkCollision(map, probePose, robot.safetyRadius)
                            robot.navigationState = "TRY_FORWARD";
                        else
                            robot.navigationState = "BACKUP";
                        end
                    else
                        robot.navigationState = "BACKUP";
                    end

                    robot.recoveryTimer  = 0;
                    robot.backupDistance = 0;

                end

            else

                robot.navigationState = "BACKUP";

                robot.recoveryTimer = 0;

                robot.backupDistance = 0;

            end


            targetWaypoint = waypoints(controllerState.currentWaypointIdx,:);
            distanceToTarget = robot.distanceToGoal;
            headingError = robot.headingError;

        case "BACKUP"

            linearVel = -0.25 * robot.maxLinearVel;

            angularVel = 0;

            newPose = applyMotionModel(robot.pose,...
                                    linearVel,...
                                    angularVel,...
                                    dt);

            if checkCollision(map,newPose,robot.safetyRadius)

                robot.pose = newPose;

                robot.backupDistance = robot.backupDistance + ...
                                    abs(linearVel * dt);

                if robot.backupDistance >= 0.25

                    robot.navigationState = "TRY_FORWARD";

                    % Do NOT reset collisionCounter here — it accumulates
                    % across the full ROTATE_LEFT→TRY_FORWARD→ROTATE_RIGHT
                    % →BACKUP cycle and is only cleared on waypoint advance.

                end

            else

                if robot.collisionCounter >= collisionThreshold

                    fprintf("Recovery failed. Skipping waypoint %d\n", ...
                        controllerState.currentWaypointIdx);

                    controllerState.currentWaypointIdx = ...
                        controllerState.currentWaypointIdx + 1;

                    robot.navigationState = "NAVIGATING";

                    robot.collisionCounter = 0;

                    robot.backupDistance = 0;

                    if controllerState.currentWaypointIdx > numWaypoints
                        isPathComplete = true;
                    end

                else

                    % Cannot back up either; re-analyse from the current
                    % (possibly slightly different) pose to choose the
                    % better rotation direction.
                    robot.recoveryTimer        = 0;
                    robot.recoveryStartHeading = robot.pose(3);
                    robot.backupDistance        = 0;

                    if hasAnalyzer
                        newRecov = analyzeRecovery(map, lastScan, ...
                            robot.pose, recoveryConfig, ...
                            robot.recoveryAnalysis, waypoints(controllerState.currentWaypointIdx,:));
                        if newRecov.direction >= 0
                            robot.navigationState = "ROTATE_LEFT";
                            robot.targetRecoveryHeading = ...
                                wrapAngleToPi(robot.pose(3) + newRecov.angle);
                        else
                            robot.navigationState = "ROTATE_RIGHT";
                            robot.targetRecoveryHeading = ...
                                wrapAngleToPi(robot.pose(3) - newRecov.angle);
                        end
                    else
                        % Fallback: original fixed left retry.
                        robot.navigationState = "ROTATE_LEFT";
                        robot.targetRecoveryHeading = ...
                            wrapAngleToPi(robot.pose(3) + deg2rad(25));
                    end

                end

            end

            targetWaypoint = waypoints(controllerState.currentWaypointIdx,:);
            distanceToTarget = robot.distanceToGoal;
            headingError = robot.headingError;

        otherwise

            robot.navigationState = "NAVIGATING";

    end

    % ---------------------------------------------------------------
    % Stage 2: State Update — populate diagnostics
    % ---------------------------------------------------------------
    if exist('linearVel','var')
        robot.linearVel = linearVel;
    else
        robot.linearVel = 0;
    end

    if exist('angularVel','var')
        robot.angularVel = angularVel;
    else
        robot.angularVel = 0;
    end

    if exist('targetWaypoint','var')
        robot.targetWaypoint = targetWaypoint;
    end

    if exist('distanceToTarget','var')
        robot.distanceToGoal = distanceToTarget;
    end

    if exist('headingError','var')
        robot.headingError = headingError;
    end
end

% =====================================================================
% Stage 1: Controller
% =====================================================================

function [linearVel, angularVel, distanceToTarget, headingError] = computeControlCommands(robot, targetWaypoint)
%COMPUTECONTROLCOMMANDS Proportional heading controller with speed shaping.

    headingGain          = 2.0;
    slowdownRadius       = 0.6;     % meters
    headingSlowdownAngle = pi / 3;  % radians

    currentPose = robot.pose;

    deltaX = targetWaypoint(1) - currentPose(1);
    deltaY = targetWaypoint(2) - currentPose(2);
    distanceToTarget = hypot(deltaX, deltaY);

    desiredHeading = atan2(deltaY, deltaX);
    headingError = wrapAngleToPi(desiredHeading - currentPose(3));

    % Angular velocity: proportional to heading error, clamped.
    angularVel = headingGain * headingError;
    angularVel = max(-robot.maxAngularVel, min(robot.maxAngularVel, angularVel));

    % Linear velocity: throttled by heading error and proximity.
    headingThrottle   = max(0, 1 - abs(headingError) / headingSlowdownAngle);
    proximityThrottle = min(1, distanceToTarget / slowdownRadius);
    linearVel = robot.maxLinearVel * headingThrottle * proximityThrottle;
    linearVel = max(0, min(robot.maxLinearVel, linearVel));
end

% =====================================================================
% Stage 2: Motion Model (Midpoint Integration)
% =====================================================================

function newPose = applyMotionModel(pose, linearVel, angularVel, dt)
%APPLYMOTIONMODEL Midpoint integration of the unicycle kinematic model.

    thetaHalf = pose(3) + 0.5 * angularVel * dt;

    newX     = pose(1) + linearVel * cos(thetaHalf) * dt;
    newY     = pose(2) + linearVel * sin(thetaHalf) * dt;
    newTheta = wrapAngleToPi(pose(3) + angularVel * dt);

    newPose = [newX, newY, newTheta];
end

% =====================================================================
% Stage 3: Collision Check (Dynamic Ring Sampling)
% =====================================================================

function isSafe = checkCollision(map, pose, safetyRadius)
%CHECKCOLLISION Check whether a candidate pose is collision-free.
%   Uses dynamic ring sampling: numSamples scales with safetyRadius
%   and map resolution so collision accuracy adapts to robot size.
%
%   Out-of-bounds ring samples are SKIPPED (not treated as collisions).
%   Only the center point must be strictly inside map bounds. This
%   prevents the robot from getting permanently stuck near walls and
%   doorways where the safety ring slightly overhangs the map edge.

    xLimits = map.XWorldLimits;
    yLimits = map.YWorldLimits;

    % Strict bounds check on the robot center — the center must always
    % be inside the map.
    if pose(1) < xLimits(1) || pose(1) > xLimits(2) || ...
       pose(2) < yLimits(1) || pose(2) > yLimits(2)
        isSafe = false;
        return;
    end

    % Dynamic sample count: scales with circumference and resolution.
    numRingSamples = max(12, ceil(2 * pi * safetyRadius * map.Resolution));
    angles = linspace(0, 2*pi, numRingSamples + 1);
    angles(end) = [];

    % Outer ring at safetyRadius + inner ring at half-radius for
    % better narrow-passage and doorway detection.
    samplePoints = [pose(1), pose(2)];  % always include center
    for ringScale = [1.0, 0.5]
        r = safetyRadius * ringScale;
        for k = 1:numRingSamples
            pt = [pose(1) + r * cos(angles(k)), ...
                  pose(2) + r * sin(angles(k))];
            % Skip ring points that poke outside the map — the outer
            % wall already blocks the robot via occupancy; silently
            % dropping out-of-bounds samples avoids false collision
            % reports for robots legitimately moving near boundaries.
            if pt(1) >= xLimits(1) && pt(1) <= xLimits(2) && ...
               pt(2) >= yLimits(1) && pt(2) <= yLimits(2)
                samplePoints(end+1, :) = pt; %#ok<AGROW>
            end
        end
    end

    occupiedFlags = checkOccupancy(map, samplePoints);
    isSafe = ~any(occupiedFlags);
end

% =====================================================================
% Shared helper
% =====================================================================

function wrapped = wrapAngleToPi(angle)
%WRAPANGLETOPI Wrap angle to (-pi, pi].
    wrapped = atan2(sin(angle), cos(angle));
end

% =====================================================================
% Local Planner helpers
% =====================================================================

function blocked = isPathBlocked(map, robotPose, targetWaypoint, safetyRadius, config)
%ISPATHBLOCKED Check if direct path to waypoint is collision-free.
%
%   blocked = ISPATHBLOCKED(map, robotPose, targetWaypoint, safetyRadius, config)
%
%   Samples points along the straight line from robot to waypoint and
%   checks occupancy. Returns true if any sample is occupied or out of bounds.
%
%   config fields (optional):
%       .pathCheckResolution - distance between samples (m), default 0.1
%       .pathCheckInflation  - extra inflation for safety check (m), default 0.1

    if nargin < 5 || isempty(config)
        config = struct();
    end
    
    resolution = config.pathCheckResolution;
    if isempty(resolution) || resolution <= 0
        resolution = 0.1;
    end
    
    inflation = config.pathCheckInflation;
    if isempty(inflation)
        inflation = 0.1;
    end
    
    checkRadius = safetyRadius + inflation;
    
    vec = targetWaypoint - robotPose(1:2);
    dist = norm(vec);
    
    if dist < 1e-6
        blocked = false;
        return;
    end
    
    numSamples = max(2, ceil(dist / resolution));
    t = linspace(0, 1, numSamples)';
    
    % Sample points along line
    samplePoints = robotPose(1:2) + t * vec;
    
    % Check each sample
    for i = 1:numSamples
        pt = samplePoints(i, :);
        % Use checkCollision with a dummy pose at the sample point
        dummyPose = [pt, robotPose(3)];
        if ~checkCollision(map, dummyPose, checkRadius)
            blocked = true;
            return;
        end
    end
    
    blocked = false;
end
