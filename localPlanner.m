function [tempWaypoint, plannerInfo] = localPlanner(map, robotPose, targetWaypoint, safetyRadius, config, prevTempWaypoint, lastScan, recoveryAnalysis)
%LOCALPLANNER Generate temporary waypoints to navigate around obstacles.
%
%   [tempWaypoint, plannerInfo] = LOCALPLANNER(map, robotPose, targetWaypoint, ...
%       safetyRadius, config, prevTempWaypoint, lastScan, recoveryAnalysis)
%
%   Implements a local planner that generates temporary waypoints when the direct path to the
%   target waypoint is blocked by obstacles in the occupancy map.
%
%   Strategy:
%     1. Sample candidate points in a forward-biased arc around the robot
%     2. Reject candidates in occupied space (considering safety radius)
%     3. Score remaining candidates by:
%          - Progress toward goal (distance to target waypoint)
%          - Clearance from obstacles
%          - Heading change required (smoothness)
%     4. Apply environment-specific biasing:
%          - WALL: prefer points parallel to wall (wall-following)
%          - OBJECT: prefer points that go around the object
%          - UNKNOWN/DEADEND: use LiDAR sector clearance
%     5. Select best candidate as temporary waypoint
%     6. Hysteresis: prefer previous temporary waypoint if still valid
%
%   Inputs:
%       map              - binaryOccupancyMap (SLAM or ground truth)
%       robotPose        - 1x3 [x y theta] current robot pose
%       targetWaypoint   - 1x2 [x y] original waypoint to reach
%       safetyRadius     - robot safety radius (m)
%       config           - config.localPlanner struct from main.m
%       prevTempWaypoint - 1x2 previous temporary waypoint (or [])
%       lastScan         - lidarScan object for LiDAR-based scoring (optional)
%       recoveryAnalysis - struct from analyzeRecovery (optional)
%
%   Outputs:
%       tempWaypoint - 1x2 [x y] temporary waypoint, or [] if no valid candidate
%       plannerInfo  - struct with diagnostic info:
%           .candidatesSampled    - number of candidates evaluated
%           .candidatesValid      - number of collision-free candidates
%           .selectedScore        - score of selected waypoint
%           .envType              - 'WALL'|'OBJECT'|'UNKNOWN'|'DEADEND'|'CLEAR'
%           .usedPrevious         - true if prevTempWaypoint was reused
%           .candidateScores      - Nx3 matrix [distToGoal, clearance, headingChange]
%
%   Config fields (config.localPlanner):
%       .candidateRadius       - radius to sample candidates (m), default 2.0
%       .numCandidates         - number of candidate points, default 32
%       .forwardBias           - fraction of candidates in front 180 deg, default 0.7
%       .scoreWeights          - [wGoal, wClearance, wSmooth] default [1.0, 0.5, 0.3]
%       .minClearance          - minimum clearance from obstacles (m), default 0.3
%       .hysteresisRadius      - reuse prev waypoint if within this dist (m), default 0.5
%       .wallFollowOffset      - lateral offset for wall-following (m), default 0.5
%       .objectClearance       - extra clearance for objects (m), default 0.4
%       .maxHeadingChange      - max heading change per step (rad), default pi/2
%       .replanInterval        - steps between full replans, default 5
%
%   Coordinate convention: world frame, x right, y up, theta CCW from +x.

    % ---------------------------------------------------------------
    % Default config
    % ---------------------------------------------------------------
    if nargin < 5 || isempty(config)
        config = struct();
    end
    
    defaults = struct( ...
        'candidateRadius', 2.0, ...
        'numCandidates', 32, ...
        'forwardBias', 0.7, ...
        'scoreWeights', [1.0, 0.5, 0.3], ...
        'minClearance', 0.3, ...
        'hysteresisRadius', 0.5, ...
        'wallFollowOffset', 0.5, ...
        'objectClearance', 0.4, ...
        'maxHeadingChange', pi/2, ...
        'replanInterval', 5);
    
    config = structMerge(defaults, config);
    
    % ---------------------------------------------------------------
    % Default optional arguments
    % ---------------------------------------------------------------
    if nargin < 6 || isempty(prevTempWaypoint)
        prevTempWaypoint = [];
    end
    if nargin < 7
        lastScan = [];
    end
    if nargin < 8
        recoveryAnalysis = [];
    end
    
    % ---------------------------------------------------------------
    % Classify environment using recovery analysis or map
    % ---------------------------------------------------------------
    envType = 'UNKNOWN';
    wallDirection = []; % unit vector along wall (robot frame)
    
    if ~isempty(recoveryAnalysis) && isfield(recoveryAnalysis, 'envType')
        envType = recoveryAnalysis.envType;
        
        % Extract wall direction from PCA if available
        if strcmpi(envType, 'WALL') && isfield(recoveryAnalysis, 'wallDirection')
            wallDirection = recoveryAnalysis.wallDirection;
        end
    end
    
    % Fallback: classify from map if recovery analysis unavailable
    if strcmpi(envType, 'UNKNOWN') || isempty(recoveryAnalysis)
        [envType, ~, wallDirection] = classifyFromMap(map, robotPose, config);
    end
    
    % ---------------------------------------------------------------
    % Hysteresis: reuse previous temporary waypoint if still valid
    % ---------------------------------------------------------------
    if ~isempty(prevTempWaypoint)
        distToPrev = norm(robotPose(1:2) - prevTempWaypoint);
        if distToPrev <= config.hysteresisRadius
            % Check if previous waypoint is still collision-free
            if isPointFree(map, prevTempWaypoint, safetyRadius + config.minClearance)
                % Check if it still makes progress toward goal
                prevToGoal = norm(prevTempWaypoint - targetWaypoint);
                robotToGoal = norm(robotPose(1:2) - targetWaypoint);
                if prevToGoal < robotToGoal + 0.1  % allow small tolerance
                    tempWaypoint = prevTempWaypoint;
                    plannerInfo = struct( ...
                        'candidatesSampled', 0, ...
                        'candidatesValid', 1, ...
                        'selectedScore', 1.0, ...
                        'envType', envType, ...
                        'usedPrevious', true, ...
                        'candidateScores', []);
                    return;
                end
            end
        end
    end
    
    % ---------------------------------------------------------------
    % Sample candidate points around robot
    % ---------------------------------------------------------------
    candidates = sampleCandidates(robotPose, config);
    numCandidates = size(candidates, 1);
    
    % ---------------------------------------------------------------
    % Filter collision-free candidates with adequate clearance
    % ---------------------------------------------------------------
    validMask = false(numCandidates, 1);
    clearances = zeros(numCandidates, 1);
    
    for i = 1:numCandidates
        pt = candidates(i, :);
        [isFree, clearance] = checkPointClearance(map, pt, safetyRadius + config.minClearance);
        validMask(i) = isFree;
        clearances(i) = clearance;
    end
    
    validIndices = find(validMask);
    numValid = numel(validIndices);
    
    if numValid == 0
        % No valid candidates - return empty, recovery will handle
        tempWaypoint = [];
        plannerInfo = struct( ...
            'candidatesSampled', numCandidates, ...
            'candidatesValid', 0, ...
            'selectedScore', -inf, ...
            'envType', envType, ...
            'usedPrevious', false, ...
            'candidateScores', []);
        return;
    end
    
    % ---------------------------------------------------------------
    % Score valid candidates
    % ---------------------------------------------------------------
    scores = zeros(numValid, 1);
    candidateScores = zeros(numValid, 3); % [goalProgress, clearance, smoothness]
    
    robotToGoalVec = targetWaypoint - robotPose(1:2);
    robotToGoalDist = norm(robotToGoalVec);
    robotHeading = robotPose(3);
    
    % Get LiDAR sector scores if available
    lidarLeft = config.lidarMaxRange;
    lidarRight = config.lidarMaxRange;
    if ~isempty(lastScan) && isfield(config, 'lidarMaxRange')
        [lidarLeft, ~, lidarRight] = computeSectorScores(lastScan, config);
    end
    
    for idx = 1:numValid
        i = validIndices(idx);
        pt = candidates(i, :);
        
        % Score 1: Progress toward goal (negative distance to goal)
        distToGoal = norm(pt - targetWaypoint);
        goalProgress = 1.0 - min(1.0, distToGoal / max(robotToGoalDist, 0.1));
        candidateScores(idx, 1) = goalProgress;
        
        % Score 2: Clearance (normalized)
        clearanceScore = min(1.0, clearances(i) / (config.candidateRadius * 0.5));
        candidateScores(idx, 2) = clearanceScore;
        
        % Score 3: Heading smoothness (penalize sharp turns)
        headingToPt = atan2(pt(2) - robotPose(2), pt(1) - robotPose(1));
        headingChange = abs(wrapAngleToPi(headingToPt - robotHeading));
        smoothness = 1.0 - min(1.0, headingChange / config.maxHeadingChange);
        candidateScores(idx, 3) = smoothness;
        
        % Base weighted score
        scores(idx) = config.scoreWeights(1) * goalProgress + ...
                      config.scoreWeights(2) * clearanceScore + ...
                      config.scoreWeights(3) * smoothness;
        
        % -----------------------------------------------------------
        % Environment-specific biasing
        % -----------------------------------------------------------
        switch envType
            case 'WALL'
                % Bias toward points parallel to wall (wall-following)
                if ~isempty(wallDirection)
                    % Wall direction in robot frame: project candidate onto wall
                    ptRel = pt - robotPose(1:2);
                    perpToWall = abs(dot(ptRel, [-wallDirection(2), wallDirection(1)]));
                    
                    % Prefer points that maintain wall-following offset
                    targetOffset = config.wallFollowOffset;
                    offsetError = abs(perpToWall - targetOffset);
                    wallFollowScore = exp(-offsetError / 0.3); % Gaussian-like
                    
                    % Strongly prefer forward progress along wall
                    forwardAlongWall = dot(ptRel, wallDirection);
                    if forwardAlongWall > 0
                        wallFollowScore = wallFollowScore * 1.5;
                    end
                    
                    scores(idx) = scores(idx) + 0.8 * wallFollowScore;
                end
                
            case 'OBJECT'
                % Bias toward going around object (prefer side with more clearance)
                if ~isempty(lastScan)
                    % Use LiDAR to pick side
                    if lidarLeft > lidarRight
                        % Prefer left-side candidates
                        relAngle = wrapAngleToPi(atan2(pt(2)-robotPose(2), pt(1)-robotPose(1)) - robotHeading);
                        if relAngle > 0
                            scores(idx) = scores(idx) + 0.5 * (relAngle / (pi/2));
                        end
                    else
                        % Prefer right-side candidates
                        relAngle = wrapAngleToPi(atan2(pt(2)-robotPose(2), pt(1)-robotPose(1)) - robotHeading);
                        if relAngle < 0
                            scores(idx) = scores(idx) + 0.5 * (abs(relAngle) / (pi/2));
                        end
                    end
                end
                
            case 'DEADEND'
                % Prefer backing up - candidates behind robot
                relAngle = wrapAngleToPi(atan2(pt(2)-robotPose(2), pt(1)-robotPose(1)) - robotHeading);
                if abs(relAngle) > pi/2
                    scores(idx) = scores(idx) + 0.7;
                end
                
            otherwise % 'UNKNOWN' or 'CLEAR'
                % Use LiDAR clearance bias if available
                if ~isempty(lastScan)
                    relAngle = wrapAngleToPi(atan2(pt(2)-robotPose(2), pt(1)-robotPose(1)) - robotHeading);
                    if relAngle > 0
                        % Left side
                        lidarScore = lidarLeft / config.lidarMaxRange;
                    else
                        % Right side
                        lidarScore = lidarRight / config.lidarMaxRange;
                    end
                    scores(idx) = scores(idx) + 0.4 * lidarScore;
                end
        end
    end
    
    % ---------------------------------------------------------------
    % Select best candidate
    % ---------------------------------------------------------------
    [~, bestIdx] = max(scores);
    tempWaypoint = candidates(validIndices(bestIdx), :);
    selectedScore = scores(bestIdx);
    
    plannerInfo = struct( ...
        'candidatesSampled', numCandidates, ...
        'candidatesValid', numValid, ...
        'selectedScore', selectedScore, ...
        'envType', envType, ...
        'usedPrevious', false, ...
        'candidateScores', candidateScores);
end


% =====================================================================
% Helper: Check if straight-line path to waypoint is blocked
% =====================================================================
function blocked = isPathBlocked(map, robotPose, targetWaypoint, safetyRadius, config) %#ok<DEFNU>
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
        [isFree, ~] = checkPointClearance(map, pt, checkRadius);
        if ~isFree
            blocked = true;
            return;
        end
    end
    
    blocked = false;
end


% =====================================================================
% Helper: Sample candidate points around robot
% =====================================================================
function candidates = sampleCandidates(robotPose, config)
%SAMPLECANDIDATES Generate candidate waypoints in a forward-biased arc.

    numCandidates = config.numCandidates;
    radius = config.candidateRadius;
    forwardBias = config.forwardBias;
    
    candidates = zeros(numCandidates, 2);
    
    % Number of candidates in forward 180 deg vs backward 180 deg
    nForward = round(numCandidates * forwardBias);
    nBackward = numCandidates - nForward;
    
    % Forward arc: -90 to +90 degrees relative to robot heading
    if nForward > 0
        forwardAngles = linspace(-pi/2, pi/2, nForward)';
    else
        forwardAngles = [];
    end
    
    % Backward arc: +90 to +270 (or -90 to -270)
    if nBackward > 0
        backwardAngles = linspace(pi/2, 3*pi/2, nBackward)';
    else
        backwardAngles = [];
    end
    
    % Add some radial variation (not all at max radius)
    radii = radius * (0.3 + 0.7 * rand(numCandidates, 1));
    
    allAngles = [forwardAngles; backwardAngles];
    
    % Shuffle to avoid bias in selection
    shuffleIdx = randperm(numCandidates);
    allAngles = allAngles(shuffleIdx);
    radii = radii(shuffleIdx);
    
    cosT = cos(robotPose(3));
    sinT = sin(robotPose(3));
    
    for i = 1:numCandidates
        angle = allAngles(i);
        r = radii(i);
        % Rotate to world frame
        localX = r * cos(angle);
        localY = r * sin(angle);
        worldX = robotPose(1) + cosT * localX - sinT * localY;
        worldY = robotPose(2) + sinT * localX + cosT * localY;
        candidates(i, :) = [worldX, worldY];
    end
end


% =====================================================================
% Helper: Check point clearance from obstacles
% =====================================================================
function [isFree, clearance] = checkPointClearance(map, point, checkRadius)
%CHECKPOINTCLEARANCE Check if a point has adequate clearance.

    % Quick bounds check
    if point(1) < map.XWorldLimits(1) || point(1) > map.XWorldLimits(2) || ...
       point(2) < map.YWorldLimits(1) || point(2) > map.YWorldLimits(2)
        isFree = false;
        clearance = 0;
        return;
    end
    
    % Sample ring around point
    numSamples = max(8, ceil(2 * pi * checkRadius * map.Resolution));
    angles = linspace(0, 2*pi, numSamples + 1);
    angles(end) = [];
    
    allFree = true;
    
    for k = 1:numSamples
        samplePt = [point(1) + checkRadius * cos(angles(k)), ...
                    point(2) + checkRadius * sin(angles(k))];
        
        if samplePt(1) < map.XWorldLimits(1) || samplePt(1) > map.XWorldLimits(2) || ...
           samplePt(2) < map.YWorldLimits(1) || samplePt(2) > map.YWorldLimits(2)
            allFree = false;
            break;
        end
        
        occ = checkOccupancy(map, samplePt);
        if occ
            allFree = false;
            break;
        end
    end
    
    % If all ring samples free, estimate clearance by ray casting to nearest obstacle
    if allFree
        clearance = estimateClearance(map, point, checkRadius * 3);
    else
        clearance = 0;
    end
    
    isFree = allFree;
end


% =====================================================================
% Helper: Estimate clearance from point to nearest obstacle
% =====================================================================
function clearance = estimateClearance(map, point, maxDist)
%ESTIMATECLEARANCE Ray cast in multiple directions to find nearest obstacle.

    numRays = 16;
    angles = linspace(0, 2*pi, numRays + 1);
    angles(end) = [];
    
    grid = getOccupancy(map);
    [numRows, numCols] = size(grid);
    resolution = map.Resolution;
    xLimits = map.XWorldLimits;
    yLimits = map.YWorldLimits;
    
    minDist = maxDist;
    
    for k = 1:numRays
        dx = cos(angles(k));
        dy = sin(angles(k));
        
        % Ray march
        steps = ceil(maxDist * resolution);
        for s = 1:steps
            px = point(1) + dx * s / resolution;
            py = point(2) + dy * s / resolution;
            
            if px < xLimits(1) || px > xLimits(2) || ...
               py < yLimits(1) || py > yLimits(2)
                break;
            end
            
            col = round(px * resolution) + 1;
            row = numRows - round(py * resolution);
            
            if row >= 1 && row <= numRows && col >= 1 && col <= numCols
                if grid(row, col)
                    dist = s / resolution;
                    minDist = min(minDist, dist);
                    break;
                end
            end
        end
    end
    
    clearance = minDist;
end


% =====================================================================
% Helper: Classify environment from map (fallback when no recovery analysis)
% =====================================================================
function [envType, confidence, wallDirection] = classifyFromMap(map, robotPose, config)
%CLASSIFYFROMMAP Simple map-based environment classification.

    envType = 'UNKNOWN';
    confidence = 0.3;
    wallDirection = [];
    
    if isempty(map)
        return;
    end
    
    % Sample forward sector
    theta = robotPose(3);
    cosT = cos(theta);
    sinT = sin(theta);
    
    probeRadius = config.candidateRadius;
    numRays = 20;
    angles = linspace(-pi/3, pi/3, numRays); % forward 120 deg
    
    occupiedCount = 0;
    
    grid = getOccupancy(map);
    [numRows, numCols] = size(grid);
    resolution = map.Resolution;
    xLimits = map.XWorldLimits;
    yLimits = map.YWorldLimits;
    
    occupiedPoints = zeros(numRays, 2);
    
    for k = 1:numRays
        for r = 1:ceil(probeRadius * resolution)
            px = robotPose(1) + cosT * r/resolution * cos(angles(k)) - sinT * r/resolution * sin(angles(k));
            py = robotPose(2) + sinT * r/resolution * cos(angles(k)) + cosT * r/resolution * sin(angles(k));
            
            if px < xLimits(1) || px > xLimits(2) || py < yLimits(1) || py > yLimits(2)
                break;
            end
            
            col = round(px * resolution) + 1;
            row = numRows - round(py * resolution);
            
            if row >= 1 && row <= numRows && col >= 1 && col <= numCols
                if grid(row, col)
                    occupiedCount = occupiedCount + 1;
                    % Store in robot frame
                    localX = cosT * (px - robotPose(1)) + sinT * (py - robotPose(2));
                    localY = -sinT * (px - robotPose(1)) + cosT * (py - robotPose(2));
                    occupiedPoints(occupiedCount, :) = [localX, localY];
                    break; % stop at first hit per ray
                end
            end
        end
    end
    
    occupiedPoints = occupiedPoints(1:occupiedCount, :);
    
    if occupiedCount < 3
        envType = 'CLEAR';
        confidence = 0.7;
        return;
    end
    
    % PCA on occupied points to detect wall vs object
    if size(occupiedPoints, 1) >= 3
        mu = mean(occupiedPoints, 1);
        centred = occupiedPoints - mu;
        C = (centred' * centred) / (size(occupiedPoints, 1) - 1);
        [V, D] = eig(C);
        eigvals = diag(D);
        [~, sortIdx] = sort(eigvals, 'descend');
        V = V(:, sortIdx);
        
        spanMajor = max(centred * V(:,1)) - min(centred * V(:,1));
        spanMinor = max(centred * V(:,2)) - min(centred * V(:,2));
        aspectRatio = spanMajor / max(spanMinor, 0.01);
        
        % Wall direction (major axis) in robot frame
        wallDirection = V(:, 1)'; % row vector
        
        if spanMajor > 0.8 && aspectRatio > 2.0
            envType = 'WALL';
            confidence = min(1.0, aspectRatio / 5.0);
        elseif spanMajor < 0.7 && spanMinor < 0.7
            envType = 'OBJECT';
            confidence = 0.7;
        else
            envType = 'UNKNOWN';
            confidence = 0.4;
        end
    end
end


% =====================================================================
% Helper: Compute LiDAR sector scores (copied from analyzeRecovery)
% =====================================================================
function [leftScore, frontScore, rightScore] = computeSectorScores(scan, config)
    default = config.lidarMaxRange;
    
    if isempty(scan)
        leftScore = default;
        frontScore = default;
        rightScore = default;
        return;
    end
    
    ranges = min(scan.Ranges, config.lidarMaxRange);
    angles = scan.Angles;
    
    frontMask = abs(angles) <= config.frontSectorHalfAngle;
    leftMask = angles > config.sideInnerAngle & angles <= config.sideOuterAngle;
    rightMask = angles < -config.sideInnerAngle & angles >= -config.sideOuterAngle;
    
    leftScore = sectorScore(ranges(leftMask), default);
    frontScore = sectorScore(ranges(frontMask), default);
    rightScore = sectorScore(ranges(rightMask), default);
end

function s = sectorScore(sectorRanges, default)
    if isempty(sectorRanges)
        s = default;
    else
        s = 0.5 * median(sectorRanges) + 0.5 * min(sectorRanges);
    end
end


% =====================================================================
% Helper: Check if point is free (with radius)
% =====================================================================
function isFree = isPointFree(map, point, radius)
    [isFree, ~] = checkPointClearance(map, point, radius);
end


% =====================================================================
% Helper: Struct merge for defaults
% =====================================================================
function merged = structMerge(defaults, overrides)
    merged = defaults;
    if isstruct(overrides)
        fields = fieldnames(overrides);
        for i = 1:numel(fields)
            merged.(fields{i}) = overrides.(fields{i});
        end
    end
end


% =====================================================================
% Helper: Wrap angle to (-pi, pi]
% =====================================================================
function wrapped = wrapAngleToPi(angle)
    wrapped = atan2(sin(angle), cos(angle));
end