function result = analyzeRecovery(map, scan, robotPose, cfg, prevResult, targetWaypoint)
%ANALYZERECOVERY Map-aware, LiDAR-assisted recovery decision engine.
%
%   result = ANALYZERECOVERY(map, scan, robotPose, cfg)
%   result = ANALYZERECOVERY(map, scan, robotPose, cfg, prevResult)
%   result = ANALYZERECOVERY(map, scan, robotPose, cfg, prevResult, targetWaypoint)
%
%   Fuses the occupancy map with the latest LiDAR scan across six phases:
%     1. LiDAR sector scores  (front / left / right clearance)
%     2. Map classification   (PCA + continuity for wall; cluster for object)
%     3. Dead-end persistence (requires N consecutive readings before committing)
%     4. Direction score      (LiDAR clearance + optional waypoint-heading bias)
%     5. Anti-oscillation     (recovery memory — flips direction after failures)
%     6. Smooth adaptive angle (smooth-step curve, confidence-scaled, escalating)
%
%   The result struct is consumed by simulateRobotStep; FSM states are
%   unchanged.  Pass prevResult = robot.recoveryAnalysis to enable memory.
%
%   Inputs:
%       map           - binaryOccupancyMap; [] → skip map analysis
%       scan          - lidarScan from simulateLiDAR; [] → LiDAR defaults to max
%       robotPose     - 1×3 [x y theta] world frame (m / rad)
%       cfg           - config.recovery struct from main.m
%       prevResult    - (optional) previous call's result struct for memory
%       targetWaypoint- (optional) 1×2 [x y] current target (waypoint-aware mode)
%
%   Output fields:
%       .direction           +1 left | -1 right | 0 reverse
%       .angle               adaptive turn (rad)
%       .envType             'WALL'|'OBJECT'|'CORNER'|'DEADEND'|'UNKNOWN'
%       .confidence          map classification confidence 0–1
%       .directionConfidence |L−R|/(L+R) ∈ [0,1]; 0 = tied, 1 = one side blocked
%       .leftScore           LiDAR clearance, left sector (m)
%       .frontScore          LiDAR clearance, front sector (m)
%       .rightScore          LiDAR clearance, right sector (m)
%       .lastDirection       direction chosen (for next call's prevResult)
%       .lastEnvType         envType chosen (for next call's prevResult)
%       .failedAttempts      caller-managed failure count (read-back only)
%       .deadEndCount        consecutive DEADEND readings (persistence counter)

    % ------------------------------------------------------------------
    % Optional argument defaults
    % ------------------------------------------------------------------
    if nargin < 5; prevResult     = []; end
    if nargin < 6; targetWaypoint = []; end

    hasHistory = ~isempty(prevResult) && isstruct(prevResult) && ...
                 isfield(prevResult, 'failedAttempts');

    % ------------------------------------------------------------------
    % Phase 1: LiDAR sector scores
    % ------------------------------------------------------------------
    [leftScore, frontScore, rightScore] = computeSectorScores(scan, cfg);

    % ------------------------------------------------------------------
    % Phase 2: Map classification — PCA-based wall + cluster-based object
    % ------------------------------------------------------------------
    [rawEnvType, mapConfidence] = classifyEnvironment(map, robotPose, cfg);

    % ------------------------------------------------------------------
    % Phase 3: Dead-end persistence
    %
    %   Transient scan noise can cause a single-frame DEADEND reading.
    %   Require cfg.deadEndPersistenceCount consecutive readings before
    %   committing — prevents unnecessary large-rotation escapes.
    % ------------------------------------------------------------------
    prevDeadEndCount = 0;
    if hasHistory && isfield(prevResult, 'deadEndCount')
        prevDeadEndCount = prevResult.deadEndCount;
    end

    if strcmpi(rawEnvType, 'DEADEND')
        deadEndCount = prevDeadEndCount + 1;
    else
        deadEndCount = 0;
    end

    if deadEndCount >= cfg.deadEndPersistenceCount
        envType = 'DEADEND';
    elseif strcmpi(rawEnvType, 'DEADEND')
        % Not yet confirmed — downgrade to previous class or UNKNOWN.
        if hasHistory && ~strcmpi(prevResult.lastEnvType, 'UNKNOWN')
            envType = prevResult.lastEnvType;
        else
            envType = 'UNKNOWN';
        end
    else
        envType = rawEnvType;
    end

    % ------------------------------------------------------------------
    % Phase 4: Direction score — LiDAR clearance + waypoint-heading bias
    %
    %   combinedDiff = clearanceWeight × (leftScore − rightScore)
    %                + waypointBias
    %
    %   waypointBias is positive when the waypoint is to the robot's left,
    %   negative when to the right.  It nudges direction toward the goal
    %   when both sides are similarly open, without overriding a clear
    %   LiDAR advantage on either side.
    %
    %   Confidence (Issue 4):
    %       directionConfidence = |L − R| / (L + R)
    %   Uses only the raw LiDAR clearance ratio — not the waypoint bias —
    %   so the metric is not inflated artificially.
    % ------------------------------------------------------------------
    clearanceDiff = leftScore - rightScore;

    if ~isempty(targetWaypoint) && numel(targetWaypoint) >= 2
        waypointAngle = atan2(targetWaypoint(2) - robotPose(2), ...
                              targetWaypoint(1) - robotPose(1));
        relWpAngle    = wrapAngleToPi(waypointAngle - robotPose(3));
        % sin > 0: waypoint is left; sin < 0: waypoint is right.
        waypointBias  = cfg.waypointWeight * cfg.lidarMaxRange * sin(relWpAngle);
    else
        waypointBias  = 0;
    end

    combinedDiff = cfg.clearanceWeight * clearanceDiff + waypointBias;

    % Direction confidence = normalised LiDAR asymmetry (Issue 4).
    totalScore = leftScore + rightScore;
    if totalScore > 1e-6
        directionConfidence = min(1.0, abs(clearanceDiff) / totalScore);
    else
        directionConfidence = 0.0;
    end

    % Primary direction from combined score.
    if leftScore  < cfg.deadEndScoreThreshold && ...
       rightScore < cfg.deadEndScoreThreshold
        direction = 0;           % both blocked → reverse
    elseif combinedDiff >= 0
        direction = +1;          % left (or goal) more open
    else
        direction = -1;          % right more open
    end

    % Environment-type overrides.
    switch envType
        case 'DEADEND'
            direction = 0;       % always reverse in confirmed dead-end

        case 'CORNER'
            % When confidence is very low, backing up is safer than
            % committing to an arbitrary side at a corner.
            if directionConfidence < cfg.directionConfidenceMin
                direction = 0;
            end

        case 'WALL'
            % Never back into a long wall — always pick the open side.
            if direction == 0
                if leftScore >= rightScore
                    direction = +1;
                else
                    direction = -1;
                end
            end
    end

    % ------------------------------------------------------------------
    % Phase 5: Anti-oscillation via recovery memory (Issue 5)
    %
    %   Read failedAttempts from the caller (incremented in
    %   simulateRobotStep when TRY_FORWARD is blocked).
    %
    %   a) sameDirAndFailed: after cfg.oscillationThreshold consecutive
    %      failures in the same direction, flip to the other side.
    %
    %   b) lowConfAlt: when confidence is very low AND we've had exactly
    %      one failure, try the opposite side to break a near-tie deadlock.
    % ------------------------------------------------------------------
    if hasHistory && isfield(prevResult, 'failedAttempts')
        failedAttempts = prevResult.failedAttempts;
    else
        failedAttempts = 0;
    end

    if hasHistory && direction ~= 0
        sameDirAndFailed = isfield(prevResult, 'lastDirection') && ...
                           prevResult.lastDirection == direction && ...
                           failedAttempts >= cfg.oscillationThreshold;

        lowConfAlt       = directionConfidence < cfg.directionConfidenceMin && ...
                           failedAttempts == 1 && ...
                           isfield(prevResult, 'lastDirection') && ...
                           prevResult.lastDirection ~= 0;

        if sameDirAndFailed || lowConfAlt
            direction = -direction;   % flip
        end
    end

    % ------------------------------------------------------------------
    % Phase 6: Smooth adaptive angle (Issues 4 & 6)
    %
    %   Uses a smooth-step S-curve  f(t) = 3t² − 2t³  applied to the
    %   normalised absolute combined-score difference.
    %
    %   Properties:
    %     • f(0) = 0, f(1) = 1, f'(0) = 0, f'(1) = 0
    %     • No abrupt jumps at threshold boundaries
    %     • Naturally produces small corrections for small differences
    %
    %   Additional modulations:
    %     • Wall:   upper-bound capped at mediumTurnAngle (small corrections)
    %     • Scale-down when directionConfidence is very low (uncertain side)
    %     • Angle escalation: grows by angleEscalationFactor per failure
    % ------------------------------------------------------------------
    normDiff = min(1.0, abs(combinedDiff) / max(cfg.maxDiffForMaxAngle, 0.01));
    smoothed = normDiff^2 * (3 - 2 * normDiff);   % smooth-step ∈ [0,1]

    switch envType
        case 'WALL'
            % Small wall-following corrections — cap at mediumTurnAngle.
            envUpperAngle = cfg.mediumTurnAngle;
        case {'DEADEND', 'CORNER'}
            envUpperAngle = cfg.maxTurnAngle;
        otherwise
            envUpperAngle = cfg.maxTurnAngle;
    end

    angle = cfg.minTurnAngle + smoothed * (envUpperAngle - cfg.minTurnAngle);

    % Scale down when confidence is too low to trust the direction.
    if directionConfidence < cfg.directionConfidenceMin
        scaleFactor = 0.5 + 0.5 * (directionConfidence / ...
                      max(cfg.directionConfidenceMin, 1e-6));
        angle = angle * scaleFactor;
    end

    % Escalate angle on repeated failures to break out of tight spots.
    angle = angle * (1.0 + cfg.angleEscalationFactor * failedAttempts);

    % Clamp to configured bounds.
    angle = max(cfg.minTurnAngle, min(cfg.maxTurnAngle, angle));

    % ------------------------------------------------------------------
    % Assemble result (also serves as prevResult on the next call)
    % ------------------------------------------------------------------
    result.direction           = direction;
    result.angle               = angle;
    result.envType             = envType;
    result.confidence          = mapConfidence;
    result.directionConfidence = directionConfidence;
    result.leftScore           = leftScore;
    result.frontScore          = frontScore;
    result.rightScore          = rightScore;
    result.lastDirection       = direction;        % memory: direction chosen now
    result.lastEnvType         = envType;          % memory: class chosen now
    result.failedAttempts      = failedAttempts;   % caller manages increments
    result.deadEndCount        = deadEndCount;
end


% =====================================================================
%  Local helper: LiDAR sector scoring
% =====================================================================

function [leftScore, frontScore, rightScore] = computeSectorScores(scan, cfg)
%COMPUTESECTORSCORES Clearance scores for front, left, and right sectors.
%
%   score = 0.5 × median_distance + 0.5 × minimum_distance
%
%   Using both median and minimum prevents a single long ray from making
%   a cluttered sector appear falsely open.
%
%   Sector boundaries (robot frame, CCW positive):
%       Front : |angle| ≤ frontSectorHalfAngle
%       Left  :  angle  ∈ (sideInnerAngle, sideOuterAngle]
%       Right :  angle  ∈ [−sideOuterAngle, −sideInnerAngle)
%
%   Returns cfg.lidarMaxRange for empty or missing scan (treat as open).

    default = cfg.lidarMaxRange;

    if isempty(scan)
        leftScore  = default;
        frontScore = default;
        rightScore = default;
        return;
    end

    ranges = min(scan.Ranges, cfg.lidarMaxRange);   % clamp to max range
    angles = scan.Angles;                           % radians, robot frame

    frontMask = abs(angles) <= cfg.frontSectorHalfAngle;
    leftMask  = angles >  cfg.sideInnerAngle & angles <= cfg.sideOuterAngle;
    rightMask = angles < -cfg.sideInnerAngle & angles >= -cfg.sideOuterAngle;

    leftScore  = sectorScore(ranges(leftMask),  default);
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
%  Local helper: Map-based environment classification
% =====================================================================

function [envType, confidence] = classifyEnvironment(map, robotPose, cfg)
%CLASSIFYENVIRONMENT Classify the obstacle geometry in the robot's forward arc.
%
%   Issue 1 — Wall detection:
%     Uses PCA on the occupied point cloud to test:
%       • spanMajor (principal-axis length)  > wallLengthThreshold
%       • spanMajor / spanMinor              ≥ wallAspectThreshold
%       • continuity along the principal axis ≥ wallContinuityThreshold
%     This distinguishes a long continuous wall from a shorter cluster
%     that happens to have a high axis ratio but poor continuity.
%
%   Issue 2 — Object detection:
%     Uses PCA minor-axis span (structural width), not just bounding-box
%     extent.  A compact cluster is classified as OBJECT when both the
%     principal (length) and minor (width) PCA spans are small.

    envType    = 'UNKNOWN';
    confidence = 0.5;

    if isempty(map); return; end

    theta = robotPose(3);
    cosT  = cos(theta);
    sinT  = sin(theta);
    xLim  = map.XWorldLimits;
    yLim  = map.YWorldLimits;

    % Dense sample grid in robot frame (x = forward, y = lateral).
    xrVec = linspace(cfg.minProbeDistance, cfg.wallDetectionRadius, 14);
    yrVec = linspace(-cfg.wallDetectionRadius, cfg.wallDetectionRadius, 22);
    [Xr, Yr] = meshgrid(xrVec, yrVec);
    Xr = Xr(:);
    Yr = Yr(:);

    % World frame.
    Xw = robotPose(1) + cosT .* Xr - sinT .* Yr;
    Yw = robotPose(2) + sinT .* Xr + cosT .* Yr;

    % Discard out-of-bounds sample points.
    inBounds = Xw >= xLim(1) & Xw <= xLim(2) & Yw >= yLim(1) & Yw <= yLim(2);
    if sum(inBounds) < 4; return; end

    pts  = [Xw(inBounds), Yw(inBounds)];
    XrIn = Xr(inBounds);
    YrIn = Yr(inBounds);

    isOcc = logical(checkOccupancy(map, pts));

    nOcc = sum(isOcc);
    if nOcc < 4
        envType    = 'OBJECT';
        confidence = 0.45;
        return;
    end

    occXr = XrIn(isOcc);   % occupied points, forward axis (robot frame)
    occYr = YrIn(isOcc);   % occupied points, lateral axis (robot frame)

    % Sub-sector counts in robot frame.
    leftOcc  = sum(occYr >  cfg.wallSideThreshold);
    rightOcc = sum(occYr < -cfg.wallSideThreshold);
    frontOcc = sum(abs(occYr) <= cfg.wallSideThreshold);

    % ---- PCA on occupied point cloud (Issues 1 & 2) ----
    occPts  = [occXr, occYr];
    mu      = mean(occPts, 1);
    centred = occPts - mu;
    C       = (centred' * centred) / max(nOcc - 1, 1);
    [V, D]  = eig(C);

    % eig() returns eigenvalues in ascending order — sort descending.
    eigvals = diag(D);
    [~, sortIdx] = sort(eigvals, 'descend');
    V = V(:, sortIdx);

    % Project occupied points onto the two principal axes.
    projMajor = centred * V(:, 1);
    projMinor = centred * V(:, 2);

    % Span along each axis (structural extent, not variance).
    spanMajor = max(projMajor) - min(projMajor);   % structural length
    spanMinor = max(projMinor) - min(projMinor);   % structural width
    aspectPCA = spanMajor / max(spanMinor, 0.01);

    % Wall continuity: measure the largest gap between consecutive
    % projected points along the principal axis.  A large gap means the
    % "wall" is actually two separate clusters — not a continuous wall.
    sortedMajor = sort(projMajor);
    if numel(sortedMajor) > 1
        maxGap     = max(diff(sortedMajor));
        continuity = 1.0 - maxGap / max(spanMajor, 0.01);
    else
        continuity = 0.0;
    end

    % ---- Dead-end: significant occupancy on front AND both sides. ----
    if frontOcc > 1 && leftOcc > 1 && rightOcc > 1
        sideTotal = leftOcc + rightOcc;
        if sideTotal / nOcc > cfg.deadEndSideRatio
            envType    = 'DEADEND';
            confidence = min(1.0, sideTotal / nOcc);
            return;
        end
    end

    % ---- Corner: front occupied + one side clearly dominant. ----
    if frontOcc > 1 && (leftOcc > 0 || rightOcc > 0)
        dominant  = max(leftOcc, rightOcc);
        weak      = max(min(leftOcc, rightOcc), 1);
        sideRatio = dominant / weak;
        if sideRatio > 2.5 && dominant / nOcc > 0.25
            envType    = 'CORNER';
            confidence = min(1.0, sideRatio / 6.0);
            return;
        end
    end

    % ---- Wall: long AND linear AND continuous (all three required). ----
    if spanMajor   > cfg.wallLengthThreshold && ...
       aspectPCA   >= cfg.wallAspectThreshold && ...
       continuity  >= cfg.wallContinuityThreshold
        envType    = 'WALL';
        confidence = min(1.0, continuity * aspectPCA / (2.0 * cfg.wallAspectThreshold));
        return;
    end

    % ---- Object: compact cluster — both PCA axes must be small. ----
    %   (spanMajor = length along principal axis of the cluster,
    %    spanMinor = width perpendicular to it — more robust than raw
    %    bounding-box extent because it is orientation-independent.)
    if spanMajor <= cfg.objectSizeThreshold && ...
       spanMinor <= cfg.objectSizeThreshold
        envType    = 'OBJECT';
        confidence = min(1.0, 1.0 - spanMajor / max(cfg.objectSizeThreshold, 0.01));
        return;
    end

    % ---- Default: extended but unclassified cluster → treat as wall. ----
    if spanMajor > cfg.objectSizeThreshold || spanMinor > cfg.objectSizeThreshold
        envType    = 'WALL';
        confidence = 0.45;
    end
end


% =====================================================================
%  Local shared helper
% =====================================================================

function wrapped = wrapAngleToPi(angle)
    wrapped = atan2(sin(angle), cos(angle));
end
