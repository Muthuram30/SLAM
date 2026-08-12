function [slamMap, perceptionResults] = buildOccupancyMap(slamState, mapWidth, mapHeight, mapResolution, maxRange)
%BUILDOCCUPANCYMAP Build an occupancyMap from accepted SLAM scans and poses.
%
%   slamMap = BUILDOCCUPANCYMAP(slamState, mapWidth, mapHeight, ...
%       mapResolution, maxRange)
%
%   Retrieves all accepted scans and their optimized poses from the
%   lidarSLAM object, applies the SLAM->world alignment transform,
%   then projects each scan into a probabilistic occupancyMap using
%   insertRay.
%
%   Performance design:
%       Rebuilds from scratch only every MAP_REBUILD_INTERVAL accepted
%       scans.  Between rebuilds the cached map from the last full build
%       is returned immediately (O(1) cost).  This avoids the O(N^2)
%       behaviour caused by re-inserting all N scans on every simulation
%       step.
%
%   Instrumentation:
%       A diagnostic report (worldPoses bounding box vs. map limits) is
%       printed only when the out-of-bounds STATUS changes – i.e. at most
%       once at startup and once if it ever transitions.  Never on every
%       call, so the Command Window stays clean.
%
%   Inputs:
%       slamState     - struct from updateSLAM (contains .slamObj).
%       mapWidth      - map width in meters.
%       mapHeight     - map height in meters.
%       mapResolution - cells per meter for the output map.
%       maxRange      - maximum LiDAR range (meters).
%
%   Output:
%       slamMap - occupancyMap built from the SLAM data.

    % Persistent state: cache of the last successfully built map and the
    % accepted-scan count at the time it was built.  Also tracks the last
    % reported out-of-bounds status so the diagnostic fires only on change.
    persistent cachedMap lastBuiltCount lastOutOfBoundsStatus

    % Rebuild frequency: one full scansAndPoses+insertRay pass every N
    % newly accepted scans.  Increase to trade map freshness for speed.
    MAP_REBUILD_INTERVAL = 10;

    % ------------------------------------------------------------------
    % Guard: nothing to build yet.
    % ------------------------------------------------------------------
    if ~slamState.isInitialized || slamState.acceptedCount < 1
        slamMap = occupancyMap(mapWidth, mapHeight, mapResolution);
        perceptionResults = defaultPerceptionResults();
        return;
    end

    % ------------------------------------------------------------------
    % Fast path: return the cached map if fewer than MAP_REBUILD_INTERVAL
    % new scans have been accepted since the last full rebuild.
    % ------------------------------------------------------------------
    if ~isempty(cachedMap) && ~isempty(lastBuiltCount) && ...
            (slamState.acceptedCount - lastBuiltCount) < MAP_REBUILD_INTERVAL
        slamMap = cachedMap;
        perceptionResults = defaultPerceptionResults();
        return;
    end

    % ------------------------------------------------------------------
    % Full rebuild path.
    % ------------------------------------------------------------------
    try
        [scans, poses] = scansAndPoses(slamState.slamObj);

        % Step 1: Transform all SLAM-frame poses into world frame.
        if slamState.isAligned
            worldPoses = helperFunctions.transformPosesToWorld( ...
                poses, slamState.alignmentPose);
        else
            worldPoses = poses;
        end

        % Step 2: Bounding-box instrumentation.
        %   occupancyMap(W,H,R) anchors at the world origin: x in [0,W],
        %   y in [0,H].  insertRay silently clips rays outside that box.
        nominalXLim = [0, mapWidth];
        nominalYLim = [0, mapHeight];

        poseXMin = min(worldPoses(:, 1));
        poseXMax = max(worldPoses(:, 1));
        poseYMin = min(worldPoses(:, 2));
        poseYMax = max(worldPoses(:, 2));

        xOutOfBounds  = (poseXMin < nominalXLim(1)) || (poseXMax > nominalXLim(2));
        yOutOfBounds  = (poseYMin < nominalYLim(1)) || (poseYMax > nominalYLim(2));
        anyOutOfBounds = xOutOfBounds || yOutOfBounds;

        % Print only when status changes (first build, or if it flips).
        statusChanged = isempty(lastOutOfBoundsStatus) || ...
                        (lastOutOfBoundsStatus ~= anyOutOfBounds);
        if statusChanged
            fprintf('\n--- buildOccupancyMap diagnostics (%d scans) ---\n', numel(scans));
            fprintf('  Nominal map limits      : X=[%.2f, %.2f]  Y=[%.2f, %.2f] m\n', ...
                nominalXLim(1), nominalXLim(2), nominalYLim(1), nominalYLim(2));
            fprintf('  worldPoses bounding box : X=[%.2f, %.2f]  Y=[%.2f, %.2f] m\n', ...
                poseXMin, poseXMax, poseYMin, poseYMax);
            if anyOutOfBounds
                fprintf('  STATUS: OUT-OF-BOUNDS detected – map will be padded.\n');
            else
                fprintf('  STATUS: All poses within map limits – no padding.\n');
            end
            fprintf('---------------------------------------------------\n\n');
            lastOutOfBoundsStatus = anyOutOfBounds;
        end

        % Step 3: Create map (padded only when clipping is confirmed).
        if anyOutOfBounds
            margin       = maxRange;
            xMin_w       = min(poseXMin, nominalXLim(1)) - margin;
            xMax_w       = max(poseXMax, nominalXLim(2)) + margin;
            yMin_w       = min(poseYMin, nominalYLim(1)) - margin;
            yMax_w       = max(poseYMax, nominalYLim(2)) + margin;
            paddedWidth  = xMax_w - xMin_w;
            paddedHeight = yMax_w - yMin_w;
            slamMap = occupancyMap(paddedWidth, paddedHeight, mapResolution);
            slamMap.GridOriginInLocal = [xMin_w, yMin_w];
        else
            slamMap = occupancyMap(mapWidth, mapHeight, mapResolution);
        end

        % Step 4: Insert each scan's rays into the map.
        numScans = numel(scans);
        for i = 1:numScans
            try
                insertRay(slamMap, worldPoses(i, :), ...
                    scans{i}.Ranges, scans{i}.Angles, maxRange);
            catch
                continue;   % skip corrupt individual scans silently
            end
        end

        % Step 5 (NEW): Perception layer — RANSAC wall extraction,
        %   conservative map filtering, and opening detection.
        %   Only runs when perceptionConfig is present in slamState.
        %   Does NOT modify scans, poses, or any SLAM internal state.
        perceptionResults = struct( ...
            'wallSegments', zeros(0, 4), ...
            'segmentInfo', struct('numInliers', {}, 'orientation', {}, ...
                'length', {}, 'lineParams', {}, 'inlierPoints', {}), ...
            'openings', struct('center', {}, 'width', {}, 'normal', {}, ...
                'direction', {}, 'endpoint1', {}, 'endpoint2', {}, ...
                'segmentIdx', {}, 'confidence', {}, 'wallSupport', {}), ...
            'filteredCellCount', 0);

        if isfield(slamState, 'perceptionConfig') && ...
                ~isempty(slamState.perceptionConfig)
            try
                pcfg = slamState.perceptionConfig;

                % Extract wall segments from recent scans.
                [wallSegs, segInfo] = extractWallSegments( ...
                    scans, worldPoses, pcfg.ransac);

                perceptionResults.wallSegments = wallSegs;
                perceptionResults.segmentInfo  = segInfo;

                % Count occupied cells before filtering.
                occBefore = nnz(getOccupancy(slamMap) > 0.65);

                % Filter suspicious map points.
                if ~isempty(wallSegs)
                    slamMap = filterMapPoints(slamMap, wallSegs, segInfo, ...
                        worldPoses, scans, pcfg.filter);
                end

                occAfter = nnz(getOccupancy(slamMap) > 0.65);
                perceptionResults.filteredCellCount = occBefore - occAfter;

                % Detect openings from wall segments.
                if ~isempty(wallSegs) && numel(segInfo) >= 2
                    openings = detectOpenings(wallSegs, segInfo, ...
                        pcfg.robotSafetyRadius, pcfg.opening);
                    perceptionResults.openings = openings;
                end

            catch ME
                warning('buildOccupancyMap:PerceptionError', ...
                    'Perception layer failed: %s', ME.message);
            end
        end

        % Update persistent cache.
        cachedMap      = slamMap;
        lastBuiltCount = slamState.acceptedCount;

    catch ME
        warning('buildOccupancyMap:Error', ...
            'Failed to build occupancy map: %s', ME.message);
        % Return the last good map if available; otherwise a blank fallback.
        if ~isempty(cachedMap)
            slamMap = cachedMap;
        else
            slamMap = occupancyMap(mapWidth, mapHeight, mapResolution);
        end
        perceptionResults = defaultPerceptionResults();
    end
end


% =====================================================================
%  Local helper: default (empty) perception results struct
% =====================================================================

function pr = defaultPerceptionResults()
%DEFAULTPERCEPTIONRESULTS Return an empty-but-valid perceptionResults struct
%for early-return and error paths.
    pr = struct( ...
        'wallSegments', zeros(0, 4), ...
        'segmentInfo', struct('numInliers', {}, 'orientation', {}, ...
            'length', {}, 'lineParams', {}, 'inlierPoints', {}), ...
        'openings', struct('center', {}, 'width', {}, 'normal', {}, ...
            'direction', {}, 'endpoint1', {}, 'endpoint2', {}, ...
            'segmentIdx', {}, 'confidence', {}, 'wallSupport', {}), ...
        'filteredCellCount', 0);
end
