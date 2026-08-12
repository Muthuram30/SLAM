function cleanedMap = filterMapPoints(slamMap, wallSegments, segmentInfo, ...
        worldPoses, scans, config)
%FILTERMAPPOINTS Conservative multi-layer map cleaning.
%
%   cleanedMap = FILTERMAPPOINTS(slamMap, wallSegments, segmentInfo, ...
%       worldPoses, scans, config)
%
%   Applies a 4-layer suspicion scoring system to identify and suppress
%   occupied cells that are geometrically inconsistent with the observed
%   wall structure. Each layer independently adds evidence; only cells
%   exceeding a combined suspicion threshold are suppressed.
%
%   Filtering hierarchy (decreasing evidence strength):
%     1. Ray/visibility consistency — cell behind a confirmed wall along
%        a measured ray → +2 suspicion.
%     2. RANSAC wall inconsistency — cell near a wall but not an inlier
%        → +1 suspicion.
%     3. Temporal consistency — cell observed fewer than N times → +1.
%     4. Local density/outlier — cell has too few occupied neighbours
%        in a 5×5 window → +1 suspicion.
%
%   Preservation rules (never suppress):
%     - RANSAC inlier cells.
%     - Cells within cornerPreservationRadius of any segment endpoint.
%     - Cells belonging to segments with strong inlier support.
%
%   Inputs:
%       slamMap      - occupancyMap built by buildOccupancyMap.
%       wallSegments - Mx4 matrix [x1 y1 x2 y2] from extractWallSegments.
%       segmentInfo  - Mx1 struct array with .numInliers, .inlierPoints.
%       worldPoses   - Nx3 poses from scansAndPoses (world frame).
%       scans        - cell array of lidarScan objects.
%       config       - struct with filter configuration fields:
%           .behindWallTolerance      (m)
%           .wallProximityThreshold   (m)
%           .minOccupiedObservations  count
%           .minNeighbourOccupied     count
%           .suspicionThreshold       combined score to suppress
%           .cornerPreservationRadius (m)
%           .preservationInlierCount  count
%
%   Output:
%       cleanedMap - occupancyMap with suspicious cells set to 0.5.

    % ------------------------------------------------------------------
    % Default config
    % ------------------------------------------------------------------
    defaults = struct( ...
        'behindWallTolerance',      0.10, ...
        'wallProximityThreshold',   0.15, ...
        'minOccupiedObservations',  2, ...
        'minNeighbourOccupied',     2, ...
        'suspicionThreshold',       3, ...
        'cornerPreservationRadius', 0.3, ...
        'preservationInlierCount',  6);

    if nargin < 6 || isempty(config)
        config = defaults;
    else
        fnames = fieldnames(defaults);
        for k = 1:numel(fnames)
            if ~isfield(config, fnames{k})
                config.(fnames{k}) = defaults.(fnames{k});
            end
        end
    end

    % ------------------------------------------------------------------
    % Persistent temporal consistency counter — tracks how many times
    % each cell has been marked occupied across rebuilds.
    % ------------------------------------------------------------------
    persistent occupiedCounter counterRows counterCols counterOrigin

    % ------------------------------------------------------------------
    % Prepare the occupancy grid
    % ------------------------------------------------------------------
    cleanedMap = slamMap;   % copy; we modify in place

    occGrid = getOccupancy(slamMap);
    [numRows, numCols] = size(occGrid);
    resolution = slamMap.Resolution;
    xLimits = slamMap.XWorldLimits;   % [xMin, xMax]
    yLimits = slamMap.YWorldLimits;   % [yMin, yMax]
    gridOriginX = xLimits(1);
    gridOriginY = yLimits(1);

    occupiedThreshold = 0.65;   % cells with probability > this are "occupied"
    occupiedMask = occGrid > occupiedThreshold;

    % Find all occupied cell indices.
    [occRows, occCols] = find(occupiedMask);
    numOccupied = numel(occRows);

    if numOccupied == 0
        return;   % nothing to filter
    end

    % Convert occupied cell indices to world coordinates.
    % occupancyMap convention: row 1 = top (max y), column 1 = left (min x).
    occWorldX = gridOriginX + (occCols - 0.5) / resolution;
    occWorldY = gridOriginY + (numRows - occRows + 0.5) / resolution;

    % ------------------------------------------------------------------
    % Suspicion score: one per occupied cell, starts at 0
    % ------------------------------------------------------------------
    suspicionScore = zeros(numOccupied, 1);

    % ------------------------------------------------------------------
    % Preservation mask: cells that must NEVER be suppressed
    % ------------------------------------------------------------------
    preservedMask = false(numOccupied, 1);

    numSegments = size(wallSegments, 1);

    % ------------------------------------------------------------------
    % Mark RANSAC inlier cells as preserved
    % ------------------------------------------------------------------
    if numSegments > 0 && ~isempty(segmentInfo)
        for s = 1:numSegments
            if segmentInfo(s).numInliers >= config.preservationInlierCount
                inlierPts = segmentInfo(s).inlierPoints;
                % For each occupied cell, check if any inlier falls
                % within one cell of it.
                cellTol = 1.5 / resolution;  % ~1.5 cells in meters
                for j = 1:size(inlierPts, 1)
                    distToOcc = hypot(occWorldX - inlierPts(j, 1), ...
                                     occWorldY - inlierPts(j, 2));
                    preservedMask(distToOcc < cellTol) = true;
                end
            end
        end

        % Preserve cells near segment endpoints (corners, doorway edges).
        for s = 1:numSegments
            ep1 = wallSegments(s, 1:2);
            ep2 = wallSegments(s, 3:4);
            dist1 = hypot(occWorldX - ep1(1), occWorldY - ep1(2));
            dist2 = hypot(occWorldX - ep2(1), occWorldY - ep2(2));
            preservedMask(dist1 < config.cornerPreservationRadius) = true;
            preservedMask(dist2 < config.cornerPreservationRadius) = true;
        end
    end

    % ------------------------------------------------------------------
    % Layer 1: Ray/visibility consistency (+2 suspicion)
    %
    %   For a subset of recent poses, march along each ray. If the ray
    %   crosses a confirmed wall segment BEFORE reaching a given occupied
    %   cell, that cell is "behind a wall" and is suspicious.
    %
    %   For performance, we only check the last few scans and sample rays.
    % ------------------------------------------------------------------
    if numSegments > 0 && ~isempty(scans) && ~isempty(worldPoses)
        numPoses = size(worldPoses, 1);
        checkCount = min(5, numPoses);   % check last 5 poses max
        raySubsample = 4;  % check every 4th ray for performance

        for pi = (numPoses - checkCount + 1):numPoses
            if pi < 1 || isempty(scans{pi})
                continue;
            end

            pose   = worldPoses(pi, :);
            ranges = scans{pi}.Ranges;
            angles = scans{pi}.Angles;

            for ri = 1:raySubsample:numel(ranges)
                range_i = ranges(ri);
                if range_i <= 0.12 || range_i >= 7.99
                    continue;  % skip invalid / max-range rays
                end

                worldAngle = angles(ri) + pose(3);
                rayDirX = cos(worldAngle);
                rayDirY = sin(worldAngle);

                % Find closest wall segment intersection along this ray.
                minWallDist = inf;
                for s = 1:numSegments
                    wallDist = raySegmentIntersection( ...
                        pose(1:2), [rayDirX, rayDirY], ...
                        wallSegments(s, 1:2), wallSegments(s, 3:4));
                    if wallDist > 0 && wallDist < minWallDist
                        minWallDist = wallDist;
                    end
                end

                if isinf(minWallDist)
                    continue;  % no wall crossing on this ray
                end

                % Check occupied cells: if their distance along this ray
                % from the pose is GREATER than wallDist + tolerance,
                % they are behind the wall.
                dx = occWorldX - pose(1);
                dy = occWorldY - pose(2);
                % Project onto ray direction.
                projDist = dx * rayDirX + dy * rayDirY;
                % Perpendicular distance from ray.
                perpDist = abs(-dx * rayDirY + dy * rayDirX);

                % Only consider cells close to this ray (within ~2 cells).
                rayWidth = 2.0 / resolution;
                nearRay = perpDist < rayWidth & projDist > 0;

                behindWall = nearRay & ...
                    projDist > (minWallDist + config.behindWallTolerance);

                suspicionScore(behindWall) = ...
                    suspicionScore(behindWall) + 2;
            end
        end
    end

    % ------------------------------------------------------------------
    % Layer 2: RANSAC wall inconsistency (+1 suspicion)
    %
    %   Cells near a wall segment but NOT an inlier of that segment
    %   are slightly suspicious — they may be ghost reflections or
    %   through-wall artefacts.
    % ------------------------------------------------------------------
    if numSegments > 0
        for ci = 1:numOccupied
            if preservedMask(ci)
                continue;   % skip preserved cells
            end

            pt = [occWorldX(ci), occWorldY(ci)];
            minDistToWall = inf;

            for s = 1:numSegments
                d = pointToSegmentDistance(pt, ...
                    wallSegments(s, 1:2), wallSegments(s, 3:4));
                if d < minDistToWall
                    minDistToWall = d;
                end
            end

            if minDistToWall < config.wallProximityThreshold
                % Near a wall but not preserved as inlier → suspicious.
                suspicionScore(ci) = suspicionScore(ci) + 1;
            end
        end
    end

    % ------------------------------------------------------------------
    % Layer 3: Temporal consistency (+1 suspicion)
    %
    %   Maintain a persistent counter of how many times each cell has
    %   been observed as occupied. Cells with low observation count
    %   are transient artefacts.
    % ------------------------------------------------------------------
    needReset = isempty(occupiedCounter) || ...
                ~isequal([counterRows, counterCols], [numRows, numCols]) || ...
                ~isequal(counterOrigin, [gridOriginX, gridOriginY]);

    if needReset
        occupiedCounter = zeros(numRows, numCols);
        counterRows     = numRows;
        counterCols     = numCols;
        counterOrigin   = [gridOriginX, gridOriginY];
    end

    % Increment counter for currently occupied cells.
    for ci = 1:numOccupied
        occupiedCounter(occRows(ci), occCols(ci)) = ...
            occupiedCounter(occRows(ci), occCols(ci)) + 1;
    end

    % Flag cells with low observation count.
    for ci = 1:numOccupied
        if occupiedCounter(occRows(ci), occCols(ci)) < ...
                config.minOccupiedObservations
            suspicionScore(ci) = suspicionScore(ci) + 1;
        end
    end

    % ------------------------------------------------------------------
    % Layer 4: Local density / outlier check (+1 suspicion)
    %
    %   Count occupied neighbours in a 5×5 window around each cell.
    %   Isolated cells with too few neighbours are likely noise.
    % ------------------------------------------------------------------
    halfWin = 2;  % 5x5 window
    for ci = 1:numOccupied
        r = occRows(ci);
        c = occCols(ci);

        rMin = max(1, r - halfWin);
        rMax = min(numRows, r + halfWin);
        cMin = max(1, c - halfWin);
        cMax = min(numCols, c + halfWin);

        neighbourhood = occupiedMask(rMin:rMax, cMin:cMax);
        % Subtract 1 for the cell itself.
        neighbourCount = sum(neighbourhood(:)) - 1;

        if neighbourCount < config.minNeighbourOccupied
            suspicionScore(ci) = suspicionScore(ci) + 1;
        end
    end

    % ------------------------------------------------------------------
    % Decision: suppress cells exceeding the suspicion threshold
    % ------------------------------------------------------------------
    suppressMask = suspicionScore >= config.suspicionThreshold & ...
                   ~preservedMask;

    numSuppressed = sum(suppressMask);

    if numSuppressed > 0
        for ci = 1:numOccupied
            if suppressMask(ci)
                % Set to 0.5 (unknown/free-ish) — not hard-zero, so
                % future observations can re-establish the cell.
                setOccupancy(cleanedMap, ...
                    [occWorldX(ci), occWorldY(ci)], 0.5);
            end
        end
    end
end


% =====================================================================
%  Local helper: ray–segment intersection distance
% =====================================================================

function t = raySegmentIntersection(rayOrigin, rayDir, segP1, segP2)
%RAYSEGMENTINTERSECTION Compute distance along ray to its intersection
%with a finite line segment. Returns inf if no intersection.
%
%   rayOrigin - 1x2 [x y]
%   rayDir    - 1x2 [dx dy] (not necessarily unit, but assumed ~unit)
%   segP1     - 1x2 [x y] segment start
%   segP2     - 1x2 [x y] segment end

    t = inf;

    % Ray: P = rayOrigin + t * rayDir,  t >= 0
    % Segment: Q = segP1 + s * (segP2 - segP1),  s in [0, 1]
    %
    % Solve the 2x2 system for (t, s).

    d = segP2 - segP1;
    denom = rayDir(1) * d(2) - rayDir(2) * d(1);

    if abs(denom) < 1e-10
        return;   % parallel or degenerate
    end

    diff = segP1 - rayOrigin;
    tVal = (diff(1) * d(2) - diff(2) * d(1)) / denom;
    sVal = (diff(1) * rayDir(2) - diff(2) * rayDir(1)) / denom;

    if tVal >= 0 && sVal >= 0 && sVal <= 1
        t = tVal;
    end
end


% =====================================================================
%  Local helper: point-to-segment distance
% =====================================================================

function d = pointToSegmentDistance(pt, segP1, segP2)
%POINTTOSEGMENTDISTANCE Minimum Euclidean distance from a point to a
%finite line segment.

    v = segP2 - segP1;
    w = pt - segP1;
    c1 = dot(w, v);
    c2 = dot(v, v);

    if c2 < 1e-12
        d = norm(pt - segP1);
        return;
    end

    b = c1 / c2;
    b = max(0, min(1, b));   % clamp to [0, 1]

    closest = segP1 + b * v;
    d = norm(pt - closest);
end
