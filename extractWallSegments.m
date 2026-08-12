function [wallSegments, segmentInfo] = extractWallSegments(scans, worldPoses, config)
%EXTRACTWALLSEGMENTS 2D multi-line RANSAC wall extraction from LiDAR scans.
%
%   [wallSegments, segmentInfo] = EXTRACTWALLSEGMENTS(scans, worldPoses, config)
%
%   Aggregates scan points from the most recent N scans, transforms them
%   into world-frame Cartesian coordinates, then iteratively extracts
%   dominant line segments using 2D RANSAC.
%
%   The function extracts LINE SEGMENTS (finite endpoints) rather than
%   infinite lines. Each segment's endpoints are computed by projecting
%   inliers onto the fitted line direction and taking the min/max extent.
%
%   Inputs:
%       scans      - cell array of lidarScan objects from scansAndPoses().
%       worldPoses - Nx3 matrix of [x y theta] world-frame poses aligned
%                    with scans{}.
%       config     - struct with RANSAC configuration fields:
%           .distanceThreshold  (m)  max point-to-line distance for inlier
%           .maxIterations      RANSAC iterations per line
%           .minInliers         min inliers to accept a line
%           .minSegmentLength   (m)  min segment length to accept
%           .maxSegments        max number of segments to extract
%           .windowSize         number of most-recent scans to aggregate
%           .maxRange           (m)  reject points at max range (no-hit)
%           .minRange           (m)  reject points below min range
%
%   Outputs:
%       wallSegments - Mx4 matrix [x1 y1 x2 y2] per row, one per segment.
%                      Empty (0x4) if no segments found.
%       segmentInfo  - Mx1 struct array with per-segment metadata:
%           .numInliers    - number of inlier points
%           .orientation   - segment angle in radians (atan2 of direction)
%           .length        - segment length in meters
%           .lineParams    - [a b c] for the line  ax + by + c = 0
%           .inlierPoints  - Kx2 matrix of inlier world-frame points

    % ------------------------------------------------------------------
    % Default config
    % ------------------------------------------------------------------
    defaults = struct( ...
        'distanceThreshold', 0.05, ...
        'maxIterations',     200, ...
        'minInliers',        8, ...
        'minSegmentLength',  0.5, ...
        'maxSegments',       15, ...
        'windowSize',        10, ...
        'maxRange',          8.0, ...
        'minRange',          0.12);

    if nargin < 3 || isempty(config)
        config = defaults;
    else
        fnames = fieldnames(defaults);
        for k = 1:numel(fnames)
            if ~isfield(config, fnames{k})
                config.(fnames{k}) = defaults.(fnames{k});
            end
        end
    end

    wallSegments = zeros(0, 4);
    segmentInfo  = struct('numInliers', {}, 'orientation', {}, ...
        'length', {}, 'lineParams', {}, 'inlierPoints', {});

    % ------------------------------------------------------------------
    % Guard: empty inputs
    % ------------------------------------------------------------------
    if isempty(scans) || isempty(worldPoses)
        return;
    end

    numScans = numel(scans);
    numPoses = size(worldPoses, 1);
    if numScans ~= numPoses
        warning('extractWallSegments:SizeMismatch', ...
            'scans (%d) and worldPoses (%d) must have the same count.', ...
            numScans, numPoses);
        numScans = min(numScans, numPoses);
    end

    % ------------------------------------------------------------------
    % Step 1: Aggregate recent scans into a single world-frame point cloud
    % ------------------------------------------------------------------
    startIdx = max(1, numScans - config.windowSize + 1);
    allPoints = [];

    for i = startIdx:numScans
        if isempty(scans{i})
            continue;
        end

        ranges = scans{i}.Ranges;
        angles = scans{i}.Angles;
        pose   = worldPoses(i, :);

        % Filter: reject max-range and below-min-range readings.
        validMask = ranges > config.minRange & ...
                    ranges < (config.maxRange - 0.01);

        if ~any(validMask)
            continue;
        end

        validRanges = ranges(validMask);
        validAngles = angles(validMask);

        % Transform to world frame: body → world rotation + translation.
        worldAngles = validAngles + pose(3);
        xWorld = pose(1) + validRanges .* cos(worldAngles);
        yWorld = pose(2) + validRanges .* sin(worldAngles);

        allPoints = [allPoints; xWorld(:), yWorld(:)]; %#ok<AGROW>
    end

    if size(allPoints, 1) < config.minInliers
        return;
    end

    % ------------------------------------------------------------------
    % Step 2: Iterative multi-line RANSAC extraction
    % ------------------------------------------------------------------
    remainingPoints = allPoints;
    segCount = 0;

    while segCount < config.maxSegments && ...
          size(remainingPoints, 1) >= config.minInliers

        % --- Run single-line RANSAC on remaining points ---
        [bestLine, bestInlierMask, bestInlierCount] = ...
            ransacFitLine2D(remainingPoints, ...
                config.distanceThreshold, config.maxIterations);

        % Stop if the best line has too few inliers.
        if bestInlierCount < config.minInliers
            break;
        end

        % --- Extract segment endpoints from inlier projection ---
        inlierPts = remainingPoints(bestInlierMask, :);
        [segEndpoints, segLength, segOrientation] = ...
            extractSegmentFromInliers(inlierPts, bestLine);

        % Reject segments shorter than minSegmentLength.
        if segLength < config.minSegmentLength
            % Remove these inliers but don't record the segment.
            remainingPoints = remainingPoints(~bestInlierMask, :);
            continue;
        end

        % --- Record the segment ---
        segCount = segCount + 1;
        wallSegments(segCount, :) = segEndpoints;

        info.numInliers   = bestInlierCount;
        info.orientation  = segOrientation;
        info.length       = segLength;
        info.lineParams   = bestLine;
        info.inlierPoints = inlierPts;
        segmentInfo(segCount) = info;

        % Remove inliers from the remaining pool.
        remainingPoints = remainingPoints(~bestInlierMask, :);
    end
end


% =====================================================================
%  Local helper: 2D line RANSAC
% =====================================================================

function [bestLine, bestInlierMask, bestInlierCount] = ...
        ransacFitLine2D(points, distThreshold, maxIter)
%RANSACFITLINE2D Fit a single 2D line (ax + by + c = 0) using RANSAC.
%
%   Randomly samples 2 points per iteration, fits a line through them,
%   counts inliers within distThreshold, and keeps the best line.
%
%   Returns:
%       bestLine       - 1x3 [a b c] coefficients of the line.
%       bestInlierMask - Nx1 logical, true for inlier points.
%       bestInlierCount - number of inliers.

    N = size(points, 1);
    bestInlierCount = 0;
    bestInlierMask  = false(N, 1);
    bestLine        = [0 0 0];

    px = points(:, 1);
    py = points(:, 2);

    for iter = 1:maxIter
        % Pick 2 random distinct points.
        idx = randperm(N, 2);
        p1 = points(idx(1), :);
        p2 = points(idx(2), :);

        % Degenerate check: points too close.
        dx = p2(1) - p1(1);
        dy = p2(2) - p1(2);
        segLen = hypot(dx, dy);
        if segLen < 1e-6
            continue;
        end

        % Line equation: ax + by + c = 0
        %   normal = [-dy, dx] (perpendicular to the direction)
        a = -dy / segLen;
        b =  dx / segLen;
        c = -(a * p1(1) + b * p1(2));

        % Distance from every point to this line.
        dists = abs(a * px + b * py + c);

        % Count inliers.
        inlierMask = dists <= distThreshold;
        inlierCount = sum(inlierMask);

        if inlierCount > bestInlierCount
            bestInlierCount = inlierCount;
            bestInlierMask  = inlierMask;
            bestLine        = [a, b, c];
        end
    end
end


% =====================================================================
%  Local helper: extract finite segment from inliers
% =====================================================================

function [endpoints, segLength, orientation] = ...
        extractSegmentFromInliers(inlierPts, lineParams)
%EXTRACTSEGMENTFROMINLIERS Project inliers onto the line direction and
%compute the segment endpoints as the min/max projections.
%
%   lineParams = [a b c] for line ax + by + c = 0
%   direction along the line = [b, -a] (perpendicular to normal [a, b])
%
%   Returns:
%       endpoints   - 1x4 [x1 y1 x2 y2]
%       segLength   - scalar, Euclidean length of segment
%       orientation - angle of segment direction (radians)

    a = lineParams(1);
    b = lineParams(2);

    % Line direction vector (unit length, since [a,b] is normalised).
    dirVec = [b, -a];

    % Project inlier points onto line direction.
    % Use the centroid as origin for numerical stability.
    centroid = mean(inlierPts, 1);
    centered = inlierPts - centroid;
    projections = centered * dirVec';   % Nx1 scalar projections

    [minProj, ~] = min(projections);
    [maxProj, ~] = max(projections);

    % Reconstruct world-frame endpoints.
    p1 = centroid + minProj * dirVec;
    p2 = centroid + maxProj * dirVec;

    endpoints = [p1, p2];
    segLength = maxProj - minProj;
    orientation = atan2(dirVec(2), dirVec(1));
end
