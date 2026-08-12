function openings = detectOpenings(wallSegments, segmentInfo, ...
        robotSafetyRadius, config)
%DETECTOPENINGS Detect traversable openings between collinear wall segments.
%
%   openings = DETECTOPENINGS(wallSegments, segmentInfo, robotSafetyRadius, config)
%
%   Examines all pairs of detected wall segments for approximate
%   collinearity. When two segments are collinear (similar orientation,
%   small perpendicular offset) with a meaningful gap between their
%   endpoints, the gap is classified as a candidate opening if it meets
%   geometric width and wall-support criteria.
%
%   This function ONLY REPORTS candidate openings. It does NOT modify
%   waypoints, navigation logic, or any planner state.
%
%   Inputs:
%       wallSegments     - Mx4 matrix [x1 y1 x2 y2] from extractWallSegments.
%       segmentInfo      - Mx1 struct array with .numInliers, .length.
%       robotSafetyRadius - robot safety radius (m) from createRobot.
%       config           - struct with opening detection parameters:
%           .collinearAngleThreshold   (rad) max angle diff for collinearity
%           .collinearOffsetThreshold  (m)   max perpendicular offset
%           .minOpeningWidth           (m)   min gap width
%           .maxOpeningWidth           (m)   max gap width
%           .minWallSupport            inliers per side
%           .minWallLength             (m)   per side
%
%   Output:
%       openings - Kx1 struct array (empty if none found) with fields:
%           .center         1x2 [x y] world-frame center of opening
%           .width          gap width in meters
%           .normal         1x2 unit vector perpendicular to wall direction
%           .direction      1x2 unit vector along wall direction
%           .endpoint1      1x2 [x y] inner endpoint of segment 1
%           .endpoint2      1x2 [x y] inner endpoint of segment 2
%           .segmentIdx     1x2 [i j] indices into wallSegments
%           .confidence     scalar 0–1
%           .wallSupport    1x2 [inliers_i, inliers_j]

    % ------------------------------------------------------------------
    % Default config
    % ------------------------------------------------------------------
    defaults = struct( ...
        'collinearAngleThreshold',  deg2rad(15), ...
        'collinearOffsetThreshold', 0.3, ...
        'minOpeningWidth',          0.55, ...
        'maxOpeningWidth',          3.0, ...
        'minWallSupport',           8, ...
        'minWallLength',            0.8);

    if nargin < 4 || isempty(config)
        config = defaults;
    else
        fnames = fieldnames(defaults);
        for k = 1:numel(fnames)
            if ~isfield(config, fnames{k})
                config.(fnames{k}) = defaults.(fnames{k});
            end
        end
    end

    openings = struct('center', {}, 'width', {}, 'normal', {}, ...
        'direction', {}, 'endpoint1', {}, 'endpoint2', {}, ...
        'segmentIdx', {}, 'confidence', {}, 'wallSupport', {});

    numSegments = size(wallSegments, 1);
    if numSegments < 2
        return;   % need at least 2 segments for a gap
    end

    % Apply minimum width from safety radius if not already set.
    minWidth = max(config.minOpeningWidth, 2.5 * robotSafetyRadius);

    % ------------------------------------------------------------------
    % Examine all pairs of segments for collinearity + gap
    % ------------------------------------------------------------------
    openingCount = 0;

    for i = 1:(numSegments - 1)
        for j = (i + 1):numSegments

            % --- Check 1: Approximate collinearity (orientation) ---
            ori_i = segmentInfo(i).orientation;
            ori_j = segmentInfo(j).orientation;

            % Angle difference, accounting for ±180° ambiguity in
            % segment direction.
            angleDiff = abs(ori_i - ori_j);
            angleDiff = min(angleDiff, pi - angleDiff);

            if angleDiff > config.collinearAngleThreshold
                continue;
            end

            % --- Check 2: Perpendicular offset ---
            % Use segment i's line as reference; measure distance
            % from segment j's midpoint to segment i's line.
            midJ = 0.5 * (wallSegments(j, 1:2) + wallSegments(j, 3:4));
            perpOffset = pointToLineDistance(midJ, ...
                wallSegments(i, 1:2), wallSegments(i, 3:4));

            if perpOffset > config.collinearOffsetThreshold
                continue;
            end

            % --- Check 3: Wall support on both sides ---
            if segmentInfo(i).numInliers < config.minWallSupport || ...
               segmentInfo(j).numInliers < config.minWallSupport
                continue;
            end

            % --- Check 4: Wall length on both sides ---
            if segmentInfo(i).length < config.minWallLength || ...
               segmentInfo(j).length < config.minWallLength
                continue;
            end

            % --- Compute the gap ---
            % Shared direction: average of the two segment orientations.
            avgOri = 0.5 * (ori_i + ori_j);
            % Handle wrap-around for nearly opposite directions.
            if abs(ori_i - ori_j) > pi/2
                avgOri = avgOri + pi/2;
            end
            dirVec = [cos(avgOri), sin(avgOri)];

            % Project all 4 endpoints onto the shared direction.
            p1a = wallSegments(i, 1:2);
            p1b = wallSegments(i, 3:4);
            p2a = wallSegments(j, 1:2);
            p2b = wallSegments(j, 3:4);

            % Use segment i's start as projection origin.
            proj1a = dot(p1a - p1a, dirVec);  % = 0
            proj1b = dot(p1b - p1a, dirVec);
            proj2a = dot(p2a - p1a, dirVec);
            proj2b = dot(p2b - p1a, dirVec);

            % Determine segment intervals on the shared axis.
            seg1_min = min(proj1a, proj1b);
            seg1_max = max(proj1a, proj1b);
            seg2_min = min(proj2a, proj2b);
            seg2_max = max(proj2a, proj2b);

            % The gap is between the nearest inner endpoints.
            % Case 1: seg1 is "left" of seg2 → gap = [seg1_max, seg2_min]
            % Case 2: seg2 is "left" of seg1 → gap = [seg2_max, seg1_min]
            if seg1_max <= seg2_min
                gapStart = seg1_max;
                gapEnd   = seg2_min;
                % Inner endpoint from seg1: whichever of p1a/p1b has larger proj
                if proj1b > proj1a
                    innerPt1 = p1b;
                else
                    innerPt1 = p1a;
                end
                % Inner endpoint from seg2: whichever of p2a/p2b has smaller proj
                if proj2a < proj2b
                    innerPt2 = p2a;
                else
                    innerPt2 = p2b;
                end
            elseif seg2_max <= seg1_min
                gapStart = seg2_max;
                gapEnd   = seg1_min;
                if proj2b > proj2a
                    innerPt1 = p2b;
                else
                    innerPt1 = p2a;
                end
                if proj1a < proj1b
                    innerPt2 = p1a;
                else
                    innerPt2 = p1b;
                end
            else
                % Segments overlap on the shared axis → no gap.
                continue;
            end

            gapWidth = gapEnd - gapStart;

            % --- Check 5: Gap width bounds ---
            if gapWidth < minWidth || gapWidth > config.maxOpeningWidth
                continue;
            end

            % --- Compute opening geometry ---
            openingCenter = 0.5 * (innerPt1 + innerPt2);
            openingNormal = [-dirVec(2), dirVec(1)];  % perpendicular to wall

            % Confidence: based on wall support and gap quality.
            supportFactor = min(1.0, min(segmentInfo(i).numInliers, ...
                segmentInfo(j).numInliers) / 20);
            lengthFactor = min(1.0, min(segmentInfo(i).length, ...
                segmentInfo(j).length) / 2.0);
            widthFactor = 1.0 - abs(gapWidth - 1.5) / config.maxOpeningWidth;
            widthFactor = max(0, min(1, widthFactor));
            confidence = 0.4 * supportFactor + 0.3 * lengthFactor + ...
                         0.3 * widthFactor;

            % --- Record the opening ---
            openingCount = openingCount + 1;
            op.center      = openingCenter;
            op.width       = gapWidth;
            op.normal      = openingNormal;
            op.direction   = dirVec;
            op.endpoint1   = innerPt1;
            op.endpoint2   = innerPt2;
            op.segmentIdx  = [i, j];
            op.confidence  = confidence;
            op.wallSupport = [segmentInfo(i).numInliers, ...
                              segmentInfo(j).numInliers];
            openings(openingCount) = op;
        end
    end
end


% =====================================================================
%  Local helper: point-to-line distance (infinite line through 2 points)
% =====================================================================

function d = pointToLineDistance(pt, lineP1, lineP2)
%POINTTOLINEDISTANCE Perpendicular distance from a point to the infinite
%line passing through lineP1 and lineP2.

    v = lineP2 - lineP1;
    len = norm(v);
    if len < 1e-12
        d = norm(pt - lineP1);
        return;
    end

    % Cross product magnitude / length = perpendicular distance.
    d = abs((pt(1) - lineP1(1)) * v(2) - (pt(2) - lineP1(2)) * v(1)) / len;
end
