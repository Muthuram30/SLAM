function [waypoints, figHandle] = generateWaypoints(map, startPosition, varargin)
%GENERATEWAYPOINTS Generate systematic exploration waypoints using a
%wavefront (distance-transform) coverage path over the occupancy map.
%
%   waypoints = GENERATEWAYPOINTS(map, startPosition) samples a regular
%   grid of candidate points, discards occupied/too-close candidates, and
%   orders them using a wavefront planner: a BFS distance transform from
%   the start position assigns each free cell a "wave distance", then
%   candidates are sorted by wave distance with boustrophedon (alternating
%   row-sweep) ordering within each distance band. This produces a
%   systematic room-by-room sweep without zigzagging or revisits.
%
%   [waypoints, figHandle] = GENERATEWAYPOINTS(..., 'Visualize', true)
%   also plots the waypoints and the resulting path over the map.
%
%   Name-value parameters (defaults shown):
%       'Spacing'    1.2   - grid sampling spacing, meters
%       'Clearance'  0.35  - min clearance from obstacles, meters
%       'Visualize'  false - plot the waypoints
%
%   Inputs:
%       map           - binaryOccupancyMap describing the environment.
%       startPosition - 1x2 [x y], the robot's starting position.
%
%   Outputs:
%       waypoints - Nx2 matrix of [x y] forming a continuous coverage path.
%       figHandle - visualization figure handle, or [].

    % ---------------------------------------------------------------
    % Parse and validate inputs
    % ---------------------------------------------------------------
    parser = inputParser;
    addRequired(parser, 'map');
    addRequired(parser, 'startPosition', @(p) validateattributes(p, ...
        {'double'}, {'vector', 'numel', 2, 'finite'}));
    addParameter(parser, 'Spacing', 1.2, @(v) validateattributes(v, ...
        {'double'}, {'scalar', 'positive', 'finite'}));
    addParameter(parser, 'Clearance', 0.35, @(v) validateattributes(v, ...
        {'double'}, {'scalar', 'positive', 'finite'}));
    addParameter(parser, 'Visualize', false, @(v) validateattributes(v, ...
        {'logical'}, {'scalar'}));

    parse(parser, map, startPosition, varargin{:});
    opts = parser.Results;
    startPosition = reshape(double(opts.startPosition), 1, 2);

    figHandle = [];

    % ---------------------------------------------------------------
    % Build a regular grid of candidate sample points across the map.
    % ---------------------------------------------------------------
    xLimits = map.XWorldLimits;
    yLimits = map.YWorldLimits;

    % Build candidate grid with a generous boundary inset.
    % Using max(Spacing*0.5, 1.0) keeps points at least 1 m from
    % the map edge, so no waypoint is close enough to a wall for the
    % robot's safety ring to overhang the boundary.
    boundaryMargin = max(opts.Spacing * 0.5, 1.0);
    xSamples = (xLimits(1) + boundaryMargin):opts.Spacing:(xLimits(2) - boundaryMargin);
    ySamples = (yLimits(1) + boundaryMargin):opts.Spacing:(yLimits(2) - boundaryMargin);


    if isempty(xSamples) || isempty(ySamples)
        warning('generateWaypoints:EmptyGrid', ...
            'Spacing (%.2f m) is too large for the map extent; no candidates generated.', ...
            opts.Spacing);
        waypoints = zeros(0, 2);
        return;
    end

    [xGrid, yGrid] = meshgrid(xSamples, ySamples);
    candidatePoints = [xGrid(:), yGrid(:)];
    numCandidates = size(candidatePoints, 1);

    % ---------------------------------------------------------------
    % Keep only candidates that are free and have adequate clearance.
    % ---------------------------------------------------------------
    numClearanceSamples = 6;
    clearanceAngles = linspace(0, 2*pi, numClearanceSamples + 1);
    clearanceAngles(end) = [];

    keepMask = false(numCandidates, 1);

    for idx = 1:numCandidates
        point = candidatePoints(idx, :);

        ringPoints = point + opts.Clearance * [cos(clearanceAngles)', sin(clearanceAngles)'];
        allPoints = [point; ringPoints];

        % Discard ring points that fall outside the map.
        inBounds = allPoints(:,1) >= xLimits(1) & allPoints(:,1) <= xLimits(2) & ...
                   allPoints(:,2) >= yLimits(1) & allPoints(:,2) <= yLimits(2);
        testPoints = allPoints(inBounds, :);

        if isempty(testPoints)
            continue;
        end

        occupiedFlags = checkOccupancy(map, testPoints);
        keepMask(idx) = ~any(occupiedFlags);
    end

    freeCandidates = candidatePoints(keepMask, :);

    if isempty(freeCandidates)
        warning('generateWaypoints:NoFreeCandidates', ...
            'No candidate waypoints survived the clearance filter (Clearance = %.2f m).', ...
            opts.Clearance);
        waypoints = zeros(0, 2);
        return;
    end

    % ---------------------------------------------------------------
    % Wavefront coverage-path ordering
    %
    % 1. Compute a BFS distance transform on the occupancy grid from the
    %    start position. Every free cell gets a "wave distance" (in cells).
    % 2. Look up the wave distance for each candidate waypoint.
    % 3. Assign each candidate to a distance band (band width = Spacing
    %    in cells). Within each band, sort candidates in boustrophedon
    %    order (alternating left-right sweep by Y row) so the path
    %    systematically sweeps each region before moving outward.
    % 4. Concatenate bands in order to produce the final waypoint list.
    % ---------------------------------------------------------------
    resolution = map.Resolution;
    occupancyGrid = getOccupancy(map);
    [numRows, numCols] = size(occupancyGrid);

    % BFS distance transform from start position.
    distanceGrid = computeWavefront(occupancyGrid, startPosition, numRows, numCols, resolution);

    % Look up wave distance for each candidate.
    waveDist = zeros(size(freeCandidates, 1), 1);
    for idx = 1:size(freeCandidates, 1)
        col = round(freeCandidates(idx, 1) * resolution) + 1;
        row = numRows - round(freeCandidates(idx, 2) * resolution);
        row = max(1, min(numRows, row));
        col = max(1, min(numCols, col));

        waveDist(idx) = distanceGrid(row, col);
    end

    % Assign to bands and apply boustrophedon ordering.
    waypoints = orderByWavefront(freeCandidates, waveDist, opts.Spacing * resolution);

    % ---------------------------------------------------------------
    % Optional visualization
    % ---------------------------------------------------------------
    if opts.Visualize
        figHandle = figure('Name', 'Generated Exploration Waypoints', 'NumberTitle', 'off');
        axVis = axes(figHandle);
        show(map, 'Parent', axVis);
        hold(axVis, 'on');
        plot(axVis, waypoints(:,1), waypoints(:,2), '-o', ...
            'Color', [0.85 0.33 0.10], 'MarkerFaceColor', [0.85 0.33 0.10], ...
            'MarkerSize', 4, 'LineWidth', 1.2, 'DisplayName', 'Exploration path');
        plot(axVis, startPosition(1), startPosition(2), 'gs', ...
            'MarkerFaceColor', 'g', 'MarkerSize', 10, 'DisplayName', 'Start');

        % Show wave-distance labels for debugging.
        for idx = 1:size(waypoints, 1)
            text(axVis, waypoints(idx,1), waypoints(idx,2), sprintf('%d', idx), ...
                'FontSize', 6, 'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'bottom', 'Color', [0.4 0.4 0.4]);
        end

        legend(axVis, 'Location', 'bestoutside');
        title(axVis, 'Wavefront Coverage Path');
        hold(axVis, 'off');
    end
end

% =====================================================================
% Local helper functions
% =====================================================================

function distanceGrid = computeWavefront(occupancyGrid, startPosition, numRows, numCols, resolution)
%COMPUTEWAVEFRONT BFS flood fill from startPosition on the free cells of
%the occupancy grid. Returns a grid where each free cell contains its
%BFS distance (in cells) from the start. Occupied cells get Inf.

    distanceGrid = inf(numRows, numCols);
    freeMask = occupancyGrid < 0.5;  % free cells

    % Convert start position to grid indices.
    startCol = round(startPosition(1) * resolution) + 1;
    startRow = numRows - round(startPosition(2) * resolution);
    startRow = max(1, min(numRows, startRow));
    startCol = max(1, min(numCols, startCol));

    if ~freeMask(startRow, startCol)
        % Start position is in an occupied cell; find nearest free cell.
        [freeRows, freeCols] = find(freeMask);
        dists = (freeRows - startRow).^2 + (freeCols - startCol).^2;
        [~, nearestIdx] = min(dists);
        startRow = freeRows(nearestIdx);
        startCol = freeCols(nearestIdx);
    end

    % BFS with preallocated queue.
    totalFree = nnz(freeMask);
    queueRows = zeros(totalFree, 1);
    queueCols = zeros(totalFree, 1);
    queueRows(1) = startRow;
    queueCols(1) = startCol;
    head = 1;
    tail = 1;

    distanceGrid(startRow, startCol) = 0;

    rowOffsets = [-1, 1, 0, 0, -1, -1, 1, 1];
    colOffsets = [0, 0, -1, 1, -1, 1, -1, 1];

    while head <= tail
        cr = queueRows(head);
        cc = queueCols(head);
        head = head + 1;
        currentDist = distanceGrid(cr, cc);

        for k = 1:8
            nr = cr + rowOffsets(k);
            nc = cc + colOffsets(k);

            if nr >= 1 && nr <= numRows && nc >= 1 && nc <= numCols ...
                    && freeMask(nr, nc) && distanceGrid(nr, nc) == inf
                if k <= 4
                    distanceGrid(nr, nc) = currentDist + 1;
                else
                    distanceGrid(nr, nc) = currentDist + 1.414;
                end
                tail = tail + 1;
                queueRows(tail) = nr;
                queueCols(tail) = nc;
            end
        end
    end
end

function ordered = orderByWavefront(points, waveDist, bandWidth)
%ORDERBYWAVEFRONT Sort waypoints by wavefront distance bands, with
%boustrophedon (alternating row-sweep) ordering within each band.

    % Handle unreachable points: place them at the end.
    reachable = waveDist < inf;
    reachablePoints = points(reachable, :);
    reachableDist   = waveDist(reachable);
    unreachablePoints = points(~reachable, :);

    if isempty(reachablePoints)
        % All unreachable; fall back to simple ordering.
        ordered = points;
        return;
    end

    % Assign each point to a distance band.
    bandIdx = floor(reachableDist / max(1, bandWidth));

    % Sort by band, then boustrophedon within each band.
    uniqueBands = unique(bandIdx);
    numBands = numel(uniqueBands);

    ordered = zeros(size(reachablePoints));
    writeIdx = 0;

    for b = 1:numBands
        bandMask = (bandIdx == uniqueBands(b));
        bandPoints = reachablePoints(bandMask, :);

        % Sort by Y (row), then alternate X direction per Y-row.
        % Quantize Y to Spacing-level rows to group nearby points.
        yQuantized = round(bandPoints(:, 2) * 2) / 2;  % 0.5m quantization
        uniqueYRows = unique(yQuantized);

        for r = 1:numel(uniqueYRows)
            rowMask = (yQuantized == uniqueYRows(r));
            rowPoints = bandPoints(rowMask, :);

            % Alternate sweep direction: even rows left-to-right, odd right-to-left.
            if mod(r, 2) == 1
                [~, sortIdx] = sort(rowPoints(:, 1), 'ascend');
            else
                [~, sortIdx] = sort(rowPoints(:, 1), 'descend');
            end
            rowPoints = rowPoints(sortIdx, :);

            nPts = size(rowPoints, 1);
            ordered(writeIdx+1:writeIdx+nPts, :) = rowPoints;
            writeIdx = writeIdx + nPts;
        end
    end

    ordered = ordered(1:writeIdx, :);

    % Append unreachable points at the end (if any).
    if ~isempty(unreachablePoints)
        ordered = [ordered; unreachablePoints];
    end
end
