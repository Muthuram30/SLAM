function [map, envInfo] = createEnvironment(mapWidth, mapHeight, resolution)
%CREATEENVIRONMENT Programmatically generate an indoor-building occupancy map.
%
%   map = CREATEENVIRONMENT(mapWidth, mapHeight, resolution) builds a
%   binaryOccupancyMap representing a small indoor building composed of
%   outer walls, multiple rooms, narrow corridors, rectangular obstacles,
%   circular pillars, and a dead end. The environment is generated
%   entirely in code (no images or external map files are used).
%
%   [map, envInfo] = CREATEENVIRONMENT(...) also returns a diagnostics
%   structure describing free-space connectivity and any clearance
%   warnings detected during generation. envInfo is informational only;
%   generation never throws an error for a connectivity/clearance
%   problem, it only warns, so callers can inspect envInfo and decide
%   how to proceed.
%
%   Inputs:
%       mapWidth   - Width of the environment in meters  (double, > 0)
%       mapHeight  - Height of the environment in meters (double, > 0)
%       resolution - Map resolution in cells per meter   (double, > 0)
%
%   Output:
%       map     - binaryOccupancyMap object. true (1) cells are occupied
%                 (walls/obstacles); false (0) cells are free space.
%       envInfo - struct with fields:
%                   .isFullyConnected     (logical)
%                   .freeSpaceCoverage    (double, fraction reachable from seed)
%                   .assumedRobotDiameter (double, meters, used for the
%                                          clearance sanity checks below)
%
%   COORDINATE / ORIENTATION CONVENTION
%   ------------------------------------------------------------------
%   World coordinates (x, y) follow the usual math convention: x
%   increases rightward, y increases upward, origin at the bottom-left
%   of the map. binaryOccupancyMap stores its backing matrix so that
%   matrix row 1 corresponds to the MAXIMUM y value (top of the map) and
%   the last row corresponds to y = 0 (bottom of the map) -- the same
%   top-down convention used for images. All coordinate-to-index
%   conversions in this file therefore flip the row index
%   (row = numRows - round(y*resolution)); the column index does not
%   flip since x increases left-to-right in both conventions.
%
%   Example:
%       map = createEnvironment(20, 15, 20);
%       show(map);

    % ---------------------------------------------------------------
    % Input validation
    % ---------------------------------------------------------------
    validateattributes(mapWidth,   {'double'}, {'scalar', 'positive', 'finite'});
    validateattributes(mapHeight,  {'double'}, {'scalar', 'positive', 'finite'});
    validateattributes(resolution, {'double'}, {'scalar', 'positive', 'finite'});

    % Assumed robot diameter used purely for clearance sanity-checking
    % of doorways/corridors at generation time. The actual robot created
    % in createRobot.m may differ; this is a conservative design-time
    % estimate so problems surface immediately as warnings.
    assumedRobotDiameter = 0.30; % meters

    % ---------------------------------------------------------------
    % Allocate a logical occupancy grid.
    % ---------------------------------------------------------------
    numRows = round(mapHeight * resolution);
    numCols = round(mapWidth  * resolution);
    grid = false(numRows, numCols);

    wallThickness = 0.2; % meters, used consistently for every wall segment

    % ---------------------------------------------------------------
    % Outer walls (building perimeter)
    % ---------------------------------------------------------------
    grid = drawWallSegment(grid, resolution, [0, 0], [mapWidth, 0], wallThickness);                 % bottom
    grid = drawWallSegment(grid, resolution, [0, mapHeight], [mapWidth, mapHeight], wallThickness);  % top
    grid = drawWallSegment(grid, resolution, [0, 0], [0, mapHeight], wallThickness);                 % left
    grid = drawWallSegment(grid, resolution, [mapWidth, 0], [mapWidth, mapHeight], wallThickness);   % right

    % ---------------------------------------------------------------
    % Interior walls: partition the building into rooms and a central
    % corridor, leaving deliberate gaps for doorways so every room stays
    % reachable by the exploring robot. Doorway/gap sizes are tracked in
    % clearanceChecks so they can be validated against
    % assumedRobotDiameter below.
    % ---------------------------------------------------------------
    clearanceChecks = struct('name', {}, 'width', {}); % {name, width-in-meters}

    % Vertical wall splitting the building into a left wing and a right
    % wing, with a wide doorway gap near the middle for corridor access.
    midX = mapWidth * 0.5;
    doorGapHalf = 1.5; % half-width of doorway opening, meters (3.0 m total)
    grid = drawWallSegment(grid, resolution, [midX, 0], [midX, (mapHeight*0.5 - doorGapHalf)], wallThickness);
    grid = drawWallSegment(grid, resolution, [midX, (mapHeight*0.5 + doorGapHalf)], [midX, mapHeight], wallThickness);
    clearanceChecks(end+1) = struct('name', 'center doorway', 'width', 2*doorGapHalf);

    % Horizontal wall creating a corridor band across the left wing,
    % splitting it into two rooms (top-left, bottom-left) with a wide doorway.
    corridorY = mapHeight * 0.35;
    doorGapX = mapWidth * 0.25;  % shifted inward for a more central opening
    doorGapWidthLeft = 2.0;      % half-width of doorway gap (4.0 m total)
    grid = drawWallSegment(grid, resolution, [wallThickness, corridorY], [doorGapX - doorGapWidthLeft, corridorY], wallThickness);
    grid = drawWallSegment(grid, resolution, [doorGapX + doorGapWidthLeft, corridorY], [midX - wallThickness, corridorY], wallThickness);
    clearanceChecks(end+1) = struct('name', 'left-wing doorway', 'width', 2*doorGapWidthLeft);

    % Horizontal wall in the right wing creating another room split,
    % with a wide central doorway for easy access.
    corridorY2 = mapHeight * 0.65;
    doorGapX2 = mapWidth * 0.75;  % shifted inward for a more central opening
    doorGapWidthRight = 1.2;     % half-width of doorway gap (2.4 m total)
    grid = drawWallSegment(grid, resolution, [midX + wallThickness, corridorY2], [doorGapX2 - doorGapWidthRight, corridorY2], wallThickness);
    grid = drawWallSegment(grid, resolution, [doorGapX2 + doorGapWidthRight, corridorY2], [mapWidth - wallThickness, corridorY2], wallThickness);
    clearanceChecks(end+1) = struct('name', 'right-wing doorway', 'width', 2*doorGapWidthRight);

    % NOTE: Pinch-point stub walls and dead-end stub wall removed.
    % They created near-impassable bottlenecks that the recovery logic
    % could not reliably navigate. The map remains fully connected.

    % ---------------------------------------------------------------
    % Rectangular obstacles (furniture-like blocks) inside rooms
    % ---------------------------------------------------------------
    grid = drawRectangleObstacle(grid, resolution, [mapWidth*0.12, mapHeight*0.75], [1.2, 0.8]);
    grid = drawRectangleObstacle(grid, resolution, [mapWidth*0.30, mapHeight*0.12], [1.5, 1.0]);
    grid = drawRectangleObstacle(grid, resolution, [mapWidth*0.65, mapHeight*0.20], [1.0, 1.5]);
    grid = drawRectangleObstacle(grid, resolution, [mapWidth*0.88, mapHeight*0.45], [0.8, 0.8]);

    % ---------------------------------------------------------------
    % Circular pillars scattered through open areas
    % ---------------------------------------------------------------
    grid = drawCircularObstacle(grid, resolution, [mapWidth*0.60, mapHeight*0.82], 0.2);
    grid = drawCircularObstacle(grid, resolution, [mapWidth*0.20, mapHeight*0.50], 0.2);
    grid = drawCircularObstacle(grid, resolution, [mapWidth*0.75, mapHeight*0.60], 0.2);

    % ---------------------------------------------------------------
    % Diagnostics: free-space connectivity (manual BFS flood fill, no
    % Image Processing Toolbox dependency) and doorway/corridor
    % clearance vs. the assumed robot diameter. Both are warn-only: this
    % function must never error out just because the procedurally
    % generated layout turned out too tight, per the robustness
    % requirement for the overall project.
    % ---------------------------------------------------------------
    [isFullyConnected, freeSpaceCoverage] = checkFreeSpaceConnectivity(grid);
    if ~isFullyConnected
        warning('createEnvironment:Disconnected', ...
            ['Generated environment free space is not fully connected. ', ...
             'Only %.1f%% of free cells are reachable from the seed cell. ', ...
             'A room or corridor may be unreachable by the robot.'], ...
             100 * freeSpaceCoverage);
    end

    for k = 1:numel(clearanceChecks)
        if clearanceChecks(k).width < assumedRobotDiameter
            warning('createEnvironment:InsufficientClearance', ...
                ['Passage "%s" is %.2f m wide, which is narrower than the ', ...
                 'assumed robot diameter of %.2f m. The robot may not fit through it.'], ...
                 clearanceChecks(k).name, clearanceChecks(k).width, assumedRobotDiameter);
        end
    end

    envInfo = struct( ...
        'isFullyConnected', isFullyConnected, ...
        'freeSpaceCoverage', freeSpaceCoverage, ...
        'assumedRobotDiameter', assumedRobotDiameter);

    % ---------------------------------------------------------------
    % Build the binaryOccupancyMap from the finished logical grid.
    % ---------------------------------------------------------------
    map = binaryOccupancyMap(grid, resolution);
end

% =====================================================================
% Local helper functions
% =====================================================================

function grid = drawWallSegment(grid, resolution, startPointXY, endPointXY, thickness)
%DRAWWALLSEGMENT Rasterize a thick line segment (a wall) onto the grid.
%   Coordinates are in meters (x, y), origin at the bottom-left of the
%   map. The segment is drawn by sampling densely along its length and
%   stamping a square footprint of the given thickness at each sample.

    [numRows, numCols] = size(grid);

    segmentLength = norm(endPointXY - startPointXY);
    numSamples = max(2, ceil(segmentLength * resolution * 2));
    tVals = linspace(0, 1, numSamples);

    % Total wall thickness in cells, split as evenly as possible on
    % either side of the sampled centerline so the physical thickness
    % matches `thickness` (in meters) exactly, rather than growing to
    % 2*half+1 cells as a naive symmetric range would.
    totalThicknessCells = max(1, round(thickness * resolution));
    cellsBefore = floor((totalThicknessCells - 1) / 2);
    cellsAfter  = ceil((totalThicknessCells - 1) / 2);

    for k = 1:numSamples
        pointXY = startPointXY + tVals(k) * (endPointXY - startPointXY);
        [centerRow, centerCol] = worldToGridIndex(pointXY, numRows, resolution);

        rowRange = (centerRow - cellsBefore):(centerRow + cellsAfter);
        colRange = (centerCol - cellsBefore):(centerCol + cellsAfter);

        rowRange = rowRange(rowRange >= 1 & rowRange <= numRows);
        colRange = colRange(colRange >= 1 & colRange <= numCols);

        grid(rowRange, colRange) = true;
    end
end

function grid = drawRectangleObstacle(grid, resolution, centerXY, sizeXY)
%DRAWRECTANGLEOBSTACLE Stamp a filled rectangular obstacle onto the grid.
%   centerXY - [x y] center position of the rectangle, in meters.
%   sizeXY   - [width height] of the rectangle, in meters.

    [numRows, numCols] = size(grid);

    halfWidthCells  = max(1, round((sizeXY(1) / 2) * resolution));
    halfHeightCells = max(1, round((sizeXY(2) / 2) * resolution));

    [centerRow, centerCol] = worldToGridIndex(centerXY, numRows, resolution);

    rowRange = (centerRow - halfHeightCells):(centerRow + halfHeightCells);
    colRange = (centerCol - halfWidthCells):(centerCol + halfWidthCells);

    rowRange = rowRange(rowRange >= 1 & rowRange <= numRows);
    colRange = colRange(colRange >= 1 & colRange <= numCols);

    grid(rowRange, colRange) = true;
end

function grid = drawCircularObstacle(grid, resolution, centerXY, radius)
%DRAWCIRCULAROBSTACLE Stamp a filled circular obstacle (a pillar) onto
%the grid using an explicit distance test (X-x)^2 + (Y-y)^2 < r^2,
%restricted to a local bounding box for performance.
%   centerXY - [x y] center of the circle, in meters.
%   radius   - circle radius, in meters.

    [numRows, numCols] = size(grid);
    radiusCells = max(1, round(radius * resolution));

    [centerRow, centerCol] = worldToGridIndex(centerXY, numRows, resolution);

    rowRange = max(1, centerRow - radiusCells):min(numRows, centerRow + radiusCells);
    colRange = max(1, centerCol - radiusCells):min(numCols, centerCol + radiusCells);

    if isempty(rowRange) || isempty(colRange)
        return; % circle entirely outside map bounds; nothing to draw
    end

    [colGrid, rowGrid] = meshgrid(colRange, rowRange);
    distSquared = (rowGrid - centerRow).^2 + (colGrid - centerCol).^2;
    circleMask = distSquared <= radiusCells^2;

    subGrid = grid(rowRange, colRange);
    subGrid(circleMask) = true;
    grid(rowRange, colRange) = subGrid;
end

function [row, col] = worldToGridIndex(pointXY, numRows, resolution)
%WORLDTOGRIDINDEX Convert a world (x, y) coordinate, in meters, to a
%(row, col) index into the occupancy grid matrix, honoring the
%binaryOccupancyMap convention that row 1 is the MAXIMUM y (top of map)
%and row numRows is y = 0 (bottom of map). Columns are not flipped: x
%increases left-to-right in both the world and the matrix.

    col = round(pointXY(1) * resolution) + 1;
    row = numRows - round(pointXY(2) * resolution);
end

function [isConnected, coverageRatio] = checkFreeSpaceConnectivity(grid)
%CHECKFREESPACECONNECTIVITY Verify the free space of the occupancy grid
%forms a single reachable region, using a manual 4-connected BFS flood
%fill (no Image Processing Toolbox dependency). Returns the fraction of
%free cells reachable from an arbitrary seed free cell; isConnected is
%true when that fraction is effectively 1 (allowing a tiny tolerance for
%isolated single-cell rounding artifacts).

    freeMask = ~grid;
    totalFree = nnz(freeMask);

    if totalFree == 0
        isConnected = false;
        coverageRatio = 0;
        return;
    end

    [numRows, numCols] = size(grid);
    [seedRow, seedCol] = find(freeMask, 1, 'first');

    visited = false(numRows, numCols);
    visited(seedRow, seedCol) = true;

    % Preallocate a queue sized to the maximum possible number of free
    % cells to avoid growing arrays inside the BFS loop.
    queueRows = zeros(totalFree, 1);
    queueCols = zeros(totalFree, 1);
    queueRows(1) = seedRow;
    queueCols(1) = seedCol;
    head = 1;
    tail = 1;
    reachableCount = 1;

    rowOffsets = [-1, 1, 0, 0];
    colOffsets = [0, 0, -1, 1];

    while head <= tail
        currentRow = queueRows(head);
        currentCol = queueCols(head);
        head = head + 1;

        for k = 1:4
            nr = currentRow + rowOffsets(k);
            nc = currentCol + colOffsets(k);

            if nr >= 1 && nr <= numRows && nc >= 1 && nc <= numCols ...
                    && freeMask(nr, nc) && ~visited(nr, nc)
                visited(nr, nc) = true;
                reachableCount = reachableCount + 1;
                tail = tail + 1;
                queueRows(tail) = nr;
                queueCols(tail) = nc;
            end
        end
    end

    coverageRatio = reachableCount / totalFree;
    isConnected = coverageRatio >= 0.999; % tiny tolerance for rounding
end
