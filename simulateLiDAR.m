function scan = simulateLiDAR(map, robotPose, lidarConfig)
%SIMULATELIDAR Simulate a 2D LiDAR sensor via ray casting.
%
%   scan = SIMULATELIDAR(map, robotPose, lidarConfig)
%
%   Pure sensor simulation -- performs ONLY:
%       Ground Truth Map  ->  Ray Casting  ->  Range Array  ->  lidarScan
%
%   This function does NOT call addScan, build maps, estimate poses,
%   or use any SLAM objects. It is a standalone virtual sensor.
%
%   Inputs:
%       map         - binaryOccupancyMap (ground truth environment).
%       robotPose   - 1x3 [x y theta] current robot pose in meters/rad.
%       lidarConfig - struct with fields:
%                       .maxRange          (m)  maximum detection range
%                       .minRange          (m)  minimum detection range
%                       .fov               (rad) total field of view
%                       .angularResolution (rad) angle step between rays
%                       .rangeNoise        (m)  std dev of Gaussian noise
%
%   Output:
%       scan - lidarScan object with simulated range measurements and
%              corresponding angles (in the robot body frame).
%
%   Algorithm:
%       1. Compute ray directions spanning [-fov/2, fov/2] in body frame.
%       2. For each ray, march in half-cell steps along the ray direction
%          in world coordinates, checking grid occupancy at each step.
%       3. Record the distance to the first occupied cell (hit) or
%          maxRange if no obstacle is encountered.
%       4. Apply additive Gaussian range noise.
%       5. Clamp ranges to [minRange, maxRange].
%       6. Return a MATLAB lidarScan object.

    % -----------------------------------------------------------------
    % Extract parameters.
    % -----------------------------------------------------------------
    maxRange  = lidarConfig.maxRange;
    minRange  = lidarConfig.minRange;
    fov       = lidarConfig.fov;
    angRes    = lidarConfig.angularResolution;
    noiseStd  = lidarConfig.rangeNoise;

    % -----------------------------------------------------------------
    % Compute ray angles (body frame, symmetric about forward direction).
    % -----------------------------------------------------------------
    angles  = (-fov/2 : angRes : fov/2)';
    numRays = numel(angles);

    % World-frame angles (body angles rotated by robot heading).
    worldAngles = angles + robotPose(3);

    % Precompute direction unit vectors for every ray.
    cosAngles = cos(worldAngles);
    sinAngles = sin(worldAngles);

    % -----------------------------------------------------------------
    % Prepare the occupancy grid for direct lookup (fast path).
    % Avoids the per-point overhead of checkOccupancy().
    % -----------------------------------------------------------------
    grid       = getOccupancy(map);
    [numRows, numCols] = size(grid);
    resolution = map.Resolution;
    xLimits    = map.XWorldLimits;
    yLimits    = map.YWorldLimits;

    stepSize  = 0.5 / resolution;                % half-cell steps
    maxSteps  = ceil(maxRange / stepSize);

    % -----------------------------------------------------------------
    % Ray casting -- march each ray until hit, out-of-bounds, or maxRange.
    % -----------------------------------------------------------------
    ranges = maxRange * ones(numRays, 1);         % default: no hit

    rx = robotPose(1);
    ry = robotPose(2);

    for r = 1:numRays
        dx = cosAngles(r) * stepSize;
        dy = sinAngles(r) * stepSize;
        px = rx;
        py = ry;

        for s = 1:maxSteps
            px = px + dx;
            py = py + dy;

            % Bounds check -- ray exited the map.
            if px < xLimits(1) || px > xLimits(2) || ...
               py < yLimits(1) || py > yLimits(2)
                break;   % keep maxRange
            end

            % Direct grid lookup.
            col = round(px * resolution) + 1;
            row = numRows - round(py * resolution);

            if row >= 1 && row <= numRows && col >= 1 && col <= numCols
                if grid(row, col)                 % occupied cell hit
                    ranges(r) = hypot(px - rx, py - ry);
                    break;
                end
            end
        end
    end

    % -----------------------------------------------------------------
    % Apply Gaussian noise and clamp to valid range interval.
    % -----------------------------------------------------------------
    ranges = ranges + noiseStd * randn(numRays, 1);
    ranges = max(minRange, min(maxRange, ranges));

    % -----------------------------------------------------------------
    % Create the lidarScan object (angles remain in body frame).
    % -----------------------------------------------------------------
    scan = lidarScan(ranges, angles);
end
