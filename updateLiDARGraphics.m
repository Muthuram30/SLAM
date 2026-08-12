function lidarGraphics = updateLiDARGraphics(axesHandle, lidarGraphics, robotPose, scan, lidarConfig, showRays)
%UPDATELIDARGRAPHICS Create or update persistent LiDAR ray visualization.
%
%   lidarGraphics = UPDATELIDARGRAPHICS(axesHandle, lidarGraphics, ...
%       robotPose, scan, lidarConfig, showRays)
%
%   Draws LiDAR rays from the robot to each hit point, with hit markers.
%   Uses persistent graphics handles (create once, update XData/YData)
%   for flicker-free animation.
%
%   Inputs:
%       axesHandle    - axes to draw into.
%       lidarGraphics - struct of handles from previous call, or [].
%       robotPose     - 1x3 [x y theta].
%       scan          - lidarScan object from simulateLiDAR.
%       lidarConfig   - struct with at least .maxRange field.
%       showRays      - logical; if false, hides the LiDAR visualization.
%
%   Output:
%       lidarGraphics - struct with .rays and .hitPoints handles.

    % -----------------------------------------------------------------
    % Toggle visibility off when rays are disabled.
    % -----------------------------------------------------------------
    if ~showRays
        if isstruct(lidarGraphics) && isfield(lidarGraphics, 'rays')
            set(lidarGraphics.rays, 'Visible', 'off');
            set(lidarGraphics.hitPoints, 'Visible', 'off');
        end
        return;
    end

    % -----------------------------------------------------------------
    % Compute ray endpoints in world coordinates.
    % -----------------------------------------------------------------
    ranges = scan.Ranges;
    angles = scan.Angles;
    numRays = numel(ranges);

    worldAngles = angles + robotPose(3);

    endX = robotPose(1) + ranges .* cos(worldAngles);
    endY = robotPose(2) + ranges .* sin(worldAngles);

    % Separate actual hits from max-range returns.
    isHit = ranges < (lidarConfig.maxRange * 0.99);

    % -----------------------------------------------------------------
    % Build NaN-separated line data for all rays (single line object).
    % [start NaN end NaN start NaN end NaN ...] pattern avoids creating
    % N individual line objects, which would be far slower.
    % -----------------------------------------------------------------
    rayX = nan(3 * numRays, 1);
    rayY = nan(3 * numRays, 1);
    for r = 1:numRays
        base = (r - 1) * 3;
        rayX(base + 1) = robotPose(1);
        rayY(base + 1) = robotPose(2);
        rayX(base + 2) = endX(r);
        rayY(base + 2) = endY(r);
        % base+3 stays NaN (line-segment separator)
    end

    hitX = endX(isHit);
    hitY = endY(isHit);

    % -----------------------------------------------------------------
    % Create or update graphics handles.
    % -----------------------------------------------------------------
    isFirstCall = isempty(lidarGraphics) || ~isstruct(lidarGraphics) ...
        || ~isfield(lidarGraphics, 'rays');

    if isFirstCall
        hold(axesHandle, 'on');

        % Rays as a single semi-transparent line object.
        lidarGraphics.rays = plot(axesHandle, rayX, rayY, ...
            '-', 'Color', [1 0.2 0.2 0.15], 'LineWidth', 0.5, ...
            'HandleVisibility', 'off');

        % Hit points (red dots at obstacle surfaces).
        lidarGraphics.hitPoints = plot(axesHandle, hitX, hitY, ...
            '.', 'Color', [0.9 0.1 0.1], 'MarkerSize', 3, ...
            'DisplayName', 'LiDAR hits');

        % hold(axesHandle, 'off'); % Keep hold ON to prevent high-level plotting from clearing axes
    else
        set(lidarGraphics.rays, 'XData', rayX, 'YData', rayY, 'Visible', 'on');
        set(lidarGraphics.hitPoints, 'XData', hitX, 'YData', hitY, 'Visible', 'on');
    end
end
