function graphicsHandles = updateRobotGraphics(axesHandle, graphicsHandles, robot, ...
    waypoints, currentWaypointIdx, trajectoryBuffer, trajectoryCount, ...
    estTrajectoryBuffer, estTrajectoryCount)
%UPDATEROBOTGRAPHICS Create or update persistent graphics for the robot,
%waypoints, ground-truth trajectory, and estimated trajectory.
%
%   graphicsHandles = UPDATEROBOTGRAPHICS(axesHandle, graphicsHandles, ...
%       robot, waypoints, currentWaypointIdx, trajectoryBuffer, ...
%       trajectoryCount, estTrajectoryBuffer, estTrajectoryCount)
%
%   Waypoint visualization:
%       Visited   - green filled circles
%       Current   - gold star
%       Remaining - gray open circles
%
%   Trajectories:
%       Ground truth - solid blue
%       Estimated    - dashed orange (from SLAM)
%
%   The last two arguments are optional for backward compatibility.

    if nargin < 8
        estTrajectoryBuffer = [];
    end
    if nargin < 9
        estTrajectoryCount = 0;
    end

    % Guard: axesHandle must be a live axes object.  The Navigation Toolbox
    % show() function can silently replace the axes it is given (via an
    % internal newplot() call), leaving the caller with a deleted handle.
    % Detect this early so the error message is actionable.
    if ~isgraphics(axesHandle, 'axes')
        error('updateRobotGraphics:DeletedAxes', ...
            ['axesHandle is not a valid axes object (it may have been ', ...
             'deleted by show()). Re-acquire the axes handle in main.m ', ...
             'after calling show(groundTruthMap, ...).']);
    end

    % A graphics handle set is considered invalid or needs recreation if:
    % 1. graphicsHandles is empty or not a struct.
    % 2. Any of the required fields are missing.
    % 3. Any of the handles are no longer valid graphics objects (e.g., cleared by cla or figure resize).
    requiredFields = {'remainingWaypoints', 'visitedWaypoints', 'trajectoryLine', ...
                      'estTrajectoryLine', 'robotBody', 'headingLine', 'waypointMarker'};
    
    needRecreate = isempty(graphicsHandles) || ~isstruct(graphicsHandles);
    if ~needRecreate
        for i = 1:numel(requiredFields)
            f = requiredFields{i};
            if ~isfield(graphicsHandles, f)
                needRecreate = true;
                break;
            end
            h = graphicsHandles.(f);
            % isgraphics requires a scalar numeric/handle; guard against
            % empty, non-numeric, or array values before calling it.
            if ~isscalar(h) || ~isnumeric(h) && ~isgraphics(h) || ~isgraphics(h)
                needRecreate = true;
                break;
            end
        end
    end

    pose = robot.pose;
    headingLength = robot.radius * 2.2;
    headingEndX = pose(1) + headingLength * cos(pose(3));
    headingEndY = pose(2) + headingLength * sin(pose(3));

    % Robot body: parametric circle polygon.
    circleTheta = linspace(0, 2*pi, 24);
    bodyX = pose(1) + robot.radius * cos(circleTheta);
    bodyY = pose(2) + robot.radius * sin(circleTheta);

    % ---------------------------------------------------------------
    % Compute waypoint category data.
    % ---------------------------------------------------------------
    numWaypoints = size(waypoints, 1);
    clampedIdx = max(1, min(currentWaypointIdx, numWaypoints + 1));

    if clampedIdx > 1 && numWaypoints > 0
        visitedX = waypoints(1:clampedIdx-1, 1);
        visitedY = waypoints(1:clampedIdx-1, 2);
    else
        visitedX = NaN; visitedY = NaN;
    end

    if clampedIdx >= 1 && clampedIdx <= numWaypoints
        currentX = waypoints(clampedIdx, 1);
        currentY = waypoints(clampedIdx, 2);
    else
        currentX = NaN; currentY = NaN;
    end

    if clampedIdx < numWaypoints && numWaypoints > 0
        remainingX = waypoints(clampedIdx+1:end, 1);
        remainingY = waypoints(clampedIdx+1:end, 2);
    else
        remainingX = NaN; remainingY = NaN;
    end

    % ---------------------------------------------------------------
    % Create or update graphics handles.
    % ---------------------------------------------------------------
    if needRecreate
        % Clean up any remaining valid objects to prevent duplicates/orphans.
        if isstruct(graphicsHandles)
            fields = fieldnames(graphicsHandles);
            for i = 1:numel(fields)
                h = graphicsHandles.(fields{i});
                if isgraphics(h)
                    delete(h);
                end
            end
        end
        graphicsHandles = struct();

        hold(axesHandle, 'on');

        % Remaining waypoints (gray, open circles).
        graphicsHandles.remainingWaypoints = plot(axesHandle, remainingX, remainingY, ...
            'o', 'MarkerSize', 4, 'MarkerEdgeColor', [0.6 0.6 0.6], ...
            'MarkerFaceColor', 'none', 'LineStyle', 'none', ...
            'DisplayName', 'Remaining waypoints');

        % Visited waypoints (green filled circles).
        graphicsHandles.visitedWaypoints = plot(axesHandle, visitedX, visitedY, ...
            'o', 'MarkerSize', 4, 'MarkerEdgeColor', [0.2 0.7 0.2], ...
            'MarkerFaceColor', [0.2 0.7 0.2], 'LineStyle', 'none', ...
            'DisplayName', 'Visited waypoints');

        % Ground-truth trajectory line (solid blue).
        graphicsHandles.trajectoryLine = plot(axesHandle, NaN, NaN, ...
            '-', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.5, ...
            'DisplayName', 'Ground-truth trajectory');

        % Estimated trajectory line (dashed orange).
        graphicsHandles.estTrajectoryLine = plot(axesHandle, NaN, NaN, ...
            '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5, ...
            'DisplayName', 'Estimated trajectory');

        % Robot body.
        graphicsHandles.robotBody = fill(axesHandle, bodyX, bodyY, robot.color, ...
            'EdgeColor', 'k', 'LineWidth', 1, 'DisplayName', 'Robot');

        % Heading indicator.
        graphicsHandles.headingLine = plot(axesHandle, ...
            [pose(1), headingEndX], [pose(2), headingEndY], ...
            'k-', 'LineWidth', 2, 'HandleVisibility', 'off');

        % Current target waypoint (gold star).
        graphicsHandles.waypointMarker = plot(axesHandle, currentX, currentY, ...
            'p', 'MarkerSize', 12, 'MarkerFaceColor', [0.93 0.69 0.13], ...
            'MarkerEdgeColor', 'k', 'DisplayName', 'Target waypoint');

        legend(axesHandle, 'Location', 'bestoutside');
        % hold(axesHandle, 'off'); % Keep hold ON to prevent high-level plotting from clearing axes
    else
        % Update existing handles -- efficient, no flicker.
        set(graphicsHandles.robotBody, 'XData', bodyX, 'YData', bodyY);
        set(graphicsHandles.headingLine, ...
            'XData', [pose(1), headingEndX], 'YData', [pose(2), headingEndY]);
        set(graphicsHandles.waypointMarker, 'XData', currentX, 'YData', currentY);
        set(graphicsHandles.visitedWaypoints, 'XData', visitedX, 'YData', visitedY);
        set(graphicsHandles.remainingWaypoints, 'XData', remainingX, 'YData', remainingY);
    end

    % ---------------------------------------------------------------
    % Update trajectory lines.
    % ---------------------------------------------------------------
    if trajectoryCount >= 1
        set(graphicsHandles.trajectoryLine, ...
            'XData', trajectoryBuffer(1:trajectoryCount, 1), ...
            'YData', trajectoryBuffer(1:trajectoryCount, 2));
    end

    if estTrajectoryCount >= 1 && ~isempty(estTrajectoryBuffer)
        set(graphicsHandles.estTrajectoryLine, ...
            'XData', estTrajectoryBuffer(1:estTrajectoryCount, 1), ...
            'YData', estTrajectoryBuffer(1:estTrajectoryCount, 2));
    end
end
