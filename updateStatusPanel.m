function updateStatusPanel(panelHandles, robot, controllerState, numWaypoints, ...
    simulationTime, totalDistanceTraveled, slamStats, fps)
%UPDATESTATUSPANEL Update all text values in the status panel.
%
%   updateStatusPanel(panelHandles, robot, controllerState, numWaypoints, ...
%       simulationTime, totalDistanceTraveled, slamStats, fps)
%
%   Efficient: only sets 'String' properties on existing uicontrol handles.
%
%   Inputs:
%       panelHandles          - struct from createStatusPanel
%       robot                 - robot struct with pose and diagnostics
%       controllerState       - struct with .currentWaypointIdx
%       numWaypoints          - total number of waypoints
%       simulationTime        - elapsed simulation time, seconds
%       totalDistanceTraveled - cumulative distance, meters
%       slamStats             - struct from updateSLAM (may be empty)
%       fps                   - current frames per second (may be 0)

    % Defaults for optional parameters.
    if nargin < 7 || isempty(slamStats)
        slamStats = struct('totalAccepted', 0, 'totalRejected', 0, ...
            'totalLoopClosures', 0, 'estimatedPose', [NaN NaN NaN], ...
            'localizationError', NaN);
    end
    if nargin < 8
        fps = 0;
    end

    % ---------------------------------------------------------------
    % Navigation fields.
    % ---------------------------------------------------------------
    wpIdx = min(controllerState.currentWaypointIdx, numWaypoints);
    isComplete = controllerState.currentWaypointIdx > numWaypoints;

    set(panelHandles.simTime, 'String', sprintf('%.1f s', simulationTime));
    set(panelHandles.fps,     'String', sprintf('%.0f', fps));

    if isComplete
        set(panelHandles.waypointNum, 'String', 'Done');
    else
        set(panelHandles.waypointNum, 'String', sprintf('%d / %d', wpIdx, numWaypoints));
    end

    set(panelHandles.speed,         'String', sprintf('%.3f', robot.linearVel));
    set(panelHandles.angularVel,    'String', sprintf('%.3f', robot.angularVel));
    set(panelHandles.heading,       'String', sprintf('%.1f', rad2deg(robot.pose(3))));
    set(panelHandles.distTravelled, 'String', sprintf('%.2f m', totalDistanceTraveled));
    set(panelHandles.robotState,'String',char(robot.navigationState));
    set(panelHandles.collisionCounter, 'String', sprintf('%d', robot.collisionCounter));

    % ---------------------------------------------------------------
    % Robot pose and controller fields.
    % ---------------------------------------------------------------
    set(panelHandles.poseX, 'String', sprintf('%.3f', robot.pose(1)));
    set(panelHandles.poseY, 'String', sprintf('%.3f', robot.pose(2)));

    if isnan(robot.headingError)
        set(panelHandles.headingError, 'String', '--');
    else
        set(panelHandles.headingError, 'String', sprintf('%.1f', rad2deg(robot.headingError)));
    end

    if isnan(robot.distanceToGoal)
        set(panelHandles.distToGoal, 'String', '--');
    else
        set(panelHandles.distToGoal, 'String', sprintf('%.3f m', robot.distanceToGoal));
    end

    % ---------------------------------------------------------------
    % SLAM fields.
    % ---------------------------------------------------------------
    totalScans = 0;
    if isfield(slamStats, 'totalAccepted') && isfield(slamStats, 'totalRejected')
        totalScans = slamStats.totalAccepted + slamStats.totalRejected;
    end

    set(panelHandles.scanNumber,    'String', sprintf('%d', totalScans));
    set(panelHandles.acceptedScans, 'String', sprintf('%d', slamStats.totalAccepted));
    set(panelHandles.rejectedScans, 'String', sprintf('%d', slamStats.totalRejected));
    set(panelHandles.loopClosures,  'String', sprintf('%d', slamStats.totalLoopClosures));

    if isfield(slamStats, 'estimatedPose') && ~any(isnan(slamStats.estimatedPose))
        set(panelHandles.estPoseX, 'String', sprintf('%.3f', slamStats.estimatedPose(1)));
        set(panelHandles.estPoseY, 'String', sprintf('%.3f', slamStats.estimatedPose(2)));
    else
        set(panelHandles.estPoseX, 'String', '--');
        set(panelHandles.estPoseY, 'String', '--');
    end

    if isfield(slamStats, 'localizationError') && ~isnan(slamStats.localizationError)
        set(panelHandles.locError, 'String', sprintf('%.4f', slamStats.localizationError));
    else
        set(panelHandles.locError, 'String', '--');
    end

    % Map size is set externally from main.m via a direct set() call
    % (since it depends on the slamMap object which is not passed here).
end
