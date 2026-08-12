function panelHandles = createStatusPanel(figHandle)
%CREATESTATUSPANEL Create a UI panel with real-time simulation diagnostics.
%
%   panelHandles = CREATESTATUSPANEL(figHandle)
%
%   Creates a uipanel on the right side of the figure displaying
%   navigation, robot, and SLAM fields. Returns a struct of uicontrol
%   handles whose 'String' properties are updated each tick by
%   updateStatusPanel.
%
%   Input:
%       figHandle - handle to the simulation figure
%
%   Output:
%       panelHandles - struct with a uicontrol handle for each value field

    % ---------------------------------------------------------------
    % Create the panel on the right edge of the figure.
    % ---------------------------------------------------------------
    panel = uipanel(figHandle, ...
        'Title', 'Dashboard', ...
        'FontSize', 9, ...
        'FontWeight', 'bold', ...
        'BackgroundColor', [0.95 0.95 0.95], ...
        'Units', 'normalized', ...
        'Position', [0.78 0.02 0.21 0.96]);

    % ---------------------------------------------------------------
    % Field definitions: {display-label, struct-field-name}.
    % Ordered by logical grouping: navigation, robot, SLAM.
    % ---------------------------------------------------------------
    fields = {
        'Sim Time',         'simTime'
        'FPS',              'fps'
        'Waypoint #',       'waypointNum'
        'Speed (m/s)',      'speed'
        'Ang Vel (rad/s)',  'angularVel'
        'Heading (deg)',    'heading'
        'Dist Travelled',   'distTravelled'
        'Robot State',      'robotState'
        'Collisions',       'collisionCounter'
        'Pose X (m)',       'poseX'
        'Pose Y (m)',       'poseY'
        'Hdg Error (deg)',  'headingError'
        'Dist to Goal',    'distToGoal'
        'Scan #',           'scanNumber'
        'Accepted',         'acceptedScans'
        'Rejected',         'rejectedScans'
        'Loop Closures',    'loopClosures'
        'Est Pose X',       'estPoseX'
        'Est Pose Y',       'estPoseY'
        'Loc Error (m)',    'locError'
        'Map Size',         'mapSize'
    };

    numFields  = size(fields, 1);
    rowHeight  = 0.042;
    topMargin  = 0.95;
    labelWidth = 0.58;
    bgColor    = [0.95 0.95 0.95];

    for k = 1:numFields
        yPos = topMargin - k * rowHeight;

        % Label (left side).
        uicontrol(panel, ...
            'Style', 'text', ...
            'String', [fields{k, 1}, ':'], ...
            'FontSize', 7, ...
            'FontWeight', 'bold', ...
            'HorizontalAlignment', 'left', ...
            'BackgroundColor', bgColor, ...
            'Units', 'normalized', ...
            'Position', [0.03 yPos labelWidth rowHeight - 0.005]);

        % Value (right side).
        panelHandles.(fields{k, 2}) = uicontrol(panel, ...
            'Style', 'text', ...
            'String', '--', ...
            'FontSize', 7, ...
            'HorizontalAlignment', 'right', ...
            'BackgroundColor', bgColor, ...
            'Units', 'normalized', ...
            'Position', [labelWidth + 0.03, yPos, ...
                         1 - labelWidth - 0.08, rowHeight - 0.005]);
    end
end
