function robot = createRobot(map, initialPose, varargin)
%CREATEROBOT Create a differential-drive robot model for the SLAM sim.
%
%   robot = CREATEROBOT(map, initialPose) creates a robot structure with
%   default physical parameters, validating that initialPose lies inside
%   the map bounds and in free space (including a safety-radius
%   clearance check against nearby walls/obstacles).
%
%   robot = CREATEROBOT(map, initialPose, 'ParamName', value, ...)
%   overrides any default physical parameter. Supported name-value
%   pairs (defaults shown):
%       'Radius'         0.15  - robot body radius, meters
%       'SafetyRadius'   0.22  - clearance radius used for pose
%                                validation and traversability checks,
%                                meters (>= Radius)
%       'WheelBase'      0.25  - distance between the two drive wheels, m
%       'MaxLinearVel'   0.5   - maximum linear speed, m/s
%       'MaxAngularVel'  1.5   - maximum angular speed, rad/s
%       'Color'          [0 0.4470 0.7410] - RGB robot body color
%
%   Inputs:
%       map         - binaryOccupancyMap (or occupancyMap) describing
%                     the environment the robot will operate in.
%       initialPose - 1x3 vector [x y theta] (meters, meters, radians).
%
%   Output:
%       robot - struct with fields:
%                 .pose            1x3 [x y theta], current pose
%                 .radius          robot body radius, meters
%                 .safetyRadius    clearance radius, meters
%                 .wheelBase       meters
%                 .maxLinearVel    m/s
%                 .maxAngularVel   rad/s
%                 .color           1x3 RGB
%
%   The struct representation (rather than a handle class) keeps the
%   robot easy to pass into, and return from, the later motion/SLAM
%   functions without hidden shared state.
%
%   Example:
%       map = createEnvironment(20, 15, 20);
%       robot = createRobot(map, [1.5 1.5 0]);

    % ---------------------------------------------------------------
    % Parse and validate optional parameters
    % ---------------------------------------------------------------
    parser = inputParser;
    addRequired(parser, 'map');
    addRequired(parser, 'initialPose', @(p) validateattributes(p, ...
        {'double'}, {'vector', 'numel', 3, 'finite'}));
    addParameter(parser, 'Radius', 0.15, @(v) validateattributes(v, ...
        {'double'}, {'scalar', 'positive', 'finite'}));
    addParameter(parser, 'SafetyRadius', 0.22, @(v) validateattributes(v, ...
        {'double'}, {'scalar', 'positive', 'finite'}));
    addParameter(parser, 'WheelBase', 0.25, @(v) validateattributes(v, ...
        {'double'}, {'scalar', 'positive', 'finite'}));
    addParameter(parser, 'MaxLinearVel', 0.5, @(v) validateattributes(v, ...
        {'double'}, {'scalar', 'positive', 'finite'}));
    addParameter(parser, 'MaxAngularVel', 1.5, @(v) validateattributes(v, ...
        {'double'}, {'scalar', 'positive', 'finite'}));
    addParameter(parser, 'Color', [0 0.4470 0.7410], @(v) validateattributes(v, ...
        {'double'}, {'vector', 'numel', 3}));

    parse(parser, map, initialPose, varargin{:});
    opts = parser.Results;

    if opts.SafetyRadius < opts.Radius
        error('createRobot:InvalidSafetyRadius', ...
            'SafetyRadius (%.3f m) must be >= Radius (%.3f m).', ...
            opts.SafetyRadius, opts.Radius);
    end

    initialPose = reshape(double(initialPose), 1, 3);

    % ---------------------------------------------------------------
    % Validate the initial pose: inside map bounds and clear of
    % occupied cells within the safety radius.
    % ---------------------------------------------------------------
    validatePoseIsFree(map, initialPose, opts.SafetyRadius);

    % ---------------------------------------------------------------
    % Assemble the robot structure.
    % ---------------------------------------------------------------
    % ==========================================================
    % Physical properties
    % ==========================================================

    robot.pose          = initialPose;
    robot.radius        = opts.Radius;
    robot.safetyRadius  = opts.SafetyRadius;
    robot.wheelBase     = opts.WheelBase;
    robot.maxLinearVel  = opts.MaxLinearVel;
    robot.maxAngularVel = opts.MaxAngularVel;
    robot.color         = opts.Color;

    % ==========================================================
    % Motion diagnostics
    % ==========================================================

    robot.linearVel      = 0;
    robot.angularVel     = 0;
    robot.targetWaypoint = [NaN NaN];
    robot.distanceToGoal = NaN;
    robot.headingError   = NaN;

    % ==========================================================
    % Navigation Finite State Machine (FSM)
    % ==========================================================
    %
    % States:
    %   "NORMAL"
    %   "ROTATE_LEFT"
    %   "TRY_FORWARD"
    %   "ROTATE_RIGHT"
    %   "BACKUP"
    %
    % The robot begins in NORMAL navigation mode.
    %

    robot.navigationState = "NAVIGATING";

    % Counts consecutive failed forward motions.
    robot.collisionCounter = 0;

    % Which recovery stage is currently active.
    robot.recoveryStage = 0;

    % Number of simulation steps spent in the current recovery state.
    robot.recoveryTimer = 0;

    % Direction currently used during recovery.
    % +1 = left
    % -1 = right
    robot.recoveryDirection = 1;

    % Heading when recovery starts.
    robot.recoveryStartHeading = initialPose(3);

    % Desired heading before recovery finishes.
    robot.targetRecoveryHeading = initialPose(3);

    % Distance reversed during BACKUP state.
    robot.backupDistance = 0;

    % Latest result from analyzeRecovery — populated on first collision.
    % Direction, angle, envType, confidence, and sector scores are held
    % here so the dashboard and logger can display recovery diagnostics.
    % Memory fields (lastDirection, lastEnvType, failedAttempts, deadEndCount)
    % carry state across calls to enable anti-oscillation and persistence.
    robot.recoveryAnalysis = struct( ...
        'direction',           0, ...
        'angle',               deg2rad(25), ...
        'envType',             'UNKNOWN', ...
        'confidence',          0.0, ...
        'directionConfidence', 0.0, ...
        'leftScore',           NaN, ...
        'frontScore',          NaN, ...
        'rightScore',          NaN, ...
        'lastDirection',       0, ...
        'lastEnvType',         'UNKNOWN', ...
        'failedAttempts',      0, ...
        'deadEndCount',        0);

end

% =====================================================================
% Local helper functions
% =====================================================================

function validatePoseIsFree(map, pose, safetyRadius)
%VALIDATEPOSEISFREE Ensure pose lies within the map bounds and that a
%ring of sample points at safetyRadius around it (plus the center) are
%all free space. Throws a clear error if not -- this is a one-time
%setup check, not a runtime robustness case, so failing loudly here is
%appropriate and preferable to silently starting the robot inside a wall.

    xLimits = map.XWorldLimits;
    yLimits = map.YWorldLimits;

    if pose(1) < xLimits(1) || pose(1) > xLimits(2) || ...
       pose(2) < yLimits(1) || pose(2) > yLimits(2)
        error('createRobot:PoseOutOfBounds', ...
            ['Initial pose [%.2f, %.2f] lies outside the map bounds ', ...
             'X:[%.2f, %.2f], Y:[%.2f, %.2f].'], ...
             pose(1), pose(2), xLimits(1), xLimits(2), yLimits(1), yLimits(2));
    end

    numRingSamples = 8;
    anglesToSample = linspace(0, 2*pi, numRingSamples + 1);
    anglesToSample(end) = []; % drop duplicate closing angle

    samplePoints = [pose(1), pose(2)]; % include the center itself
    for k = 1:numRingSamples
        candidate = [pose(1) + safetyRadius*cos(anglesToSample(k)), ...
                     pose(2) + safetyRadius*sin(anglesToSample(k))];
        % Only keep candidates that remain inside the map bounds; a
        % safety ring that pokes slightly outside the map at an edge
        % spawn point should not itself fail validation.
        if candidate(1) >= xLimits(1) && candidate(1) <= xLimits(2) && ...
           candidate(2) >= yLimits(1) && candidate(2) <= yLimits(2)
            samplePoints(end+1, :) = candidate; %#ok<AGROW> -- 9 rows max, negligible cost
        end
    end

    occupiedFlags = checkOccupancy(map, samplePoints);
    if any(occupiedFlags)
        error('createRobot:PoseNotFree', ...
            ['Initial pose [%.2f, %.2f, %.2f] is inside or too close to an ', ...
             'obstacle (safety radius %.2f m). Choose a different starting pose.'], ...
             pose(1), pose(2), pose(3), safetyRadius);
    end
end
