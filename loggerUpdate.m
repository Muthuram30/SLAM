function logger = loggerUpdate(logger, time, robot, slamStats, scan)
%LOGGERUPDATE Record one simulation step into the preallocated logger.
%
%   logger = LOGGERUPDATE(logger, time, robot, slamStats, scan)
%
%   Inputs:
%       logger    - logger struct from loggerInit / previous loggerUpdate.
%       time      - simulation time for this step (seconds).
%       robot     - robot struct with pose, velocities, and diagnostics.
%       slamStats - struct from updateSLAM (may be empty before SLAM runs).
%       scan      - lidarScan object (may be empty before LiDAR is active).
%
%   Output:
%       logger - updated logger struct with count incremented.

    logger.count = logger.count + 1;
    idx = logger.count;

    % Guard against overflow.
    if idx > logger.maxSteps
        warning('loggerUpdate:Overflow', ...
            'Logger capacity (%d) exceeded; step not recorded.', ...
            logger.maxSteps);
        logger.count = logger.maxSteps;
        return;
    end

    % Core robot state.
    logger.time(idx)               = time;
    logger.groundTruthPose(idx, :) = robot.pose;
    logger.linearVelocity(idx)     = robot.linearVel;
    logger.angularVelocity(idx)    = robot.angularVel;
    logger.headingError(idx)       = robot.headingError;
    logger.distanceToGoal(idx)     = robot.distanceToGoal;
    logger.collisionCounter(idx)   = robot.collisionCounter;

    % LiDAR scan (cell array element).
    if ~isempty(scan)
        logger.lidarScans{idx} = scan;
    end

    % SLAM statistics (safe when slamStats is empty or incomplete).
    if ~isempty(slamStats) && isstruct(slamStats)
        if isfield(slamStats, 'estimatedPose')
            logger.estimatedPose(idx, :) = slamStats.estimatedPose;
        end
        if isfield(slamStats, 'isAccepted')
            logger.acceptedScan(idx) = slamStats.isAccepted;
        end
        if isfield(slamStats, 'loopClosureDetected')
            logger.loopClosureDetected(idx) = slamStats.loopClosureDetected;
        end
        if isfield(slamStats, 'localizationError')
            logger.localizationError(idx) = slamStats.localizationError;
        end
    end
end
