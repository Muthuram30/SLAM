function logger = loggerInit(maxSteps)
%LOGGERINIT Initialize the simulation logger with preallocated arrays.
%
%   logger = LOGGERINIT(maxSteps) creates a struct containing
%   preallocated arrays for every field logged during the simulation.
%   Pass this struct to loggerUpdate each step and to loggerExport
%   at the end.
%
%   Input:
%       maxSteps - maximum number of simulation steps to accommodate.
%
%   Output:
%       logger - struct with preallocated storage and a .count field
%                tracking how many entries have been recorded.

    validateattributes(maxSteps, {'double'}, {'scalar', 'positive', 'integer'});

    logger.maxSteps             = maxSteps;
    logger.count                = 0;

    % Per-step arrays (preallocated, never grown inside the loop).
    logger.time                 = zeros(maxSteps, 1);
    logger.groundTruthPose      = zeros(maxSteps, 3);
    logger.estimatedPose        = nan(maxSteps, 3);
    logger.linearVelocity       = zeros(maxSteps, 1);
    logger.angularVelocity      = zeros(maxSteps, 1);
    logger.headingError         = nan(maxSteps, 1);
    logger.distanceToGoal       = nan(maxSteps, 1);
    logger.collisionCounter     = zeros(maxSteps, 1);
    logger.lidarScans           = cell(maxSteps, 1);
    logger.acceptedScan         = false(maxSteps, 1);
    logger.loopClosureDetected  = false(maxSteps, 1);
    logger.localizationError    = nan(maxSteps, 1);

    % Perception layer metrics (updated per map rebuild, not per step).
    logger.wallSegmentCount     = zeros(maxSteps, 1);
    logger.openingCount         = zeros(maxSteps, 1);
    logger.filteredCellCount    = zeros(maxSteps, 1);
    logger.lastPerceptionIdx    = 0;   % index of last perception update
end
