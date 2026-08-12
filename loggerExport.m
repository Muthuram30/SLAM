function loggerExport(logger, filename)
%LOGGEREXPORT Export the logger data to a MAT file.
%
%   LOGGEREXPORT(logger, filename)
%
%   Trims all arrays to the number of actually recorded steps, then
%   saves to the specified MAT file using v7.3 format for large data.
%
%   Inputs:
%       logger   - logger struct from loggerUpdate (with .count field).
%       filename - output file path (e.g. 'slam_simulation_log.mat').

    if nargin < 2
        filename = 'slam_simulation_log.mat';
    end

    n = logger.count;

    exportData.time                = logger.time(1:n);
    exportData.groundTruthPose     = logger.groundTruthPose(1:n, :);
    exportData.estimatedPose       = logger.estimatedPose(1:n, :);
    exportData.linearVelocity      = logger.linearVelocity(1:n);
    exportData.angularVelocity     = logger.angularVelocity(1:n);
    exportData.headingError        = logger.headingError(1:n);
    exportData.distanceToGoal      = logger.distanceToGoal(1:n);
    exportData.collisionCounter    = logger.collisionCounter(1:n);
    exportData.lidarScans          = logger.lidarScans(1:n);
    exportData.acceptedScan        = logger.acceptedScan(1:n);
    exportData.loopClosureDetected = logger.loopClosureDetected(1:n);
    exportData.localizationError   = logger.localizationError(1:n);

    save(filename, '-struct', 'exportData', '-v7.3');
    fprintf('Logger data exported to: %s (%d steps)\n', filename, n);
end
