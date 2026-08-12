function logger = loggerUpdatePerception(logger, wallSegCount, openingCount, filteredCellCount)
%LOGGERUPDATEPERCEPTION Record perception layer metrics into the logger.
%
%   logger = LOGGERUPDATEPERCEPTION(logger, wallSegCount, openingCount, filteredCellCount)
%
%   Called from main.m each time the SLAM map is rebuilt (not every step).
%   Records the latest perception layer results at the current logger
%   index, and fills any gap since the last update with the previous
%   values so time-series plotting works correctly.
%
%   Inputs:
%       logger           - logger struct from loggerInit / loggerUpdate.
%       wallSegCount     - number of wall segments detected.
%       openingCount     - number of openings detected.
%       filteredCellCount - number of map cells suppressed by filtering.
%
%   Output:
%       logger - updated logger struct.

    idx = logger.count;
    if idx < 1 || idx > logger.maxSteps
        return;
    end

    % Fill from last perception update to current index with the new values.
    startIdx = max(1, logger.lastPerceptionIdx + 1);
    endIdx   = idx;

    logger.wallSegmentCount(startIdx:endIdx)  = wallSegCount;
    logger.openingCount(startIdx:endIdx)       = openingCount;
    logger.filteredCellCount(startIdx:endIdx)  = filteredCellCount;
    logger.lastPerceptionIdx = idx;
end
