function [slamState, slamStats] = updateSLAM(slamState, scan, groundTruthPose)
%UPDATESLAM Add a scan to the lidarSLAM pipeline and update state.
%
%   [slamState, slamStats] = UPDATESLAM(slamState, scan, groundTruthPose)
%
%   Gracefully handles every runtime condition:
%       - Empty or invalid scans (returns defaults, no crash)
%       - Scan rejection by lidarSLAM (increments rejectedCount)
%       - Missing loop closures (loopClosureDetected stays false)
%       - SLAM internal errors (caught, warned, execution continues)
%       - First-scan alignment (sets SLAM->world coordinate transform)
%
%   Inputs:
%       slamState      - struct from initializeSLAM / previous updateSLAM.
%       scan           - lidarScan object (may be empty).
%       groundTruthPose - 1x3 [x y theta] for localization error.
%
%   Outputs:
%       slamState - updated state struct.
%       slamStats - struct with per-step SLAM statistics:
%                     .isAccepted, .loopClosureDetected,
%                     .estimatedPose, .localizationError,
%                     .totalAccepted, .totalRejected,
%                     .totalLoopClosures

    % Default outputs: safe values when no SLAM update occurs.
    slamStats.isAccepted           = false;
    slamStats.loopClosureDetected  = false;
    slamStats.estimatedPose        = slamState.estimatedPose;
    slamStats.localizationError    = NaN;
    slamStats.totalAccepted        = slamState.acceptedCount;
    slamStats.totalRejected        = slamState.rejectedCount;
    slamStats.totalLoopClosures    = slamState.loopClosureCount;

    % Guard: empty or invalid scan.
    if isempty(scan)
        return;
    end

    try
        slamState.scanCount = slamState.scanCount + 1;

        % Attempt to add the scan to the SLAM object.
        % Returns (per MATLAB docs):
        %   isAccepted      – logical
        %   loopClosureInfo – struct with EdgeIDs, Edges, Scores
        %   optimInfo       – struct with IsPerformed, IsAccepted, etc.
        [isAccepted, loopClosureInfo, ~] = addScan(slamState.slamObj, scan);
        slamStats.isAccepted = isAccepted;

        if isAccepted
            slamState.acceptedCount = slamState.acceptedCount + 1;

            % Set alignment transform on the very first accepted scan.
            % SLAM always starts at [0, 0, 0]; this maps it to the
            % robot's actual world pose at that moment.
            if ~slamState.isAligned
                slamState.alignmentPose = groundTruthPose;
                slamState.isAligned = true;
            end

            % Retrieve the latest optimized pose and transform to world frame.
            [~, poses] = scansAndPoses(slamState.slamObj);
            if ~isempty(poses)
                rawPose = poses(end, :);
                worldPose = transformToWorld(rawPose, slamState.alignmentPose);
                slamState.estimatedPose = worldPose;
                slamStats.estimatedPose = worldPose;
            end

            % Check for loop closure detection.
            % Per MATLAB docs, loopClosureInfo is a struct with field
            % EdgeIDs (vector of newly connected edge IDs).  A non-empty
            % EdgeIDs vector means loop closure(s) were found.
            % Count actual number of edges, not just +1.
            if isstruct(loopClosureInfo) && isfield(loopClosureInfo, 'EdgeIDs')
                numNewClosures = numel(loopClosureInfo.EdgeIDs);
                if numNewClosures > 0
                    slamState.loopClosureCount = slamState.loopClosureCount + numNewClosures;
                    slamStats.loopClosureDetected = true;
                end
            end
        else
            slamState.rejectedCount = slamState.rejectedCount + 1;
        end

        % Compute localization error (position component only).
        if slamState.isAligned
            slamStats.localizationError = hypot( ...
                slamState.estimatedPose(1) - groundTruthPose(1), ...
                slamState.estimatedPose(2) - groundTruthPose(2));
        end

    catch ME
        warning('updateSLAM:RuntimeError', 'SLAM update failed: %s', ME.message);
    end

    % Ensure totals reflect the latest counts.
    slamStats.totalAccepted     = slamState.acceptedCount;
    slamStats.totalRejected     = slamState.rejectedCount;
    slamStats.totalLoopClosures = slamState.loopClosureCount;
end

% =====================================================================
% Local helper
% =====================================================================

function worldPose = transformToWorld(slamPose, alignmentPose)
%TRANSFORMTOWORLD Apply the SLAM->world rigid transform to a single pose.
    ct = cos(alignmentPose(3));
    st = sin(alignmentPose(3));
    worldPose = zeros(1, 3);
    worldPose(1) = ct * slamPose(1) - st * slamPose(2) + alignmentPose(1);
    worldPose(2) = st * slamPose(1) + ct * slamPose(2) + alignmentPose(2);
    worldPose(3) = atan2(sin(slamPose(3) + alignmentPose(3)), ...
                         cos(slamPose(3) + alignmentPose(3)));
end
