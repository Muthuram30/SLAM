classdef helperFunctions
%HELPERFUNCTIONS Shared utility functions for the 2D SLAM simulation.
%
%   All methods are static. Call as: helperFunctions.methodName(args)
%
%   Example:
%       wrapped = helperFunctions.wrapAngleToPi(3.5);
%       [posErr, hdgErr] = helperFunctions.computeLocalizationError(gt, est);

    methods (Static)

        function wrapped = wrapAngleToPi(angle)
        %WRAPANGLETOPI Wrap angle(s) in radians to the interval (-pi, pi].
            wrapped = atan2(sin(angle), cos(angle));
        end

        function [posError, headingError] = computeLocalizationError(gtPose, estPose)
        %COMPUTELOCALIZATIONERROR Euclidean position and absolute heading errors.
        %   gtPose  - 1x3 [x y theta] ground truth
        %   estPose - 1x3 [x y theta] estimated
            posError = hypot(gtPose(1) - estPose(1), gtPose(2) - estPose(2));
            headingError = abs(helperFunctions.wrapAngleToPi(gtPose(3) - estPose(3)));
        end

        function worldPoses = transformPosesToWorld(slamPoses, alignmentPose)
        %TRANSFORMPOSESTOWORLD Apply rigid transform from SLAM frame to world.
        %   slamPoses     - Nx3 poses in SLAM frame
        %   alignmentPose - 1x3 [x y theta] world pose at first SLAM scan
            ct = cos(alignmentPose(3));
            st = sin(alignmentPose(3));
            n = size(slamPoses, 1);
            worldPoses = zeros(n, 3);
            worldPoses(:,1) = ct * slamPoses(:,1) - st * slamPoses(:,2) + alignmentPose(1);
            worldPoses(:,2) = st * slamPoses(:,1) + ct * slamPoses(:,2) + alignmentPose(2);
            worldPoses(:,3) = atan2(sin(slamPoses(:,3) + alignmentPose(3)), ...
                                    cos(slamPoses(:,3) + alignmentPose(3)));
        end

        function [row, col] = worldToGrid(pointXY, numRows, resolution)
        %WORLDTOGRID Convert world (x, y) to grid (row, col) indices.
        %   Follows binaryOccupancyMap convention: row 1 = max y (top).
            col = round(pointXY(1) * resolution) + 1;
            row = numRows - round(pointXY(2) * resolution);
        end

        function str = formatPose(pose)
        %FORMATPOSE Format [x y theta] as a compact readable string.
            str = sprintf('[%.2f, %.2f, %.1f deg]', ...
                pose(1), pose(2), rad2deg(pose(3)));
        end

        function bytes = getMemoryUsage()
        %GETMEMORYUSAGE Approximate current MATLAB memory usage in bytes.
        %   Returns NaN if unavailable (e.g. MATLAB Online on some platforms).
            try
                memInfo = memory;
                bytes = memInfo.MemUsedMATLAB;
            catch
                bytes = NaN;
            end
        end

    end
end
