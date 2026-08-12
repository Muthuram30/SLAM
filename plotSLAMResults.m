function plotSLAMResults(logger, slamState, groundTruthMap, slamMap, config, perceptionResults) %#ok<INUSD>
%PLOTSLAMRESULTS Generate the final comprehensive SLAM results figure.
%
%   PLOTSLAMRESULTS(logger, slamState, groundTruthMap, slamMap, config)
%   PLOTSLAMRESULTS(logger, slamState, groundTruthMap, slamMap, config, perceptionResults)
%
%   Creates a multi-panel figure (3x3) showing:
%       1. Trajectory comparison (ground truth vs. estimated)
%       2. Final SLAM occupancy map
%       3. Pose graph (optimized node positions)
%       4. Localization error over time
%       5. Velocity profiles
%       6. Simulation statistics summary
%       7. Wall segments and openings (when perceptionResults provided)
%
%   All plots use persistent axes handles; nothing is deleted/recreated.

    if nargin < 6
        perceptionResults = [];
    end

    n = logger.count;
    if n < 1
        warning('plotSLAMResults:NoData', 'No logged data to display.');
        return;
    end

    % -----------------------------------------------------------------
    % Trim logged data.
    % -----------------------------------------------------------------
    t            = logger.time(1:n);
    gtPose       = logger.groundTruthPose(1:n, :);
    estPose      = logger.estimatedPose(1:n, :);
    locError     = logger.localizationError(1:n);
    linVel       = logger.linearVelocity(1:n);
    angVel       = logger.angularVelocity(1:n);

    % Compute aggregate statistics.
    validErr    = locError(~isnan(locError));
    if isempty(validErr)
        rmseError = NaN; maxError = NaN; meanError = NaN;
    else
        rmseError = sqrt(mean(validErr.^2));
        maxError  = max(validErr);
        meanError = mean(validErr);
    end
    totalDist = sum(sqrt(sum(diff(gtPose(:,1:2)).^2, 2)));

    % =================================================================
    %  Create the results figure.
    % =================================================================
    fig = figure('Name', 'SLAM Results', 'NumberTitle', 'off', ...
        'Position', [50, 50, 1400, 1000]);

    % Determine grid layout: 3x3 if perception data, 2x3 otherwise.
    if ~isempty(perceptionResults) && ...
            size(perceptionResults.wallSegments, 1) > 0
        gridRows = 3;
    else
        gridRows = 2;
    end
    gridCols = 3;

    % -----------------------------------------------------------------
    % Panel 1: Trajectory Comparison
    % -----------------------------------------------------------------
    ax1 = subplot(gridRows, gridCols, 1, 'Parent', fig);
    show(groundTruthMap, 'Parent', ax1);
    hold(ax1, 'on');
    plot(ax1, gtPose(:,1), gtPose(:,2), '-', 'Color', [0 0.45 0.74], ...
        'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
    validEst = ~isnan(estPose(:,1));
    if any(validEst)
        plot(ax1, estPose(validEst,1), estPose(validEst,2), '--', ...
            'Color', [0.85 0.33 0.10], 'LineWidth', 1.5, ...
            'DisplayName', 'SLAM Estimated');
    end
    plot(ax1, gtPose(1,1), gtPose(1,2), 'gs', 'MarkerSize', 10, ...
        'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot(ax1, gtPose(end,1), gtPose(end,2), 'r^', 'MarkerSize', 10, ...
        'MarkerFaceColor', 'r', 'DisplayName', 'End');
    legend(ax1, 'Location', 'best', 'FontSize', 7);
    title(ax1, 'Trajectory Comparison');
    hold(ax1, 'off');

    % -----------------------------------------------------------------
    % Panel 2: SLAM Occupancy Map
    % -----------------------------------------------------------------
    ax2 = subplot(gridRows, gridCols, 2, 'Parent', fig);
    show(slamMap, 'Parent', ax2);
    title(ax2, 'SLAM-Built Occupancy Map');

    % -----------------------------------------------------------------
    % Panel 3: Pose Graph
    % -----------------------------------------------------------------
    ax3 = subplot(gridRows, gridCols, 3, 'Parent', fig);
    if slamState.acceptedCount > 0
        try
            [~, poses] = scansAndPoses(slamState.slamObj);
            if slamState.isAligned
                poses = helperFunctions.transformPosesToWorld( ...
                    poses, slamState.alignmentPose);
            end
            hold(ax3, 'on');
            plot(ax3, poses(:,1), poses(:,2), 'b-', 'LineWidth', 1, ...
                'DisplayName', 'Scan path');
            plot(ax3, poses(:,1), poses(:,2), 'b.', 'MarkerSize', 8, ...
                'HandleVisibility', 'off');
            plot(ax3, poses(1,1), poses(1,2), 'gs', 'MarkerSize', 10, ...
                'MarkerFaceColor', 'g', 'DisplayName', 'Start');
            plot(ax3, poses(end,1), poses(end,2), 'r^', 'MarkerSize', 10, ...
                'MarkerFaceColor', 'r', 'DisplayName', 'End');
            hold(ax3, 'off');
            legend(ax3, 'Location', 'best', 'FontSize', 7);
        catch
            text(ax3, 0.5, 0.5, 'Pose graph unavailable', ...
                'HorizontalAlignment', 'center');
        end
    else
        text(ax3, 0.5, 0.5, 'No accepted scans', ...
            'HorizontalAlignment', 'center');
    end
    axis(ax3, 'equal');
    grid(ax3, 'on');
    title(ax3, sprintf('Pose Graph (%d nodes)', slamState.acceptedCount));

    % -----------------------------------------------------------------
    % Panel 4: Localization Error over Time
    % -----------------------------------------------------------------
    ax4 = subplot(gridRows, gridCols, 4, 'Parent', fig);
    if ~isempty(validErr)
        validIdx = find(~isnan(locError));
        plot(ax4, t(validIdx), locError(validIdx), '-', ...
            'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
        hold(ax4, 'on');
        yline(ax4, rmseError, '--r', sprintf('RMSE = %.4f m', rmseError), ...
            'LineWidth', 1, 'LabelHorizontalAlignment', 'left', 'FontSize', 7);
        yline(ax4, meanError, '--b', sprintf('Mean = %.4f m', meanError), ...
            'LineWidth', 1, 'LabelHorizontalAlignment', 'left', 'FontSize', 7);
        hold(ax4, 'off');
    end
    xlabel(ax4, 'Time (s)');
    ylabel(ax4, 'Position Error (m)');
    title(ax4, 'Localization Error vs. Time');
    grid(ax4, 'on');

    % -----------------------------------------------------------------
    % Panel 5: Velocity Profiles
    % -----------------------------------------------------------------
    ax5 = subplot(gridRows, gridCols, 5, 'Parent', fig);
    yyaxis(ax5, 'left');
    plot(ax5, t, linVel, '-', 'Color', [0 0.45 0.74], 'LineWidth', 1);
    ylabel(ax5, 'Linear Vel (m/s)');
    yyaxis(ax5, 'right');
    plot(ax5, t, angVel, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1);
    ylabel(ax5, 'Angular Vel (rad/s)');
    xlabel(ax5, 'Time (s)');
    title(ax5, 'Velocity Profiles');
    grid(ax5, 'on');

    % -----------------------------------------------------------------
    % Panel 6: Statistics Summary
    % -----------------------------------------------------------------
    ax6 = subplot(gridRows, gridCols, 6, 'Parent', fig);
    axis(ax6, 'off');

    memBytes = helperFunctions.getMemoryUsage();
    if isnan(memBytes)
        memStr = 'N/A';
    else
        memStr = sprintf('%.1f MB', memBytes / 1e6);
    end

    statsText = {
        sprintf('Simulation Time:     %.1f s', t(end))
        sprintf('Total Distance:      %.2f m', totalDist)
        sprintf('Total Scans:         %d', slamState.scanCount)
        sprintf('Accepted Scans:      %d', slamState.acceptedCount)
        sprintf('Rejected Scans:      %d', slamState.rejectedCount)
        sprintf('Loop Closures:       %d', slamState.loopClosureCount)
        ''
        sprintf('RMSE:                %.4f m', rmseError)
        sprintf('Max Error:           %.4f m', maxError)
        sprintf('Mean Error:          %.4f m', meanError)
        ''
        sprintf('GT Final Pose:       %s', helperFunctions.formatPose(gtPose(end,:)))
        sprintf('Est Final Pose:      %s', helperFunctions.formatPose(slamState.estimatedPose))
        ''
        sprintf('Map Size:            %dx%d', size(getOccupancy(slamMap)))
        sprintf('Memory Usage:        %s', memStr)
    };

    % Add perception stats if available.
    if ~isempty(perceptionResults)
        statsText{end+1} = '';
        statsText{end+1} = sprintf('Wall Segments:       %d', size(perceptionResults.wallSegments, 1));
        statsText{end+1} = sprintf('Openings Detected:   %d', numel(perceptionResults.openings));
        statsText{end+1} = sprintf('Filtered Cells:      %d', perceptionResults.filteredCellCount);
    end

    text(ax6, 0.05, 0.95, statsText, ...
        'FontName', 'FixedWidth', 'FontSize', 9, ...
        'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');
    title(ax6, 'Simulation Statistics');

    % -----------------------------------------------------------------
    % Panel 7: Wall Segments and Openings (NEW, perception layer)
    % -----------------------------------------------------------------
    if gridRows == 3 && ~isempty(perceptionResults)
        ax7 = subplot(gridRows, gridCols, 7, 'Parent', fig);
        show(slamMap, 'Parent', ax7);
        hold(ax7, 'on');

        wallSegs = perceptionResults.wallSegments;
        numSegs = size(wallSegs, 1);

        % Draw each wall segment as a thick colored line.
        segColors = lines(max(numSegs, 1));
        for si = 1:numSegs
            plot(ax7, [wallSegs(si,1), wallSegs(si,3)], ...
                [wallSegs(si,2), wallSegs(si,4)], ...
                '-', 'Color', segColors(si,:), 'LineWidth', 2.5);
            % Mark endpoints.
            plot(ax7, wallSegs(si,[1 3]), wallSegs(si,[2 4]), 'o', ...
                'Color', segColors(si,:), 'MarkerSize', 5, ...
                'MarkerFaceColor', segColors(si,:));
        end

        % Draw openings as green rectangles / markers.
        numOpenings = numel(perceptionResults.openings);
        for oi = 1:numOpenings
            op = perceptionResults.openings(oi);
            % Mark opening center.
            plot(ax7, op.center(1), op.center(2), 'p', ...
                'Color', [0 0.8 0.2], 'MarkerSize', 14, ...
                'MarkerFaceColor', [0 0.8 0.2], 'LineWidth', 1.5);
            % Draw gap line between inner endpoints.
            plot(ax7, [op.endpoint1(1), op.endpoint2(1)], ...
                [op.endpoint1(2), op.endpoint2(2)], '--', ...
                'Color', [0 0.8 0.2], 'LineWidth', 2);
            % Label.
            text(ax7, op.center(1) + 0.2, op.center(2) + 0.2, ...
                sprintf('%.1fm', op.width), 'Color', [0 0.7 0.1], ...
                'FontSize', 8, 'FontWeight', 'bold');
        end

        hold(ax7, 'off');
        title(ax7, sprintf('Wall Segments (%d) & Openings (%d)', numSegs, numOpenings));
        axis(ax7, 'equal');
    end
end
