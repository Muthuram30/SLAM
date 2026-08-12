function slamState = initializeSLAM(slamConfig)
%INITIALIZESLAM Create and configure a lidarSLAM object and bookkeeping.
%
%   slamState = INITIALIZESLAM(slamConfig)
%
%   Inputs:
%       slamConfig - struct with fields:
%                      .mapResolution           (cells per meter)
%                      .maxRange                (meters)
%                      .loopClosureThreshold    (score threshold)
%                      .loopClosureSearchRadius (meters)
%                      .movementThreshold       (optional) [translation rotation]
%                      .optimizationInterval    (optional) positive integer
%
%   Output:
%       slamState - struct containing the configured lidarSLAM object,
%                   counters, alignment transform, and state flags.
%                   Also exposes .movementThreshold so the simulation
%                   loop can gate scan acquisition using the same value.
%
%   The alignment transform (set on the first accepted scan) maps
%   from the SLAM frame (which starts at [0, 0, 0]) to the world
%   frame, enabling fair comparison with ground truth poses.

    % Create the lidarSLAM object (Navigation Toolbox).
    slamObj = lidarSLAM(slamConfig.mapResolution, slamConfig.maxRange);

    % --- Configure SLAM parameters ---

    % LoopClosureThreshold: score threshold for accepting loop closures.
    %   Higher = stricter.  Default 100.
    slamObj.LoopClosureThreshold    = slamConfig.loopClosureThreshold;

    % LoopClosureSearchRadius: radius (m) to search for loop closures.
    %   Default 8 m.
    slamObj.LoopClosureSearchRadius = slamConfig.loopClosureSearchRadius;

    % MovementThreshold: [translation(m) rotation(rad)].
    %   Default [0 0] → accepts every scan regardless of motion.
    %   For a slow indoor robot (dt=0.05 s, ~0.3 m/s), accepting one
    %   scan per ~0.5 m of translation or ~15° of rotation avoids
    %   oversampling while preserving map density.
    if isfield(slamConfig, 'movementThreshold')
        mt = slamConfig.movementThreshold;
    else
        mt = [0.5, deg2rad(15)];   % sensible indoor default
    end
    slamObj.MovementThreshold = mt;

    % OptimizationInterval: number of *loop closures* accepted before
    %   triggering a pose-graph optimization.  Default 1 (every closure).
    %   Setting it to 3 amortises CPU cost while still responding to
    %   loop closures promptly.
    if isfield(slamConfig, 'optimizationInterval')
        oi = slamConfig.optimizationInterval;
    else
        oi = 3;
    end
    slamObj.OptimizationInterval = oi;

    % --- Assemble the state struct ---
    slamState.slamObj           = slamObj;
    slamState.scanCount         = 0;
    slamState.acceptedCount     = 0;
    slamState.rejectedCount     = 0;
    slamState.loopClosureCount  = 0;
    slamState.estimatedPose     = [0, 0, 0];
    slamState.isInitialized     = true;

    % Expose the translation component of MovementThreshold so the
    % simulation loop can use the same value for pre-gating LiDAR.
    slamState.movementThreshold = mt(1);

    % Alignment: maps SLAM frame [0,0,0] -> world frame on first acceptance.
    slamState.isAligned         = false;
    slamState.alignmentPose     = [0, 0, 0];

    fprintf(['SLAM initialized: res=%d cells/m, maxRange=%.1f m, ', ...
        'LCT=%d, LCSR=%.1f m, MT=[%.2f m, %.1f deg], OI=%d\n'], ...
        slamConfig.mapResolution, slamConfig.maxRange, ...
        slamConfig.loopClosureThreshold, slamConfig.loopClosureSearchRadius, ...
        mt(1), rad2deg(mt(2)), oi);
end
