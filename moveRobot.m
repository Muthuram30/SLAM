function [robot, controllerState, isPathComplete] = moveRobot(robot, waypoints, controllerState, dt, waypointTolerance, map, collisionThreshold)
%MOVEROBOT Advance the robot one time step along a list of waypoints.
%
%   Thin backward-compatible wrapper around simulateRobotStep.
%
%   See also: simulateRobotStep

    if nargin < 7
        collisionThreshold = 50;
    end

    [robot, controllerState, isPathComplete] = simulateRobotStep( ...
        robot, waypoints, controllerState, dt, waypointTolerance, ...
        map, collisionThreshold);
end
