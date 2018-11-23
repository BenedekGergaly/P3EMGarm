function y = TrajectoryGeneration(inputs)
%TRAJECTORYGENERATION Summary of this function goes here
%   Detailed explanation goes here

thetaDesired = [inputs(1); inputs(2); inputs(3)];
timer = inputs(4);

tb = [0 0 0];

    function TrajectoryMatrix = SetNewGoal(currentAngles, goalAngles, goalAccelerations, desiredTime)
        startTime = timer;
        localGoalAccelerations = goalAccelerations;
        for i = 1:3
            if goalAngles(i) - startAngles(i) < 0 && goalAccelerations(i) > 0
                localGoalAccelerations(i) = -localGoalAccelerations(i);
            end
            
            tb(i) = 0.5 * desiredTime - abs((sqrt(localGoalAccelerations(1)^2) * desiredTime^2 ...
                    - (4 * localGoalAccelerations(i) * (goalAngles(i) - startAngles(i)))) / (2 * localGoalAccelerations(i)));
        end
    end

end

