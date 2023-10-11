function [E, F , G, slope, y_intercept] = compute_obstacle_constraints(x, detection, obstacle, lane_width, lanes)
ego_x = x(1);
ego_y = x(2);

% plot the constraint line (vehicle should ideally stay to the left part of
% the region marked by this line)
if detection
    slope = ((ego_y - obstacle.rlSafeY) / (ego_x - obstacle.rlSafeX));

    % if the vehcle is yet to encounter the obstacle
    if (ego_x <= obstacle.rlSafeX)
        % if ego_y > rlSafeY, this means that the obstacle has steered
        % enough to move to the adjacent lane
        if (ego_y > obstacle.rlSafeY)
            slope = 0;
            y_intercept = obstacle.rlSafeY;
        else
            slope = tan(atan2(slope,1));
            y_intercept = obstacle.rlSafeY - slope * obstacle.rlSafeX;
        end

    % if vehicle is parallel to obstacle
    elseif ((ego_x > obstacle.rlSafeX) && (ego_x <= obstacle.flX))
        slope = 0;
        y_intercept = obstacle.rlSafeY;

    % if vehicle has passed the obstacle
    else 
        slope = 0;
        y_intercept = -lane_width * lanes/2;
    end

else
    slope = 0;
    y_intercept = -lane_width * lanes/2;
end

% define the constraint matrices
E = [0 0; 0 0; 0 0];
F = [0 1 0 0; 0 -1 0 0; slope -1 0 0]; 
G = [lane_width * lanes/2; lane_width * lanes/2; -1 * y_intercept];
