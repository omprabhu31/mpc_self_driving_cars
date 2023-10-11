function detection = detect_obstacle(x, obstacle, lane_width)
% function to detect when the vehicle sees an obstacle
ego_x = x(1);
ego_y = x(2);

% find out if the vehicle is within the obstacle detection distance
dist_to_obstacle = sqrt( (obstacle.X - ego_x)^2 + (obstacle.Y - ego_y)^2 );
flag_close_enough = (dist_to_obstacle < obstacle.DetectionDistance);
flag_in_lane = (abs(obstacle.Y - ego_y) < 2*lane_width);

detection = (flag_close_enough && (ego_x < obstacle.frSafeX) && flag_in_lane);