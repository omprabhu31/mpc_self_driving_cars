function f = initialize_plot(x_0 ,obstacle, lane_width, lanes)
f = figure;

% plot the vehicle
vehicle_length = 5;
vehicle_width = 2;
X_0 = x_0(1);
Y_0 = x_0(2);
plot(X_0,Y_0,'gx'); hold on; grid on;
rectangle('Position', [X_0 - vehicle_length/2, Y_0 - vehicle_width/2, vehicle_length, vehicle_width]);

% plot the obstacle and safe zone
rectangle('Position',[obstacle.rrX, obstacle.rrY, obstacle.Length, obstacle.Width]);
rectangle('Position',[obstacle.rrSafeX, obstacle.rrSafeY, (obstacle.safeDistanceX) * 2,(obstacle.safeDistanceY) *2 ],'LineStyle','--','EdgeColor', 'r');

% plot road lanes
road_x = [0;50;100];
road_y = [2;2;2];
line(road_x, road_y, 'LineStyle', '--', 'Color', 'b');

road_x = [0;50;100];
road_y = [-2;-2;-2];
line(road_x, road_y, 'LineStyle', '--', 'Color', 'b');

axis([0 100 -lane_width*lanes/2 lane_width*lanes/2]);
xlabel('X');
ylabel('Y');