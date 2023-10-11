function obstacle = define_obstacle(obstacle)

% obstacle geometry
obstacle.flX = obstacle.X + obstacle.Length/2;
obstacle.flY = obstacle.Y + obstacle.Width/2;

obstacle.frX = obstacle.X + obstacle.Length/2;
obstacle.frY = obstacle.Y - obstacle.Width/2;

obstacle.rlX = obstacle.X - obstacle.Length/2;
obstacle.rlY = obstacle.flY;

obstacle.rrX = obstacle.X - obstacle.Length/2;
obstacle.rrY = obstacle.frY;

% safe zone geometry
obstacle.flSafeX = obstacle.X + obstacle.safeDistanceX; 
obstacle.flSafeY = obstacle.Y + obstacle.safeDistanceY;

obstacle.frSafeX = obstacle.X + obstacle.safeDistanceX;
obstacle.frSafeY = obstacle.Y - obstacle.safeDistanceY;

obstacle.rlSafeX = obstacle.X - obstacle.safeDistanceX; 
obstacle.rlSafeY = obstacle.flSafeY;

obstacle.rrSafeX = obstacle.X - obstacle.safeDistanceX;
obstacle.rrSafeY = obstacle.frSafeY;
