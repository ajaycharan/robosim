function ControllerLL(time, robot)
% CONTOLLERLL l-l controller for leader follower robot
assert(numel(robot.leaders) == 2, 'Invalid number of leaders, should be 2');

robot1 = robot.leaders(1);
robot2 = robot.leaders(2);

robot.U = [v; w];

end
