function ControllerLP(~, robot)
%% CONTROLLERLP - Length-Psi controller for non-holonomic robot

if numel(robot.leaders)~=1
    error('LP controller requires only 1 leader');
end

% leader state
leader = robot.leaders(1);
x1 = leader.x;
y1 = leader.y;
th1 = leader.theta;

% current state
x2 = robot.x;
y2 = robot.y;
th2 = robot.theta;

% controller variables
L12 = sqrt((x1-x2)^2 + (y1-y2)^2);
P12 = 

end
