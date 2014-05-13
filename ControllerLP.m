function ControllerLP(~, robot)
%% CONTROLLERLP - Length-Psi controller for non-holonomic robot

if numel(robot.leaders)~=1
    error('LP controller requires only 1 leader');
end

% controller gains
a1 = 1;
a2 = 1;

% leader state
leader = robot.leaders(1);
x1 = leader.x;
y1 = leader.y;
th1 = leader.theta;
v1 = leader.v;
w1 = leader.w;

% current state
x2 = robot.x;
y2 = robot.y;
th2 = robot.theta;

% system outputs
L12 = sqrt((x1-x2)^2 + (y1-y2)^2);
P12 = robot.getPsi(leader);

% desired outputs
L12_des = robot.control_params(1);
P12_des = robot.control_params(2);

% length of robot
d = robot.d;

% gamma
g1 = th1 + P12 - th2;

% rho
R12 = (a1*(L12_des - L12) + v1*cos(P12)) / cos(g1);

% calculate outputs
w2 = cos(g1) * ( a2*L12*(P12_des - P12) - v1*sin(P12) + L12*w1 + R12*sin(g1)) / d;

end
