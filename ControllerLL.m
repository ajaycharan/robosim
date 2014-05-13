function ControllerLL(time, robot)
% CONTOLLERLL l-l controller for leader follower robot
assert(numel(robot.leaders) == 2, 'Invalid number of leaders, should be 2');
a1 = 1;
a2 = 1;
d = Robot.d;

% desired l
l13_d = robot.control_params(1);
l23_d = robot.control_params(2);

l13 = robot.getLength(leader(1));
l23 = robot.getLenght(leader(2));
psi13 = robot.getPsi(leader(1));
psi23 = robot.getPsi(leader(2));
r1 = leader(1).theta + psi13 - robot.theta;
r2 = leader(2).theta + psi23 - robot.theta;
v1 = leader(1).v;
v2 = leader(2).v;

w3 = 1/(d*sin(r1 - r2)) * ...
    (a1(l13_d - l13)*cos(r2) + v1 * cos(psi13) * cos(r2) - ...
     a2*(l23_d - l23)*cos(r1) - v2 * cos(psi23) * cos(r1));

v3 = (a1(l13_d - l13) + v1 * cos(psi13) - d * w3 * sin(r1)) / cos(r1);

robot.U = [v3; w3];

end
