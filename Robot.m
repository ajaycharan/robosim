classdef Robot < handle
    
    properties (Constant)
        length = 0.3
        d = 0.15
        color = lines(3)
    end
    
    properties (Access = public)
        id
        X = zeros(3,1)  % [x y theta]
        U = zeros(2,1)  % [v w]
        
        Xhist  % state history
        Thist  % time history
        
        leaders  % a list of learder robot
        controller = @(time, robot) robot.U
        control_params
    end  % properties public
    
    properties (Access = private)
        hgRobot  % graphic handle
        
    end  % properties private
    
    properties (Dependent = true)
        x      % x coordinate
        y      % y coordinate
        theta  % heading angle
        v      % velocity
        w      % angular velocity
    end
    
    methods
        
        function self = Robot(id, X)
            % Constructor of Robot class
            self.id = id;
            self.X = X(:);
        end
        
        function assignLeader(self, robots, des1, des2)
            % Assigns leaders (0 or 1 or 2) to 
            self.leaders = robots;
            nRobots = numel(robots);
            if nRobots == 1
                self.controller = @ControllerLP;
            elseif numel(robots) == 2
                self.controller = @ControllerLL;
            end
            self.control_params = [des1 des2];
        end
        
        function updateState(self, t, state)
            % updates state of the robot and append to history
            self.X = state;
            self.Xhist = [self.Xhist, self.X];
            self.Thist = [self.Thist, t];
        end
        
        function plot(self)
            % Plots robot
            nRobot = numel(self);
            for iRobot = 1:nRobot
                thisRobot = self(iRobot);
                xRobot = thisRobot.x;
                yRobot = thisRobot.y;
                thetaRobot = thisRobot.theta;
                lineX = [xRobot, xRobot + Robot.length * cos(thetaRobot)];
                lineY = [yRobot, yRobot + Robot.length * sin(thetaRobot)];
                if all(isempty(thisRobot.hgRobot))
                    holdIsOn = ishold;
                    if ~holdIsOn
                        hold('on')
                    end
                    
                    thisRobot.hgRobot(1) = ...
                        plot(xRobot, yRobot, 'o', ...
                        'Color', Robot.color(thisRobot.id,:));
                    thisRobot.hgRobot(2) = ...
                        plot(lineX, lineY, ...
                        'Color', Robot.color(thisRobot.id,:));
                    
                    if ~holdIsOn
                        hold('off')
                    end
                else
                    set(thisRobot.hgRobot(1), ...
                        'XData', xRobot, ...
                        'YData', yRobot);
                    set(thisRobot.hgRobot(2), ...
                        'XData', lineX, ...
                        'YData', lineY);
                end
            end
        end
        
        function x = get.x(self)
            x = self.X(1);
        end
            
        function y = get.y(self)
            y = self.X(2);
        end
        
        function theta = get.theta(self)
            theta = self.X(3);
        end
        
        function v = get.v(self)
            v = self.U(1);
        end
        
        function w = get.w(self)
            w = self.U(2);
        end
        
        function psi = getPsi(self, leader)
            % self is robot 1, assumed to be leader
            
            p1 = leader.X(1:2,:);
            th1 = leader.theta;
            
            % add on length of robot for p2
            p2 = self.X(1:2,:);
            th2 = self.theta;
            p2 = p2 + [cos(th2) -sin(th2); sin(th2) cos(th2)] * [self.d; 0];
            
            % vector of psi
            v = p2 - p1;
            beta = atan2(v(2),v(1));
            psi = beta - th1;
        end
        
        function L = getLength(self, leader)
            p1 = leader.X(1:2,:);
            p2 = self.X(1:2,:);
            L = norm(p1 - p2);
        end
        
    end  % methods public
    
end  % classdef Robot
