classdef Robot < handle
    
    properties (Constant)
        length = 2
        color = lines(3)
    end
    
    properties (Access = public)
        id
        X = zeros(3,1)  % [x y theta]
        U = zeros(2,1)  % [v w]
        
        Xhist  % state history
        Thist  % time history
        
        leaders  % a list of learder robot
        controller = @(robot)robot.U
    end  % properties public
    
    properties (Access = private)
        hgRobot  % graphic handle
        
    end  % properties private
    
    properties (Dependent = true)
        x      % x coordinate
        y      % y coordinate
        theta  % heading angle
    end
    
    methods
        
        function self = Robot(id, X)
            % Constructor of Robot class
            self.id = id;
            self.X = X(:);
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
        
    end  % methods public
    
end  % classdef Robot
