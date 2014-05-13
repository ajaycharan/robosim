classdef Simulator < handle

    properties
        t_step = 0.01;
        robots = Robot.empty();
    end
    
    properties(Access=private)
        hFigure
    end
    
    methods
        function self = Simulator(t_range)
            hFigure = figure;
            
            
        end
        
        function simulate(self)
            
            
            
        end
        
        function step(self)
            
            % ODE options
            options = odeset('RelTol', 1e-2, 'AbsTol', 1e-4);
            
            for i=1:numel(self.robots)
                
                X = self.robots(i).X;
                U = self.robots(i).U;
                
                % integrate
                [T,X] = ode45(@(t,x)Simulator.dynamics(t,x,U), [0 self.t_step], X, options);
                self.robots(i).updateState(T(end), X(end,:)');
            end
            
            % perform control
            for i=1:numel(self.robots)
                self.robots(i).controller( self.robots(i) );
            end
                        
        end        
    end
    
    methods(Static)
       
        function Xdot = dynamics(~,X,U)
            Xdot = zeros(3,1);
            Xdot(1) = U(1) * cos(X(3));
            Xdot(2) = U(1) * sin(X(3));
            Xdot(3) = U(2);
        end
        
    end
    
end
