classdef Simulator < handle

    properties
        t_step = 0.01;
        t_max = 0;
        t_cur = 0;
        robots = Robot.empty();
    end
    
    properties(Access=private)
        hFigure
    end
    
    methods
        function self = Simulator(t_max)
            self.hFigure = figure;
            
            if t_max <= 0
                error('t_max must be >= 0');
            end
            self.t_max = t_max;
            
        end
        
        function simulate(self)
            
            % configure plot
            figure(self.hFigure);
            plot(0,0);  % empty
            hold on;
            grid on;
            set(gca, 'Box', 'On');
            
            % run simulation
            while self.t_cur < self.t_max
                [min_pos, max_pos] = self.step();
                
                % reshape axes
                axis([min_pos(1)-1 max_pos(1)+1 min_pos(2)-1 max_pos(2)+1]);
                
                % update all robot plots
                self.robots.plot();
            end            
        end
        
        function [min_pos, max_pos] = step(self)
            
            % ODE options
            options = odeset('RelTol', 1e-2, 'AbsTol', 1e-4);
            
            t_start = self.t_cur;
            t_final = self.t_cur + self.t_step;
            
            % min/max positions
            min_pos = [ Inf  Inf];
            max_pos = [-Inf -Inf];
            
            for i=1:numel(self.robots)
                
                X = self.robots(i).X;
                U = self.robots(i).U;
                
                % integrate
                [T,X] = ode45(@(t,x)Simulator.dynamics(t,x,U), [t_start t_final], X, options);
                self.robots(i).updateState(T(end), X(end,:)');
                
                min_pos = min(min_pos, [X(1) X(2)]);
                max_pos = max(max_pos, [X(1) X(2)]);
            end
            self.t_cur = t_final;
            
            % perform control
            for i=1:numel(self.robots)
                self.robots(i).controller( t_final, self.robots(i) );
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
