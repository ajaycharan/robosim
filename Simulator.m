%% SIMULATOR.m
% Run the simuation using ODE45 and generate plots

classdef Simulator < handle

    properties
        t_step = 0.01;              
        t_max = 0;
        t_cur = 0;
        robots = Robot.empty();
    end
    
    properties(Access=private)
        hFigure % figure for simulation
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
            axis equal;
            
            % run simulation
            while self.t_cur < self.t_max
                self.step();
              
                % update all robot plots
                self.robots.plot();
                
                title(sprintf('Simulation time: %.3f', self.t_cur));
                drawnow;
            end            
        end
        
        function step(self)
            
            % ODE options
            options = odeset('RelTol', 1e-2, 'AbsTol', 1e-4);
            
            t_start = self.t_cur;
            t_final = self.t_cur + self.t_step;
            
            for i=1:numel(self.robots)
                
                X = self.robots(i).X;
                U = self.robots(i).U;
                
                % integrate
                [T,X] = ode45(@(t,x)Simulator.dynamics(t,x,U), [t_start t_final], X, options);
                self.robots(i).updateState(T(end), X(end,:)');
            end
            self.t_cur = t_final;
            
            % perform control
            for i=1:numel(self.robots)
                self.robots(i).controller( t_final, self.robots(i) );
            end
                        
        end
        
        function plot(self)
            
            titles = {'x','y','theta'};
            
            % x/y plot 
            figure;
            for i=1:3
                subplot(3,1,i);
                hold on;
                grid on;
                for j=1:numel(self.robots)
                    plot(self.robots(j).Thist, self.robots(j).Xhist(i,:), 'Color',... 
                        Robot.color(self.robots(j).id,:));
                end
                axis tight;
                xlabel('Time');
                ylabel(titles{i});
            end
            
            % top-down plot
            figure;
            hold on;
            grid on;
            for i=1:numel(self.robots)
                
                plot(self.robots(i).Xhist(1,:), self.robots(i).Xhist(2,:), 'Color',...
                    Robot.color(self.robots(i).id,:));
                
                plot(self.robots(i).Xhist(1,end), self.robots(i).Xhist(2,end), 'o', 'Color',...
                    Robot.color(self.robots(i).id,:));
            end
            title('2D plot');
            xlabel('x');
            ylabel('y');
            
        end
    end
    
    methods(Static)
       
        function Xdot = dynamics(~,X,U)
            % simple dynamics for non-holonomic robot
            Xdot = zeros(3,1);
            Xdot(1) = U(1) * cos(X(3));
            Xdot(2) = U(1) * sin(X(3));
            Xdot(3) = U(2);
        end
        
    end 
end
