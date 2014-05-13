clear all
close all

rb(1) = Robot(1, zeros(3,1));
rb(2) = Robot(2, ones(3,1));
rb(3) = Robot(3, ones(3,1)*2);

grid on
rb.plot();
set(gca, 'Box', 'On')

for i = 1:100
    rb(1).X = rb(1).X + 0.01;
    rb(1).plot
    drawnow
    
end