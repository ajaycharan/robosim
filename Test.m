%% TEST.m
clear;
close all;
home;

sim = Simulator();

sim.robots(1) = Robot();
sim.robots(2) = Robot();

for i=1:100
    sim.step();
end
