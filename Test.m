%% TEST.m
clear;
close all;
home;

sim = Simulator(10);

sim.robots(1) = Robot(1, [0 0 0]');
sim.robots(2) = Robot(2, [1 2 0.4]');

sim.robots(1).U = [0.1; 0]; 
sim.robots(2).U = [0; 0.05];

sim.simulate();

