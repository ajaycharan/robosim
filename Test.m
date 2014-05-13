%% TEST.m
% Demonstrate simulator and controllers

clear;
close all;
home;

sim = Simulator(4);

sim.robots(1) = Robot(1, [1 0 0]');
sim.robots(2) = Robot(2, [3 -2 0]');
sim.robots(3) = Robot(3, [-5 1 0]);

sim.robots(1).U = [-0.1; 0];
sim.robots(2).assignLeader(sim.robots(1), 1, pi*2/3);
sim.robots(3).assignLeader(sim.robots(1:2), 1, 1);

sim.simulate();
sim.plot();