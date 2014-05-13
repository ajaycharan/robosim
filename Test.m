%% TEST.m
clear;
close all;
home;

sim = Simulator(10);

sim.robots(1) = Robot();
sim.robots(2) = Robot();

sim.simulate();

