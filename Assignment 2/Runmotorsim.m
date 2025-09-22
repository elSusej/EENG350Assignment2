%% Runmotorsim.m
% This script runs a simulation of a motor and plots the results
%
% required file: motorsim.slx
%
%% Define motor parameters
k=2.2; % DC gain [rad/Vs]
sigma=2; % time constant reciprocal [1/s]

%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('motorsim')

%
% run the simulation
%
out=sim('motorsim');

%% A Plot of the results
%
figure
subplot(2,1,1)
plot(out.Voltage,'LineWidth',2)
xlabel('Time (s)')
ylabel('Voltage (V)')
subplot(2,1,2)
plot(out.Velocity,'LineWidth',2)
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')

