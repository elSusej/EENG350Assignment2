%% Runmotorsim.m
% This script runs a simulation of a motor and plots the results
%
% required file: motorsim.slx
%
%% Define motor parameters (values of 1.9 and 18 for Alpha, and same for Bravo)
k=1.9; % DC gain [rad/Vs] 
sigma=18; % time constant reciprocal [1/s]
%load('stepData.mat') %I also changed step value to 7.5V w/ a 128/255 pwm wave form (avg V is 3.76)
load('stepData2.mat') %contains experimental data for bravo, line 9 contains data for alpha
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
plot(out.Voltage,'--','linewidth',2)
hold on
plot(data(:,1),data(:,2),'linewidth',2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
subplot(2,1,2)
plot(out.Velocity,'--','linewidth',2)
hold on
plot(data(:,1),data(:,3),'linewidth',2)
hold off
legend('Simulated','Experimental','location','southeast')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
