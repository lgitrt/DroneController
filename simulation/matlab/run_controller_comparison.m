%% RUN_CONTROLLER_COMPARISON
% Compares Aggressive vs Conservative LQR tuning for tracking a circular
% trajectory on the nonlinear quadrotor model, and produces comparison
% plots saved to results/.
%
% Author: Luca Obwegs

clear; clc; close all;

here = fileparts(mfilename('fullpath'));
addpath(here);

p = parameters();

%% Trajectory definition: smooth-start horizontal circle
traj.radius   = 2;          % [m]
traj.omega    = 2*pi/30;    % [rad/s]  (30 s period)
traj.altitude = 2.5;        % [m]
traj.rampTime = 5;          % [s] smooth-start duration

tEnd = 45;                  % [s] total simulation time
x0 = zeros(12,1);           % start at rest on the ground

%% Controller design
fprintf('Designing Aggressive LQR controller...\n');
K_agg = designLQR(p, 'Tuning', 'Aggressive');
lqrControllerAgg.type = 'LQR';
lqrControllerAgg.K = K_agg;

fprintf('Designing Conservative LQR controller...\n');
K_cons = designLQR(p, 'Tuning', 'Conservative');
lqrControllerCons.type = 'LQR';
lqrControllerCons.K = K_cons;

%% Closed-loop simulation
fprintf('Simulating Aggressive LQR...\n');
TsLQR = 0.01;
aggOut = simulateClosedLoop(p, traj, lqrControllerAgg, TsLQR, tEnd, x0);

fprintf('Simulating Conservative LQR...\n');
consOut = simulateClosedLoop(p, traj, lqrControllerCons, TsLQR, tEnd, x0);

%% Plots and metrics
outDir = fullfile(here, '..', 'results');
metrics = plotComparison(aggOut, consOut, p, outDir);

fprintf('\nDone. Figures saved to %s\n', outDir);
