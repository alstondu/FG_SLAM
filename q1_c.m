% This script runs Q1(c)

clc;
clear;
close all;

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

%% Q1c: Set the configuration to enable the compass
configuration.enableCompass = true;
%% Q1c end

% Set the compass angular offset. DO NOT change this value.
configuration.compassAngularOffset=0.75*pi;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q1_c');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% Force the optimizer to run with this frequency. This lets you see what's
% happening in greater detail, but slows everything down.
drivebotSLAMSystem.setRecommendOptimizationPeriod(20);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);
title('Simulator Output');
xlabel('x Position');
ylabel('y Position');

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.vehicleStateTime, results{1}.optimizationTimes, '*')
title('Optimization Times');
xlabel('Time (s)');
ylabel('Optimization Times (s)');
hold on

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
res = results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory';
res(:,3) = g2o.stuff.normalize_thetas(res(:,3));
plot(results{1}.vehicleStateTime, res);
% plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
legend('error in x', 'error in y', 'error in \phi');
title('Errors');
xlabel('Time (s)');
ylabel('Errors (m)');
hold on

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleCovarianceHistory')
legend('covariance in x', 'covariance in y', 'covariance in \phi');
title('Vehicle Covariances');
xlabel('Time (s)');
ylabel('Vehicle Covariances');
hold on

% Plot chi2 values
minislam.graphics.FigureManager.getFigure('chi2');
clf
plot(results{1}.chi2Time, log(results{1}.chi2History))
title('Chi2');
xlabel('Time (s)');
ylabel('Chi2');
hold on