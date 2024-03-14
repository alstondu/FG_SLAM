% This script runs Q2(d)

clc;
clear;
close all;

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableLaser = true;

% For this part of the coursework, this should be set to false.
configuration.perturbWithNoise = false;

% Set this value to truncate the run at a specified timestep rather than
% run through the whole simulation to its end.
configuration.maximumStepNumber = 1207; % before closure: 1207, after closure: 1208

% Set up the simulator
simulator_1 = drivebot.DriveBotSimulator(configuration, 'q2_d');

% Create the localization system
drivebotSLAMSystem_1 = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem_1.setRecommendOptimizationPeriod(inf);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem_1.setValidateGraph(false);

% Run the main loop and correct results
results_1 = minislam.mainLoop(simulator_1, drivebotSLAMSystem_1);

title('Simulator Output before loop closure')
xlabel('x Position')
ylabel('y Position')

%% Q2d:
% Explore the  timestep where the loop closure occurs, and get
% results just before and after the loop closure event
configuration.maximumStepNumber = 1208; % before closure: 1207, after closure: 1208
simulator_2 = drivebot.DriveBotSimulator(configuration, 'q2_d');
drivebotSLAMSystem_2 = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem_2.setRecommendOptimizationPeriod(inf);
drivebotSLAMSystem_2.setValidateGraph(false);
result_2 = minislam.mainLoop(simulator_2, drivebotSLAMSystem_2);
title('Simulator Output after loop closure')
xlabel('x Position')
ylabel('y Position')

% get the covariance matrix of the landmarks and the vehicle before and after loop closure
[trash, SLAM_1_cov, trash] = drivebotSLAMSystem_1.landmarkEstimates();
[trash, SLAM_2_cov, trash] = drivebotSLAMSystem_2.landmarkEstimates();
[trash, P_V_1] = drivebotSLAMSystem_1.platformEstimate();
[trash, P_V_2] = drivebotSLAMSystem_2.platformEstimate();
for i = 1 : length(SLAM_1_cov)
    fprintf('--------------------\n');
    fprintf('For landmark %d:\n', i);
    det1 = det(SLAM_1_cov(:,:,i));
    fprintf('det (before loop closure): %e\n', det1);
    fprintf('Landmark Covariance (before loop closure):\n');
    disp(SLAM_1_cov(:,:,i));
    
    det2 = det(SLAM_2_cov(:,:,i));
    fprintf('det (after loop closure): %e\n', det2);
    fprintf('Landmark Covariance (after loop closure):\n');
    disp(SLAM_2_cov(:,:,i));

    fprintf('det2-det1: %e\n', det2-det1);
    fprintf('(det2-det1)/det1: %.2f%%\n', (det2-det1)/det1*100);
end
fprintf('Vehicle Covariance (before loop closure):\n');
disp(P_V_1);
fprintf('Vehicle Covariance (after loop closure):\n');
disp(P_V_2);
% warning('q2_d:unimplemented', ...
%         'Analyse loop closure behaviour for Q2d.')
%% Q2d end

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% % Plot optimisation times
% minislam.graphics.FigureManager.getFigure('Optimization times');
% clf
% plot(results{1}.vehicleStateTime, results{1}.optimizationTimes, '*')
% title('Optimization Times');
% xlabel('Time (s)');
% ylabel('Optimization Times (s)');
% hold on
% 
% % Plot errors
% minislam.graphics.FigureManager.getFigure('Errors');
% clf
% res = results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory';
% res(:,3) = g2o.stuff.normalize_thetas(res(:,3));
% plot(results{1}.vehicleStateTime, res);
% % plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
% legend('error in x', 'error in y', 'error in \phi');
% title('Errors');
% xlabel('Time (s)');
% ylabel('Errors (m)');
% hold on
% 
% % Plot covariance
% minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
% clf
% plot(results{1}.vehicleStateTime, results{1}.vehicleCovarianceHistory')
% legend('covariance in x', 'covariance in y', 'covariance in \phi');
% title('Vehicle Covariances');
% xlabel('Time (s)');
% ylabel('Vehicle Covariances');
% hold on
% 
% % Plot chi2 values
% minislam.graphics.FigureManager.getFigure('chi2');
% clf
% plot(results{1}.chi2Time, log(results{1}.chi2History))
% title('Chi2');
% xlabel('Time (s)');
% ylabel('Chi2');
% hold on
