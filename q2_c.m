% This script runs Q2(c)

clc;
clear;
close all;

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_c');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(inf);

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

% This is how to extract the graph from the optimizer
graph = drivebotSLAMSystem.optimizer();

% This is how to extract cell arrays of the vertices and edges from the
% graph
allVertices = graph.vertices();
allEdges = graph.edges();

% Work out the number of vehicle poses and landmarks. 
numVehicleVertices = 0;
numLandmarks = 0;

landmarkObservationsPerVehicleVertex = 0;
observationsPerLandmarkVertex = 0;

%% Q2c:
% Finish implementing the code to capture information about the graph
% structure.

for i = 1 : length(allVertices)
    curVertix = allVertices{i};
    numVehicleVertices = numVehicleVertices + isa(curVertix, 'drivebot.graph.VehicleStateVertex');
    numLandmarks = numLandmarks + isa(curVertix, 'drivebot.graph.LandmarkStateVertex');
end

fprintf('Number of vehicle poses stored: %d\n', numVehicleVertices);
fprintf('Number of landmards initalized: %d\n', numLandmarks);

numObservations = 0;
for i = 1 : length(allEdges)
    curEdge = allEdges{i};
    numObservations = numObservations + isa(curEdge, 'drivebot.graph.LandmarkRangeBearingEdge');
end
fprintf('numObservations: %d\n', numObservations);

landmarkObservationsPerVehicleVertex = numObservations / numVehicleVertices;
observationsPerLandmarkVertex = numObservations / numLandmarks;

fprintf('Average number of observations made by a robot at each timestep: %f\n', landmarkObservationsPerVehicleVertex);
fprintf('Average number of observations received by each landmark: %f\n', observationsPerLandmarkVertex);


% warning('q2_c:unimplemented', ...
%         'Implement the rest of the graph query code for Q2c.')
%% Q2c end