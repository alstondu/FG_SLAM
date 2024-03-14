% This class implements an event-based estimation system using g2o and
% the barebones for building up a minimal, ideal SLAM system. The system is
% event-based and responds to a sequence of events which are time stamped
% and served in order. To implement your SLAM system, you will need to
% implement various methods which mostly involve working with the graph.
% These methods are initially stubbed out and will generate exceptions if
% you try to call them.

classdef DriveBotSLAMSystem < minislam.slam.SLAMSystem
    
    properties(Access = public, Constant)
        % Platform state dimension
        NP = 3;
        
        % Landmark dimension
        NL = 2;
        
        % Initial cache size; might help a bit with performance
        INITIAL_CACHE_SIZE = 10000;
    end
    
    properties(Access = protected)
        
        % The most recently created vehicle vertex.
        currentVehicleVertex;
        
        % The set of all vertices associated with the vehicle state over
        % time.
        vehicleVertices;
        vehicleVertexId;
        
        % The set of all prediction edges. These are removed from the graph
        % afterwards if we don't use prediction
        processModelEdges;
        numProcessModelEdges;
        
        % The landmark vertices. Confusingly enough, "Map" here refers to
        % the data structure which is used to store the landmarks. (It
        % allows random access of landmarkID to landmark object.)
        landmarkIDStateVectorMap;
        
        % How often we recommend running the optimization
        recommendOptimizationPeriod;
        
        % Flag to show if we should prune the edges. This is needed for
        % question Q3a
        removePredictionEdgesFromGraph;
        keepFirstPredictionEdge;

        %% Q3b: Flag to show if we should prune the edges
        removeLandmarkEdgesFromGraph;
        %% Q3b end
        
    end
    
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = DriveBotSLAMSystem(configuration)
            
            % Call the base class constructor
            this = this@minislam.slam.SLAMSystem(configuration);
            
            % Preallocate for convenience
            this.vehicleVertices = cell(1, this.INITIAL_CACHE_SIZE);
            
            % No vehicle vertices initally set
            this.vehicleVertexId = 0;
            
            % The set of prediction edges, initially empty
            this.processModelEdges = cell(1, this.INITIAL_CACHE_SIZE);
            this.numProcessModelEdges = 0;
            
            % Allocate the landmark map
            this.landmarkIDStateVectorMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            
            % By default, run very infrequently
            this.recommendOptimizationPeriod = inf;
            
            this.removePredictionEdgesFromGraph = false;
            this.keepFirstPredictionEdge = false;

            %% Q3b: initialize the flag as false
            this.removeLandmarkEdgesFromGraph = false;
            %% Q3b end
        end
        
        % Destroy the graph when we destroy the SLAM system.
        % Without this, MATLAB will crash whenever this object is destroyed.

        function delete(this)
            vertices = this.graph.vertices();

            for v = 1 : length(vertices)
                this.graph.removeVertex(vertices{v});
            end
        end
        
        % Recommend if an optimization is a good idea. Based on an event,
        % some activities (e.g., such as loop closing) can have a very big
        % impact on the estimates. The logic we have here just recommends
        % an optimization if a fixed number of steps have been completed.
        
        function recommendation = recommendOptimization(this)
            
            % This is how to do it after every 100 steps
            recommendation = rem(this.stepNumber, ...
                this.recommendOptimizationPeriod) == 0;
        end
        
        % Set the value of how often recommend optimization should return
        % true
        function setRecommendOptimizationPeriod(this, newRecommendOptimizationPeriod)
            this.recommendOptimizationPeriod = newRecommendOptimizationPeriod;
        end
        
        % Return the current mean and covariance estimate of the robot.
        % This is only valid after optimization has been called.
        function [x, P] = platformEstimate(this)
            [xS, PS] = this.graph.computeMarginals(this.currentVehicleVertex);
            x=full(xS);
            P=full(PS);
        end
        
        % Returns the entire history of the platform estimates. Suppose
        % there are n vehicle vertices. T is a 1 by N dimensional vector of
        % timesteps. X is a 3 by N dimensional vector of vehicle state (x,
        % y, theta). P is a 3 by N dimensional vector where the nth column
        % are the diagonals from the covariance matrix.
        function [T, X, P] = platformEstimateHistory(this)
            
            % Extract the graph
            [xS, PS] = this.graph.computeMarginals();
            
            % Create the output array
            X = zeros(this.NP, this.vehicleVertexId);
            P = zeros(this.NP, this.vehicleVertexId);
            T = zeros(1, this.vehicleVertexId);
            
            % Copy the outputs over
            for v = 1 : this.vehicleVertexId
                idx = this.vehicleVertices{v}.hessianIndex();
                
                T(v) = this.vehicleVertices{v}.time();
                
                % Copy the estimate into the array. If the vertices is
                % fixed (conditioned), its estimate is okay. The covariance
                % is not explicitly defined, but has a value of zero.
                % Therefore we fill this manually.
                if (isempty(idx) == true)
                    X(:, v) = this.vehicleVertices{v}.estimate();
                    P(:, v) = zeros(3, 1);
                else
                    X(:, v) = full(xS(idx));
                    P(:, v) = full(diag(PS(idx, idx)));
                end
            end
        end
        
        % Return the means and covariances of the landmark estimates. These
        % are only valid after optimization has been called.
        function [x, P, landmarkIds] = landmarkEstimates(this)
            
            landmarkVertices = values(this.landmarkIDStateVectorMap);
            
            numberOfLandmarks = length(landmarkVertices);
            
            landmarkIds = NaN(1, numberOfLandmarks);
            x = NaN(this.NL, numberOfLandmarks);
            P = NaN(this.NL, this.NL, numberOfLandmarks);
            
            [xS, PS] = this.graph.computeMarginals();
            
            for l = 1 : numberOfLandmarks
                landmarkIds(l) = landmarkVertices{l}.landmarkId();
                idx = landmarkVertices{l}.hessianIndex();
                x(:, l) = full(xS(idx));
                if (isempty(idx == true))
                    P(:, :, l) = zeros(3, 3);
                else
                    P(:, :, l) = full(PS(idx, idx));
                end
            end
        end
        
        % We overload the optimize method so that you can add additional
        % logic here
        function chi2 = optimize(this, maximumNumberOfOptimizationSteps)
            
            %% Q3: count the number of vertices and edges before and after pruning
            allEdges = this.graph.edges();
            allVertices = this.graph.vertices();
            fprintf('Current num of vertices: %d, num of edges: %d\n', length(allVertices), length(allEdges));
            
            tic
            % Remove the prediction edges if requested.
            %% Q3a
            if (this.removePredictionEdgesFromGraph == true) 
                this.deleteVehiclePredictionEdges();
            end
            
            %% Q3b
            if (this.removeLandmarkEdgesFromGraph == true)
                this.graphPruning();
            end

            elapsedTime = toc;
            
            allEdges = this.graph.edges();
            allVertices = this.graph.vertices();
            fprintf('After remove, num of vertices: %d, num of edges: %d\n', length(allVertices), length(allEdges));
            fprintf('Time used for pruning graph: %.2fs\n', elapsedTime);
            %% Q3 end

            % Now call the actual optimizer. Let it handle the default if
            % no steps are specified.
            if (nargin > 1)
                chi2 = optimize@minislam.slam.SLAMSystem(this, ...
                    maximumNumberOfOptimizationSteps);
            else
                chi2 = optimize@minislam.slam.SLAMSystem(this);
            end
        end
        
        function setRemovePredictionEdges(this, removeEdges, keepFirst)
            this.removePredictionEdgesFromGraph = removeEdges;
            this.keepFirstPredictionEdge = keepFirst;
            
        end

        %% Q3b: function to change the flag
        function setRemoveLandmarkEdges(this, removeEdges)
            this.removeLandmarkEdgesFromGraph = removeEdges;
        end
        %% Q3b end
    end
    
    % These are the methods you will need to overload
    methods(Access = protected)
        
        % Handle the initial condition
        
        function handleInitialConditionEvent(this, event)
            
            % Create the first vertex, set its estimate to the initial
            % value and add it to the graph.
            this.currentVehicleVertex = drivebot.graph.VehicleStateVertex(this.currentTime);
            this.currentVehicleVertex.setEstimate(event.data);
            this.graph.addVertex(this.currentVehicleVertex);
            
            % Set the book keeping for this initial vertex.
            this.vehicleVertexId = 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
            
            % If the covariance is 0, the vertex is known perfectly and so
            % we set it as fixed. If the covariance is non-zero, add a
            % unary initial prior condition edge instead. This adds a soft
            % constraint on where the state can be.
            if (det(event.covariance) < 1e-6)
                this.currentVehicleVertex.setFixed(true);
            else
                initialPriorEdge = drivebot.graph.InitialPriorEdge();
                initialPriorEdge.setMeasurement(event.data);
                initialPriorEdge.setInformation(inv(event.covariance));
                initialPriorEdge.setVertex(this.currentVehicleVertex);
                this.graph.addEdge(initialPriorEdge);
            end
        end
        
        function handleNoPrediction(~)
            % Nothing to do
        end
        
        function handleHeartbeatEvent(this, ~)
            % Nothing to do
        end
        
        function handlePredictToTime(this, time, dT)

            % Create the next vehicle vertex and add it to the graph
            
            this.currentVehicleVertex = drivebot.graph.VehicleStateVertex(time);
            
            %% Q1b:
            % Implement prediction code here

            % create process model edge
            processModelEdge = drivebot.graph.VehicleKinematicsEdge(dT);

            % assign 2 vertices for this binary edge
            processModelEdge.setVertex(1, this.vehicleVertices{this.vehicleVertexId}); % x_k
            processModelEdge.setVertex(2, this.currentVehicleVertex); % x_{k+1}

            % set input u and inv(covariance)
            processModelEdge.setMeasurement(this.u); % input u
            processModelEdge.setInformation(inv(this.uCov)); % inverse of the input covariance

            processModelEdge.initialize();

            % create the new prediction vertex
            this.currentVehicleVertex.setEstimate(processModelEdge.vertex(2).estimate());

            % add the new edge and vertex to the graph
            this.graph.addEdge(processModelEdge);
            this.graph.addVertex(this.currentVehicleVertex);
            this.numProcessModelEdges = this.numProcessModelEdges + 1;
            this.processModelEdges{this.numProcessModelEdges} = processModelEdge;

            % warning('drivebotslam:handlepredicttotime:unimplemented', ...
            %     'Implement the rest of this method for Q1b.');
            %% Q1b end
            
            % Bump the indices
            this.vehicleVertexId = this.vehicleVertexId + 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
        end
        
        function handleGPSObservationEvent(this, event)

            %% Q1d:
            % Create a GPS measurement edge and add it to the graph

            % create the GPS edge
            gpsMeasurementEdge = drivebot.graph.GPSMeasurementEdge(this.configuration.gpsPositionOffset);

            % assign to x_k
            gpsMeasurementEdge.setVertex(1, this.currentVehicleVertex);

            % add [delta_x; delta_y] and inv(covariance) to the edge
            gpsMeasurementEdge.setMeasurement(event.data);
            gpsMeasurementEdge.setInformation(inv(event.covariance));

            % add the edge to the graph
            this.graph.addEdge(gpsMeasurementEdge);

            % warning('drivebotslam:handlegpsobservationevent:unimplemented', ...
            %     'Implement the rest of this method for Q1d.');
            %% Q1d end
        end
        
        function handleCompassObservationEvent(this, event)
            
            % Q1c
            % Create a compass measurement edge and add it to the graph
            compassMeasurementEdge = drivebot.graph.CompassMeasurementEdge(this.configuration.compassAngularOffset);
            compassMeasurementEdge.setVertex(1, this.currentVehicleVertex);
            compassMeasurementEdge.setMeasurement(event.data);
            compassMeasurementEdge.setInformation(inv(event.covariance));
            this.graph.addEdge(compassMeasurementEdge);
        end
        
        function handleLandmarkObservationEvent(this, event)
            
            % Iterate over all the landmark measurements
            for l = 1 : length(event.landmarkIds)
                
                % Get the landmark vertex associated with this measurement.
                % If necessary, a new landmark vertex is created and added
                % to the graph.
                [landmarkVertex, newVertexCreated] = this.createOrGetLandmark(event.landmarkIds(l));
                z = event.data(:, l);

                %% Q2b:
                % Complete the implementation

                % create the landmark observation edge
                landmarkRangeBearingEdge = drivebot.graph.LandmarkRangeBearingEdge();

                % assign the vertices, one for x_k and the other for the
                % landmark observed
                landmarkRangeBearingEdge.setVertex(1, this.currentVehicleVertex);
                landmarkRangeBearingEdge.setVertex(2, landmarkVertex);

                % add z and inv(covariance
                landmarkRangeBearingEdge.setMeasurement(z);
                landmarkRangeBearingEdge.setInformation(inv(event.covariance));
                
                % initialize the landmark that has not on the graph
                if (newVertexCreated)
                    landmarkRangeBearingEdge.initialize();
                end

                % warning('drivebotslamsystem:handlelandmarkobservationevent:unimplemented', ...
                %     'Implement the rest of this method for Q2b.');
                % Q2b end

                this.graph.addEdge(landmarkRangeBearingEdge);
            end
        end
        
        function deleteVehiclePredictionEdges(this)

            %% Q3a:

            % get all the edges
            allEdges = this.graph.edges();
            flagFirstEdge = true;

            for i = 1 : length(allEdges)
                curEdge = allEdges{i};
                % figure out if current edge is Vehicle Edge
                if (isa(curEdge, 'drivebot.graph.VehicleKinematicsEdge'))
                    if (flagFirstEdge) % if current edge is the first edge
                        flagFirstEdge = false;
                        if (this.keepFirstPredictionEdge) % determine whether the first edge will be deleted
                            continue
                        end
                    end
                    this.graph.removeEdge(allEdges{i}); % actually delete the edge
                end
            end
            % warning('drivebotslam:deletevehiclepredictionedges:unimplemented', ...
            %     'Implement the rest of this method for Q3a.');
            %% Q3a end
        end

        function graphPruning(this)

            %% Q3b: core implement of the graph pruning algorithm

            % step 1. count the number of vehicle vertex and landmark edge
            allVertices = this.graph.vertices();
            numVehicleVertices = 0;
            numLandmarkEdges = 0;
            for i = 1 : length(allVertices)
                curVertex = allVertices{i};
                if isa(curVertex, 'drivebot.graph.VehicleStateVertex')
                    numVehicleVertices = numVehicleVertices + 1;
                    edges = allVertices{i}.edges();
                    for j = 1 : length(edges)
                        if isa(edges{j}, 'drivebot.graph.LandmarkRangeBearingEdge')
                            numLandmarkEdges = numLandmarkEdges + 1;
                        end
                    end
                end
            end
            fprintf('numVehicleVertices: %d, numAllEdges: %d, average %f\n', numVehicleVertices, numLandmarkEdges, numLandmarkEdges/numVehicleVertices);
            % divide them as the threshold
            threshold = round(numLandmarkEdges / numVehicleVertices);

            indexToDelete = []; % decision vertex index array, 0 means not to delete, 1 means to delete
            numVertexLessThanT = 0; % count number of vertex less than threshold
            numVertexLargerThanT = 0; % count number of vertex larger than threshold
            for i = 1 : length(allVertices)
                curVertex = allVertices{i};
                if isa(curVertex, 'drivebot.graph.VehicleStateVertex')
                    numLandmarkEdge = 0; % count the number of edges attached for current vertex
                    edges = allVertices{i}.edges();
                    for j = 1 : length(edges)
                        if isa(edges{j}, 'drivebot.graph.LandmarkRangeBearingEdge')
                            numLandmarkEdge = numLandmarkEdge + 1;
                        end
                    end
                    if numLandmarkEdge < threshold % vertex less than threshold, its landmark edge should be deleted
                        numVertexLessThanT = numVertexLessThanT + 1;
                        indexToDelete = [indexToDelete 1];
                    else % vertex larger than threshold
                        numVertexLargerThanT = numVertexLargerThanT + 1;
                        indexToDelete = [indexToDelete 0];
                    end
                else % landmark vertex, not to consider
                    indexToDelete = [indexToDelete -1];
                end
            end
            fprintf('numVertexLessThanT: %d, numVertexLargerThanT, %d\n', numVertexLessThanT, numVertexLargerThanT);
            fprintf('length(Array): %d\n', length(indexToDelete));
            
            a = 4;
            k = round(numVertexLargerThanT / a); % select 1/a of vertex above the threshold

            zeroIndices = find(indexToDelete == 0);
            %% HINT:
            % If you get an error when running the script at this point, 
            % please install the 'Statistics and Machine Learning Toolbox'.
            % This function is to select 1/a of the vertices that above the
            % threshold.
            selectedIndices = randsample(zeroIndices, k);
            indexToDelete(selectedIndices) = 1; % the vertices selected, their landmark edge should be deleted

            for i = 1 : length(allVertices)
                curVertex = allVertices{i};
                if indexToDelete(i) == 1
                    edges = curVertex.edges();
                    for k = 1 : length(edges)
                        if isa(edges{k}, 'drivebot.graph.LandmarkRangeBearingEdge')
                            this.graph.removeEdge(edges{k}); % delete the landmark edge for the vertices selected
                        end
                    end
                end
            end
            %% Q3b end
        end
        
        
        % This method returns a landmark associated with landmarkId. If a
        % landmark exists already, it is returned. If it does not exist, a
        % vertex is created and is added to the graph.
        function [landmarkVertex, newVertexCreated] = createOrGetLandmark(this, landmarkId)

            
            % If the landmark exists already, return it
            if (isKey(this.landmarkIDStateVectorMap, landmarkId) == true)
                landmarkVertex = this.landmarkIDStateVectorMap(landmarkId);
                newVertexCreated = false;
                return
            end
            
            fprintf('Creating landmark %d\n', landmarkId);
            
            % Create the new landmark add it to the graph
            landmarkVertex = drivebot.graph.LandmarkStateVertex(landmarkId);
            this.landmarkIDStateVectorMap(landmarkId) = landmarkVertex;
            
            this.graph.addVertex(landmarkVertex);
            
            newVertexCreated = true;
        end
        
        function storeStepResults(this)
            % Nothing
        end
        
    end
end
