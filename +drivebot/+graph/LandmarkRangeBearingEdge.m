% This edge encodes a 3D range bearing measurement.
%
% The measurement is in spherical polar coordinates

% Jacobian from https://github.com/petercorke/robotics-toolbox-matlab/blob/master/RangeBearingSensor.m

classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    
    methods(Access = public)
    
        function this = LandmarkRangeBearingEdge()
            this = this@g2o.core.BaseBinaryEdge(2);
        end
        
        function initialize(this)
            %% Q2b:
            % Complete implementation
            priorX = this.edgeVertices{1}.x;
            landmark = zeros(2,1);

            % compute the x and y position of the landmark
            landmark(1) = priorX(1) + this.z(1) * cos(this.z(2) + priorX(3)); % landmark(x)=vehicle(x)+range*cos(beta+phi)
            landmark(2) = priorX(2) + this.z(1) * sin(this.z(2) + priorX(3)); % landmark(y)=vehicle(y)+range*sin(beta+phi)
            this.edgeVertices{2}.setEstimate(landmark);

            % warning('landmarkrangebearingedge:initialize:unimplemented', ...
            %     'Implement the rest of this method for Q2b.');
            %% Q2b end
        end
        
        function computeError(this)
            %% Q2b:
            % Complete implementation
            priorX = this.edgeVertices{1}.estimate();
            landmark = this.edgeVertices{2}.estimate();
            dx = landmark(1:2) - priorX(1:2);

            % error(1)=r_k^i - z(1)
            this.errorZ(1) = norm(dx) - this.z(1);
            % error(2)=beta_k^i - z(2)
            this.errorZ(2) = g2o.stuff.normalize_theta(atan2(dx(2), dx(1)) - priorX(3) - this.z(2));

            % warning('landmarkrangebearingedge:computeerror:unimplemented', ...
            %     'Implement the rest of this method for Q2b.');
            %% Q2b end
        end
        
        function linearizeOplus(this)
            %% Q2b:
            % Complete implementation
            priorX = this.edgeVertices{1}.estimate();
            landmark = this.edgeVertices{2}.estimate();
            dx = landmark(1:2) - priorX(1:2);
            r = norm(dx);
            
            % compute the Jacobian for vehicle state
            this.J{1} = [-dx(1)/r, -dx(2)/r, 0;
                         dx(2)/r^2, -dx(1)/r^2, -1];
            % compute the Jacobian for landmark state
            this.J{2} = -this.J{1}(1:2, 1:2);

            % warning('landmarkrangebearingedge:linearizeoplus:unimplemented', ...
            %     'Implement the rest of this method for Q2b.');
            %% Q2b end
        end        
    end
end