classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
   
    properties(Access = protected)
        
        xyOffset;
        
    end
    
    methods(Access = public)
    
        function this = GPSMeasurementEdge(xyOffset)
            this = this@g2o.core.BaseUnaryEdge(2);
            this.xyOffset = xyOffset;
        end
        
        function computeError(this)

	    %% Q1d:
        % Implement the code
        x = this.edgeVertices{1}.estimate(); % [x_k; y_k; phi_k]

        c = cos(x(3));
        s = sin(x(3));

        M = [c, -s;
             s, c]; % the rotation matrix M in the 2x2 form
        
        % compute error: x_k + M * [delta_x; delta_y] - z_k
        this.errorZ = x(1:2) + M * this.xyOffset - this.z;

        % warning('gpsmeasurementedge:computeerror:unimplemented', ...
        %         'Implement the rest of this method for Q1d.');
        %% Q1d end
        end
        
        function linearizeOplus(this)

	    %% Q1d:
        % Implement the code
        x = this.edgeVertices{1}.estimate();

        c = cos(x(3));
        s = sin(x(3));

        % compute the Jacobian for x_k, the result is a 2x3 matrix
        this.J{1} = zeros(2,3);
        this.J{1}(1,1) = 1;
        this.J{1}(2,1) = 0;
        this.J{1}(1,2) = 0;
        this.J{1}(2,2) = 1;
        this.J{1}(1,3) = -this.xyOffset(1) * s - this.xyOffset(2) * c;
        this.J{1}(2,3) = this.xyOffset(1) * c - this.xyOffset(2) * s;

        % warning('gpsmeasurementedge:lineareizeoplus:unimplemented', ...
        %         'Implement the rest of this method for Q1d.');
        %% Q1d end
        end
    end
end
