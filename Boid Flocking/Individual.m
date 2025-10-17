classdef Individual
    properties
        % Physical Properties
        Pos     MVector
        Vel     MVector
        Acc     MVector = MVector(0, 0, 0)

        % Force (unused but kept for parity)
        F       MVector = MVector(0, 0, 0)

        % Perception Radius
        R = 1
    end

    methods
        function obj = Individual()
            % Random position in [-2, 2]^3, zero velocity
            obj.Pos = MVector( (rand(1,3) * 4) - 2 );
            obj.Vel = MVector(0, 0, 0);
        end

        function d = Dist(obj, other)
            arguments
                obj     Individual
                other   (:, :) Individual
            end

            N = numel(other);
            d = zeros(N, 1);
            for i = 1:N
                d(i) = obj.Pos.dist(other(i).Pos);
            end
        end

        function Neighbors = GetNeighbors(obj, F)
            arguments
                obj     Individual
                F       Flock
            end

            % Distances to everyone (includes self at 0)
            Dists = obj.Dist(F.Individuals);

            % Neighbors within radius R (includes self; matches original behavior)
            Neighbors = F.Individuals(Dists < obj.R);
        end
    end
end
