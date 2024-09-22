classdef Agent
    
    properties
        States          % For Each Dim, 1 for Pos, 1 for Vel
        dt
        Dim
    end

    methods
        function obj = Agent(Dim)
            obj.Dim = Dim;
            
            % States: [Pos; Vel]
            obj.States = zeros(2*Dim, 1);
            obj.States(1:Dim) = randi([-150, 150], [Dim, 1]);
        end

        function dStates = ODE(obj, u)
            dStates = [obj.States(obj.Dim + 1:end); u];
        end

        function Arr = DistanceFromOthers(obj, AgentArray)
            AgentStates = [AgentArray.States];
            PosDiff     = AgentStates(1:obj.Dim, :) - obj.States(1:obj.Dim);
            
            Arr = vecnorm(PosDiff, 2, 1);
        end
    end
end