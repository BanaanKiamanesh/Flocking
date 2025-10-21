classdef Agent
    properties
        States          % [pos; vel] column vector, length = 2*Dim
        dt
        Dim
    end

    methods
        function obj = Agent(Dim)
            obj.Dim = Dim;
            obj.States = zeros(2*Dim, 1);   % [p; v]
        end

        function dStates = ODE(obj, u)
            % Double-integrator: p_dot = v, v_dot = u
            dStates = [obj.States(obj.Dim + 1:end); u];
        end

        function Arr = DistanceFromOthers(obj, AgentArray)
            AgentStates = [AgentArray.States];
            PosDiff     = AgentStates(1:obj.Dim, :) - obj.States(1:obj.Dim);
            Arr = vecnorm(PosDiff, 2, 1);
        end
    end
end
