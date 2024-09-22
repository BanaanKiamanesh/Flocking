        classdef Flock        

    properties
        Agents

        % Flock Params
        NumAgents
        Dim
        A
        B
        C
        H

        Radius
        Distance
        Epsilon

        C1_alpha
        C2_alpha
        C1_gamma
        C2_gamma
    end

    methods
        function obj = Flock(Par)
            % Params Unpack
            obj.NumAgents = Par.NumAgents;
            obj.Dim = Par.Dim;
            obj.A = Par.A;
            obj.B = Par.B;
            obj.C = Par.C; 
            obj.H = Par.H;

            obj.Radius   = Par.Radius;        % Perception Radius
            obj.Distance = Par.Distance;      % Distance of Agents
            obj.Epsilon  = Par.Epsilon;

            obj.C1_alpha = Par.C1_alpha;
            obj.C2_alpha = Par.C2_alpha;
            obj.C1_gamma = Par.C1_gamma;
            obj.C2_gamma = Par.C2_gamma;
           

            % Random Init Flock
            obj.Agents = Agent(obj.Dim);
            for i = 2:obj.NumAgents
                obj.Agents(i) = Agent(obj.Dim);
            end
        end

        function dX = CollectiveODE(obj, t, X, Radius, Traj)
            
            % Create Indices to Access Agents States
            TmpIdx = 0:(obj.Dim*2):numel(X);
            StartIdx = 1 + TmpIdx(1:end - 1);
            EndIdx = TmpIdx(2:end);
            
            % Set States
            for i = 1:obj.NumAgents
                obj.Agents(i).States = X(StartIdx(i):EndIdx(i));
            end

            % Calculate Control Inputs and Apply
            dX = zeros(size(X));
            for i = 1:obj.NumAgents
                % P n Q of This
                ThisP = obj.Agents(i).States(          1:obj.Dim);
                ThisQ = obj.Agents(i).States(obj.Dim + 1:    end);

                % Others That This Agent Perceives
                Idx = obj.Agents(i).DistanceFromOthers(obj.Agents) <= Radius;
                
                % Calculate U
                U = zeros(obj.Dim, 1);
                if sum(Idx) > 1
                    % States of Neighbors
                    OtherStates = [obj.Agents(Idx).States];

                    % P n Q of Others
                    OthersP = OtherStates(          1: obj.Dim, :);
                    OthersQ = OtherStates(obj.Dim + 1:     end, :);
                    
                    % Calc Nij and Aij
                    N = Nij(ThisP, OthersP, obj.Epsilon);
                    Ap = Aij(ThisP, OthersP, obj.Epsilon, obj.Radius, obj.H);
                
                    % Terms 1
                    T1 = sum(PhiAlpha(SigmaNorm(OthersP - ThisP, obj.Epsilon), ...
                                      obj.Radius, obj.Distance, obj.Epsilon, ...
                                      obj.A, obj.B, obj.C, obj.H) .* N, 2) ...
                         * obj.C2_alpha;
                    
                    % Term 2
                    T2 = sum(Ap .* (OthersQ - ThisQ), 2) * obj.C2_alpha;
                    
                    U = T1 + T2;
                end

                % State Derivative
                dX(StartIdx(i):EndIdx(i)) = obj.Agents(i).ODE(U);
            end
        end

        function Motion = Simulate(obj, SimTime, TrajFunc)
            tSpan    = [0, SimTime];
            ODEFun   = @(t, X) obj.CollectiveODE(t, X, obj.Radius, TrajFunc);
            InitCond = reshape([obj.Agents.States], [], 1);

            % [Motion.t, Motion.Y] = ode23(ODEFun, tSpan, InitCond);
            [Motion.t, Motion.Y] = odeEuler(ODEFun, tSpan, InitCond, .05);
        end

        function M = AdjacencyMat(obj)
            M = zeros(obj.NumAgents, 'logical');
            for i = 1:obj.NumAgents
                M(i, :) = obj.Agents(i).DistanceFromOthers(obj.Agents) <= obj.Radius;
            end
        end

        function MHist = AdjMatChange(obj, Motion)
            MHist = zeros(obj.NumAgents, obj.NumAgents, length(Motion.t), 'logical');

            for j = 1:numel(Motion.t)
                %%%% Set States
                X = Motion.Y(j, :)';

                % Create Indices to Access Agents States
                TmpIdx = 0:(obj.Dim*2):numel(X);
                StartIdx = 1 + TmpIdx(1:end - 1);
                EndIdx = TmpIdx(2:end);

                for i = 1:obj.NumAgents
                    obj.Agents(i).States = X(StartIdx(i):EndIdx(i));
                end

                %%%% Adj Matrix
                MHist(:, :, j) = obj.AdjacencyMat();
            end
        end
    end
end

function [t, Y] = odeEuler(ODEFun, tSpan, Y0, dt)
    
    t0 = tSpan(1);
    tf = tSpan(2);
    t = t0:dt:tf;  
    
    Y = zeros(length(t), length(Y0));
    Y(1, :) = Y0;
    
    for i = 1:(length(t)-1)
        Y(i+1, :) = Y(i, :) + dt * ODEFun(t(i), Y(i, :)')';
    end
end
