classdef Flock < handle
    properties
        N
        Individuals (:, 1) Individual

        Ksep
        Kali
        Kcoh
        Kgoal
    end

    methods
        function obj = Flock(N_, Ksep_, Kali_, Kcoh_, Kgoal_)
            obj.N = N_;
            obj.Ksep  = Ksep_;
            obj.Kali  = Kali_;
            obj.Kcoh  = Kcoh_;
            obj.Kgoal = Kgoal_;

            obj.Init;
        end

        function Init(obj)
            for i = 1:obj.N
                obj.Individuals(i, 1) = Individual;
            end
        end

        function X = GetState(obj)
            X = zeros(3, obj.N);
            for i = 1:obj.N
                X(:, i) = [obj.Individuals(i).Pos.x, ...
                           obj.Individuals(i).Pos.y, ...
                           obj.Individuals(i).Pos.z];
            end
        end

        function [T, X] = Simulate(obj, Tend, dt, Traj)
            % Time Vector
            T = 0 : dt : Tend;
            StepNum = numel(T);

            X = zeros(3, obj.N, StepNum);
            for i = 1:StepNum
                X(:, :, i) = obj.GetState;

                % Actual time at this step
                t = T(i);

                % Find the Goal Position and Velocity for this time
                TrajIdx = 1;
                for j = 1:numel(Traj)
                    if t > Traj{j}{1}
                        TrajIdx = j + 1;
                    end
                end
                if TrajIdx > numel(Traj)
                    TrajIdx = numel(Traj);
                end

                Pgoal = Traj{TrajIdx}{2};
                Vgoal = Traj{TrajIdx}{3};

                % Calculate accelerations from current state
                obj2 = obj.ODE(Pgoal, Vgoal);

                % Integrate (semi-implicit Euler)
                for j = 1:obj.N
                    obj.Individuals(j).Vel = obj.Individuals(j).Vel + obj2.Individuals(j).Acc .* dt;
                    obj.Individuals(j).Pos = obj.Individuals(j).Pos + obj.Individuals(j).Vel .* dt;
                end
            end
        end

        function obj2 = ODE(obj, Pgoal, Vgoal)
            % Function to compute one-step accelerations based on current state

            % Copy handle (alias is fine; we only write Acc)
            obj2 = obj;

            for i = 1:obj.N
                % Get neighbors for individual i
                Neighbors = obj.Individuals(i).GetNeighbors(obj);
                K = numel(Neighbors);

                % Separation
                Fsep = MVector(0,0,0);
                for j = 1:K
                    Pos_i = obj.Individuals(i).Pos;
                    Pos_j = Neighbors(j).Pos;

                    d = Pos_i.dist(Pos_j);
                    if d > 1e-8
                        Fsep = Fsep + (Pos_i - Pos_j) .* (obj.Ksep / d);
                    end
                end

                % Alignment
                VelAvg = MVector(0,0,0);
                for j = 1:K
                    Vel_j = Neighbors(j).Vel;
                    VelAvg = VelAvg + Vel_j .* (1 / max(K,1));
                end
                Fali = (VelAvg - obj.Individuals(i).Vel) .* obj.Kali;

                % Cohesion
                PosAvg = MVector(0,0,0);
                for j = 1:K
                    Pos_j = Neighbors(j).Pos;
                    PosAvg = PosAvg + Pos_j .* (1 / max(K,1));
                end
                Fcoh = (PosAvg - obj.Individuals(i).Pos) .* obj.Kcoh;

                % Goal Seeking PD controller
                p = (Pgoal - obj.Individuals(i).Pos) .* obj.Kgoal;
                d = (Vgoal - obj.Individuals(i).Vel) .* (3 * obj.Kgoal);
                Fgoal = p + d;

                % Net acceleration
                a_i = Fsep + Fali + Fcoh + Fgoal;

                % Apply acceleration
                obj2.Individuals(i).Acc = a_i;
            end
        end
    end
end
