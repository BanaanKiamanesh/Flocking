classdef Flock
    properties
        Agents

        % Dimensions / counts
        NumAgents
        Dim

        % Interaction radii
        r_alpha       % neighbor radius
        d_alpha       % desired separation
        r_beta        % obstacle influence radius

        % Smooth/bump params
        Epsilon
        H

        % Gains
        C1_alpha
        C2_alpha
        C1_beta
        C1_gamma

        % Motion / environment
        v_max
        damping_coeff
        safety_buffer
        threshold_distance
        majority_percentage
        arena_size

        % Goal and obstacles
        Goal          % [Dim x 1]
        Obstacles     % [K x (Dim+1)] rows: [center(1:Dim), radius]
    end

    methods
        function obj = Flock(Par)
            % Core sizes
            obj.NumAgents = Par.NumAgents;
            obj.Dim       = Par.Dim;

            % Radii / geometry
            obj.r_alpha = Par.r_alpha;
            obj.d_alpha = Par.d_alpha;
            obj.r_beta  = Par.r_beta;

            % Smoothness
            obj.Epsilon = Par.Epsilon;
            obj.H       = Par.H;

            % Gains
            obj.C1_alpha = Par.C1_alpha;
            obj.C2_alpha = Par.C2_alpha;
            obj.C1_beta  = Par.C1_beta;
            obj.C1_gamma = Par.C1_gamma;

            % Dynamics / bounds
            obj.v_max               = Par.v_max;
            obj.damping_coeff       = Par.damping_coeff;
            obj.safety_buffer       = Par.safety_buffer;
            obj.threshold_distance  = Par.threshold_distance;
            obj.majority_percentage = Par.majority_percentage;
            obj.arena_size          = Par.arena_size;

            % Goal & obstacles
            obj.Goal      = Par.Goal(:);
            obj.Obstacles = Par.Obstacles;

            % Init agents
            obj.Agents = Agent(obj.Dim);
            for i = 2:obj.NumAgents
                obj.Agents(i) = Agent(obj.Dim);
            end
            for i = 1:obj.NumAgents
                obj.Agents(i).States(1:obj.Dim) = 200*rand(obj.Dim,1);
                obj.Agents(i).States(obj.Dim+1:end) = zeros(obj.Dim,1);
            end
        end

        function dX = CollectiveODE(obj, ~, X)
            tmp = 0:(obj.Dim*2):numel(X);
            sIdx = 1 + tmp(1:end-1);
            eIdx = tmp(2:end);
            for i = 1:obj.NumAgents
                obj.Agents(i).States = X(sIdx(i):eIdx(i));
            end

            % Compute control u_i for each agent (Algorithm 3)
            dX = zeros(size(X));
            for i = 1:obj.NumAgents
                pi = obj.Agents(i).States(          1:obj.Dim);
                vi = obj.Agents(i).States(obj.Dim + 1:    end);

                % neighbors within r_alpha
                distAll = obj.Agents(i).DistanceFromOthers(obj.Agents);
                nMask   = distAll <= obj.r_alpha;
                nMask(i)= false;

                f_alpha = zeros(obj.Dim,1);
                align   = zeros(obj.Dim,1);

                if any(nMask)
                    otherStates = [obj.Agents(nMask).States];
                    Pn = otherStates(          1: obj.Dim, :);
                    Vn = otherStates(obj.Dim + 1:     end, :);

                    diffs = Pn - pi;
                    dists = vecnorm(diffs, 2, 1);

                    for k = 1:size(Pn,2)
                        dist = dists(k);
                        if dist > 1e-8 && dist < obj.r_alpha
                            e_ij  = diffs(:,k) / dist;

                            % bump(dist/r_alpha, H) * (sigma1(dist - d_alpha) - sigma1(r_alpha - d_alpha))
                            b     = Flock.bump(dist/obj.r_alpha, obj.H);
                            phi   = b * (Flock.sigma1(dist - obj.d_alpha) - Flock.sigma1(obj.r_alpha - obj.d_alpha));

                            f_alpha = f_alpha + obj.C1_alpha * phi * e_ij;
                            align   = align   + obj.C2_alpha * b * (Vn(:,k) - vi);
                        end
                    end
                end

                % Obstacle avoidance (dimension-agnostic)
                f_beta = zeros(obj.Dim,1);
                for ob = 1:size(obj.Obstacles,1)
                    oc   = obj.Obstacles(ob,1:obj.Dim).';
                    orad = obj.Obstacles(ob,end);

                    to_obs = pi - oc;
                    dist   = norm(to_obs);
                    if dist < obj.r_beta && dist > 1e-8
                        n     = to_obs / dist;                 % outward normal
                        buffer = orad + obj.safety_buffer;

                        % exponential repulsion
                        repel = exp(-0.5*(dist - buffer)) * n;

                        % tangential slide: velocity projected to tangent plane
                        t = vi - (vi.'*n)*n;
                        nt = norm(t);
                        if nt > 1e-8, t = t/nt; else, t = zeros(obj.Dim,1); end

                        f_beta = f_beta + (repel + 0.02*t);
                    end
                end
                f_beta = obj.C1_beta * f_beta;

                % Goal attraction
                to_goal = obj.Goal - pi;
                ng = norm(to_goal);
                f_gamma = obj.C1_gamma * to_goal / (ng + 1e-8);

                % Damping on velocity (appears in acceleration)
                a_i = f_alpha + align + f_beta + f_gamma - obj.damping_coeff * vi;

                % State derivative for agent i
                dX(sIdx(i):eIdx(i)) = obj.Agents(i).ODE(a_i);
            end
        end

        function Motion = Simulate(obj, dt, steps)
            t = (0:steps).' * dt;

            % Flatten Initial state
            X = reshape([obj.Agents.States], [], 1);

            Y = zeros(numel(t), numel(X));
            Y(1,:) = X.';

            stop_when  = ceil(obj.majority_percentage * obj.NumAgents);
            stopped_at = steps + 1;

            for k = 1:steps
                % derivative
                dX = obj.CollectiveODE(t(k), X);

                % Unpack
                tmp = 0:(obj.Dim*2):numel(X);
                sIdx = 1 + tmp(1:end-1);
                eIdx = tmp(2:end);

                for i = 1:obj.NumAgents
                    pi = X(sIdx(i):sIdx(i)+obj.Dim-1);
                    vi = X(sIdx(i)+obj.Dim:eIdx(i));

                    % Euler integration
                    pi = pi + dt * vi;
                    vi = vi + dt * dX(sIdx(i)+obj.Dim:eIdx(i));

                    % speed cap
                    sp = norm(vi);
                    if sp > obj.v_max
                        vi = vi * (obj.v_max / sp);
                    end

                    % keep inside arena [0, arena_size] for each coordinate
                    pi = min(max(pi, 0), obj.arena_size);

                    % write back
                    X(sIdx(i):sIdx(i)+obj.Dim-1) = pi;
                    X(sIdx(i)+obj.Dim:eIdx(i))   = vi;
                end

                Y(k+1,:) = X.';

                % early stop condition (majority within threshold of goal)
                cnt = 0;
                for i = 1:obj.NumAgents
                    pi = X(sIdx(i):sIdx(i)+obj.Dim-1);
                    if norm(pi - obj.Goal) < obj.threshold_distance
                        cnt = cnt + 1;
                    end
                end
                if cnt >= stop_when
                    stopped_at = k+1;
                    Y = Y(1:stopped_at, :);
                    t = t(1:stopped_at);
                    break;
                end
            end

            Motion.t = t;
            Motion.Y = Y;
        end

        function M = AdjacencyMat(obj)
            % neighbor graph (radius = r_alpha)
            M = false(obj.NumAgents);
            for i = 1:obj.NumAgents
                M(i,:) = obj.Agents(i).DistanceFromOthers(obj.Agents) <= obj.r_alpha;
            end
        end

        function MHist = AdjMatChange(obj, Motion)
            % Build adjacency history along a trajectory
            MHist = false(obj.NumAgents, obj.NumAgents, numel(Motion.t));
            tmp = 0:(obj.Dim*2):size(Motion.Y,2);
            sIdx = 1 + tmp(1:end-1);
            eIdx = tmp(2:end);

            for j = 1:numel(Motion.t)
                % set states at frame j
                X = Motion.Y(j,:).';
                for i = 1:obj.NumAgents
                    obj.Agents(i).States = X(sIdx(i):eIdx(i));
                end
                MHist(:,:,j) = obj.AdjacencyMat();
            end
        end
    end

    methods (Static)
        function y = sigma1(z)
            y = z ./ sqrt(1 + z.^2);
        end
        function b = bump(z_norm, h)
            if z_norm < h
                b = 1.0;
            elseif z_norm < 1.0
                b = 0.5 * (1.0 + cos(pi * (z_norm - h) / (1.0 - h)));
            else
                b = 0.0;
            end
        end
    end
end
