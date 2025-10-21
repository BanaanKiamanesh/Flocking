% CR_FinalProject.m  (No GIF, No File Saving — just show plots)
% Multi-Agent Flocking (Olfati-Saber Alg. 3 style) with obstacle avoidance and goal seeking

%% Parameters
rng(42);
N          = 100;
arena_w    = 800;     % width (x)
arena_h    = 600;     % height (y)
dt         = 0.05;
steps      = 6000;

% Interactions
r_alpha    = 50.0;
d_alpha    = 45.0;
r_beta     = 100.0;
h_alpha    = 0.2;

% Force coefficients
c1_alpha   = 1.5;
c2_alpha   = 1.0;
c1_beta    = 6.0;
c1_gamma   = 2.0;

v_max         = 5.0;
safety_buffer = 15.0;
damping_coeff = 0.01;
threshold_dist = 40.0;
majority_pct   = 0.8;

goal = [650.0, 300.0];

% Obstacles: [cx, cy, r]
obstacles = [
    400.0, 250.0, 30.0;
    400.0, 350.0, 30.0
];

%% Initialization
positions  = 200 * rand(N, 2);   % bottom-left 0..200
velocities = zeros(N, 2);
accelerations = zeros(N, 2);
trajectory = zeros(steps, N, 2); % will trim later

[accelerations, ~] = compute_acceleration_vec(positions, velocities, goal, obstacles, ...
    r_alpha, d_alpha, h_alpha, r_beta, safety_buffer, c1_alpha, c2_alpha, c1_beta, c1_gamma);

%% Figure 1: Initial positions & neighbor connections
figure('Color','w','Position',[100 100 900 720]);
scatter(positions(:,1), positions(:,2), 30, 'b', '^', 'filled'); hold on; grid on;
title('Initial Positions and Neighbor Connections'); axis equal;
xlim([0 arena_w]); ylim([0 arena_h]);

dt_tri = delaunayTriangulation(positions);
tri    = dt_tri.ConnectivityList;
edges  = tri_to_edges(tri);
plot_edges(positions, edges, [0.7 0.7 0.7]);

% Obstacles & goal
for k = 1:size(obstacles,1)
    draw_circle(obstacles(k,1), obstacles(k,2), obstacles(k,3), [1 0 0], 0.5);
end
plot(goal(1), goal(2), 'r*', 'MarkerSize', 12, 'LineWidth', 1.5);

%% Simulation (no frame capture)
stop_agents     = false;
capture_stride  = 20;     % used only for connectivity sampling
conn_over_time  = [];     % average degree samples

for t = 1:steps
    % Goal-reach check
    d2g = sqrt(sum((positions - goal).^2, 2));
    if sum(d2g < threshold_dist) >= majority_pct * N
        stop_agents = true;
    end

    % Velocity-Verlet: position update
    positions = positions + velocities * dt + 0.5 * accelerations * dt^2;
    % Clip to arena
    positions(:,1) = min(max(positions(:,1), 0), arena_w);
    positions(:,2) = min(max(positions(:,2), 0), arena_h);

    % New accelerations
    [new_accel, ~] = compute_acceleration_vec(positions, velocities, goal, obstacles, ...
        r_alpha, d_alpha, h_alpha, r_beta, safety_buffer, c1_alpha, c2_alpha, c1_beta, c1_gamma);

    % Velocity update
    velocities = velocities + 0.5 * (accelerations + new_accel) * dt;

    % Damping + speed cap
    velocities = (1 - damping_coeff) * velocities;
    speeds = sqrt(sum(velocities.^2, 2));
    over = speeds > v_max;
    if any(over)
        velocities(over,:) = bsxfun(@times, velocities(over,:), v_max ./ speeds(over));
    end

    accelerations = new_accel;
    trajectory(t,:,:) = positions;

    % Connectivity sample (average degree over Delaunay)
    if mod(t, capture_stride) == 0
        dt_tri = delaunayTriangulation(positions);
        tri    = dt_tri.ConnectivityList;
        edges  = tri_to_edges(tri);

        % Build adjacency and compute average degree
        A = false(N,N);
        for e = 1:size(edges,1)
            i = edges(e,1); j = edges(e,2);
            A(i,j) = true; A(j,i) = true;
        end
        degs = sum(A,2);
        conn_over_time(end+1) = mean(degs); %#ok<AGROW>
    end

    % Early stop once at goal and settled
    if stop_agents && all(speeds < 0.05)
        fprintf('Simulation stopped early at step %d.\n', t);
        trajectory = trajectory(1:t,:,:);
        break;
    end

    if t == steps
        trajectory = trajectory(1:t,:,:);
    end
end

%% Derive velocities + smoothing (for plots)
T = size(trajectory,1);
vel_array = zeros(T, N, 2);
vel_array(1:end-1,:,:) = diff(trajectory,1,1) / dt;
vel_array(end,:,:)     = vel_array(end-1,:,:);

vel_mag = sqrt( sum( vel_array.^2, 3 ) );

window = 10;
kernel = ones(1,window)/window;
smoothed_vel = zeros(T, N);
for i = 1:N
    smoothed_vel(:,i) = conv(vel_mag(:,i), kernel, 'same');
end

%% Figure 2: Trajectories
figure('Color','w','Position',[100 100 900 720]); hold on; grid on; axis equal;
for i = 1:N
    traj = squeeze(trajectory(:,i,:));
    plot(traj(:,1), traj(:,2), 'LineWidth', 1, 'Color', [0 0 1 0.6]);
end
scatter(goal(1), goal(2), 200, 'r', '*', 'filled');
for k = 1:size(obstacles,1)
    draw_circle(obstacles(k,1), obstacles(k,2), obstacles(k,3), [1 0 0], 0.5);
end
title('Agent Trajectories');
xlabel('X Position'); ylabel('Y Position');
xlim([0 arena_w]); ylim([0 arena_h]);

%% Figure 3: Smoothed velocity magnitudes
figure('Color','w','Position',[100 100 900 540]); hold on; grid on;
for i = 1:N
    plot(smoothed_vel(:,i), 'LineWidth', 0.75, 'Color', [0 0 0 0.35]);
end
title('Smoothed Velocity Magnitudes Over Time');
xlabel('Time Steps'); ylabel('Velocity Magnitude');

%% Figure 4: Smoothed X components
figure('Color','w','Position',[100 100 900 540]); hold on; grid on;
time_steps = (1:T).';
for i = 1:N
    vx = squeeze(vel_array(:,i,1));
    vx_s = conv(vx, kernel, 'same');
    plot(time_steps, vx_s, 'LineWidth', 0.75, 'Color', [0 0 0 0.35]);
end
title('Smoothed X Velocity Components Over Time');
xlabel('Time Steps'); ylabel('X Velocity');

%% Figure 5: Smoothed Y components
figure('Color','w','Position',[100 100 900 540]); hold on; grid on;
for i = 1:N
    vy = squeeze(vel_array(:,i,2));
    vy_s = conv(vy, kernel, 'same');
    plot(time_steps, vy_s, 'LineWidth', 0.75, 'Color', [0 0 0 0.35]);
end
title('Smoothed Y Velocity Components Over Time');
xlabel('Time Steps'); ylabel('Y Velocity');

%% Figure 6: Average connectivity over time (samples)
figure('Color','w','Position',[100 100 900 540]);
plot(conn_over_time, 'LineWidth', 2);
grid on; title('Average Connectivity Over Time (sampled)');
xlabel(sprintf('Samples (every %d steps)', capture_stride));
ylabel('Average Number of Connections');

disp('Done: plots displayed (no files saved).');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Local functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function val = sigma1(z)
    val = z ./ sqrt(1 + z.^2);
end

function B = bump(Z, h)
    B = zeros(size(Z));
    m1 = Z < h;
    m2 = (Z >= h) & (Z < 1);
    B(m1) = 1.0;
    if any(m2(:))
        B(m2) = 0.5 * (1 + cos(pi * (Z(m2) - h) ./ (1 - h)));
    end
end

function [acc, near_flags] = compute_acceleration_vec(pos, vel, goal, obs, ...
    r_alpha, d_alpha, h_alpha, r_beta, safety_buffer, c1_alpha, c2_alpha, c1_beta, c1_gamma)

    N = size(pos,1);

    % Pairwise differences X_j - X_i (no implicit expansion)
    DX = bsxfun(@minus, pos(:,1)', pos(:,1));   % N x N
    DY = bsxfun(@minus, pos(:,2)', pos(:,2));   % N x N
    D  = sqrt(DX.^2 + DY.^2);
    D(1:N+1:end) = inf;                         % no self

    % Bump & phi
    B = bump(D / r_alpha, h_alpha);
    phi_shift = sigma1(r_alpha - d_alpha);
    PHI = B .* ( sigma1(D - d_alpha) - phi_shift );

    % Unit directions
    Ux = DX ./ D;
    Uy = DY ./ D;
    Ux(1:N+1:end) = 0;
    Uy(1:N+1:end) = 0;

    % α spacing force
    f_alpha_x = c1_alpha * sum(PHI .* Ux, 2);
    f_alpha_y = c1_alpha * sum(PHI .* Uy, 2);
    f_alpha   = [f_alpha_x, f_alpha_y];

    % Alignment: B*V - diag(sum(B,2))*V
    sumB = sum(B, 2);                        % N x 1
    align = c2_alpha * (B * vel - bsxfun(@times, vel, sumB));

    % γ: unit pull to goal
    to_goal = goal - pos;                    % N x 2
    ng = sqrt(sum(to_goal.^2,2)) + eps;
    f_gamma = bsxfun(@times, to_goal, c1_gamma ./ ng);

    % β: obstacles (loop over few obstacles only)
    f_beta = zeros(N,2);
    near_flags = false(N,1);
    for k = 1:size(obs,1)
        to_obs = bsxfun(@minus, pos, obs(k,1:2));
        dist   = sqrt(sum(to_obs.^2,2)) + eps;
        buffer = obs(k,3) + safety_buffer;

        m = dist < r_beta;
        if any(m)
            repel_mag = exp(-0.5 * (dist(m) - buffer));
            repel_dir = bsxfun(@rdivide, to_obs(m,:), dist(m));
            repel = bsxfun(@times, repel_dir, repel_mag);

            orth = [-to_obs(m,2), to_obs(m,1)];
            orth = bsxfun(@rdivide, orth, dist(m));

            f_beta(m,:) = f_beta(m,:) + (repel + 0.02 * orth);
        end
        near_flags = near_flags | (dist < obs(k,3) + 1.25 * safety_buffer);
    end

    acc = f_alpha + align + c1_beta * f_beta + f_gamma;
end

function edges = tri_to_edges(tri)
    if isempty(tri), edges = zeros(0,2); return; end
    e = [tri(:,[1 2]); tri(:,[2 3]); tri(:,[3 1])];
    e = sort(e, 2);
    edges = unique(e, 'rows');
end

function plot_edges(P, edges, colorRGB)
    if isempty(edges), return; end
    X = [P(edges(:,1),1) P(edges(:,2),1)]';
    Y = [P(edges(:,1),2) P(edges(:,2),2)]';
    line(X, Y, 'Color', colorRGB, 'LineWidth', 0.5);
end

function draw_circle(cx, cy, r, colorRGB, alphaVal)
    if nargin < 5, alphaVal = 0.5; end
    rectangle('Position',[cx-r, cy-r, 2*r, 2*r], ...
              'Curvature',[1 1], ...
              'FaceColor',colorRGB, ...
              'FaceAlpha',alphaVal, ...
              'EdgeColor','none');
end
