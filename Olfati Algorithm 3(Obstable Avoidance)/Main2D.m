clear
close all
clc
format compact
format short

%% Parameters
Params.NumAgents = 20;
Params.Dim       = 2;

Params.arena_size = 800;
dt    = 0.05;
steps = 6000;

% Interaction radii / smoothness
Params.r_alpha = 50.0;       % neighbor radius
Params.d_alpha = 45.0;       % desired separation
Params.r_beta  = 100.0;      % obstacle influence radius
Params.Epsilon = 0.1;
Params.H       = 0.2;

% Gains
Params.C1_alpha = 1.5;       % cohesion/repulsion gain
Params.C2_alpha = 1.0;       % alignment gain
Params.C1_beta  = 6.0;       % obstacle gain
Params.C1_gamma = 2.0;       % goal attraction gain

% Motion limits / damping
Params.v_max         = 5.0;
Params.safety_buffer = 15.0;
Params.damping_coeff = 0.01;

% Stop rule
Params.threshold_distance  = 40.0;
Params.majority_percentage = 0.8;

% Goal and obstacles (two disks)
Params.Goal      = [650.0; 300.0];
Params.Obstacles = [400.0 250.0 30.0;
    400.0 350.0 30.0];

%% Init + Simulate
F = Flock(Params);
Motion = F.Simulate(dt, steps);
MHist  = F.AdjMatChange(Motion);

% MP4 Writer
videoFileName = 'motion_simulation_2D.mp4';
video = VideoWriter(videoFileName, 'MPEG-4');
video.FrameRate = 30;
video.Quality   = 100;
open(video);

%% Plotting
Pos = GetPos(Motion.Y(1,:), Params.Dim);
[Xl, Yl] = GetNeighbors(MHist(:,:,1), Pos);

figure('Name','Algorithm 3 – 2D Flocking','Units','normalized','OuterPosition',[0,0,1,1]);
P1 = plot(Xl, Yl, 'Color',[.6 .6 .6], 'DisplayName','Links'); hold on
P2 = scatter(Pos(:,1), Pos(:,2), 36, 'filled', 'DisplayName','Agents');

% Obstacles
th = linspace(0, 2*pi, 200);
for k = 1:size(Params.Obstacles,1)
    ox = Params.Obstacles(k,1); oy = Params.Obstacles(k,2); r = Params.Obstacles(k,3);
    plot(ox + r*cos(th), oy + r*sin(th), 'r-', 'LineWidth', 1, 'HandleVisibility', 'off');
end

% Goal
plot(Params.Goal(1), Params.Goal(2), 'k*', 'MarkerSize', 10, 'DisplayName','Goal');

axis equal
xlim([0, Params.arena_size])
ylim([0, 600])
grid on
xlabel('X')
ylabel('Y')
legend
title(sprintf('Algo 3 (2D)  |  t = %.1f s', Motion.t(1)))

% First frame
frame = getframe(gcf); writeVideo(video, frame);

%% Animate + Write Frames
for i = 2:5:numel(Motion.t)
    Pos = GetPos(Motion.Y(i,:), Params.Dim);
    P2.XData = Pos(:,1); P2.YData = Pos(:,2);

    [Xl, Yl] = GetNeighbors(MHist(:,:,i), Pos);
    P1.XData = Xl; P1.YData = Yl;

    title(sprintf('Algo 3 (2D)  |  t = %.1f s', Motion.t(i)))
    drawnow % limitrate

    frame = getframe(gcf);
    writeVideo(video, frame);
end

close(video);
disp(['Saved: ' videoFileName]);

%% Auxiliary Functions
function Pos = GetPos(X, Dims)
    Idx = 0:(2*Dims):numel(X);
    Pos = zeros(numel(X)/(2*Dims), Dims);
    for ii = 1:size(Pos,1)
        Pos(ii,:) = X(Idx(ii)+(1:Dims));
    end
end

function [X, Y] = GetNeighbors(Hist, Pos)
    N = size(Hist,1);
    L = sum(Hist,'all') - N;
    if L <= 0, X = NaN; Y = NaN; return; end
    X = zeros(L*3,1); Y = zeros(L*3,1);
    c = 1;
    for ii = 1:N
        for jj = 1:N
            if ii~=jj && Hist(ii,jj)
                X(c:c+2) = [Pos(ii,1); Pos(jj,1); NaN];
                Y(c:c+2) = [Pos(ii,2); Pos(jj,2); NaN];
                c = c + 3;
            end
        end
    end
end
