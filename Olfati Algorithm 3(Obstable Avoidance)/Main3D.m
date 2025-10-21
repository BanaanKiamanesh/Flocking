clear
close all
clc
format compact
format short

%% Parameters
Params.NumAgents = 20;
Params.Dim       = 3;

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
Params.C1_alpha = 1.5;       % cohesion/repulsion
Params.C2_alpha = 1.0;       % alignment
Params.C1_beta  = 6.0;       % obstacle
Params.C1_gamma = 2.0;       % goal

% Motion limits / damping
Params.v_max         = 5.0;
Params.safety_buffer = 15.0;
Params.damping_coeff = 0.01;

% Stop rule
Params.threshold_distance  = 40.0;
Params.majority_percentage = 0.8;

% Goal and obstacles (two spheres)
Params.Goal      = [650.0; 300.0; 300.0];
Params.Obstacles = [650.0 200.0 300.0 50.0;   % [x y z r]
    650.0 400.0 300.0 50.0];

%% Init + Simulate
F = Flock(Params);
Motion = F.Simulate(dt, steps);
MHist  = F.AdjMatChange(Motion);

% MP4 Writer
videoFileName = 'motion_simulation_3D.mp4';
video = VideoWriter(videoFileName, 'MPEG-4');
video.FrameRate = 30;
video.Quality   = 100;
open(video);

%% Plotting
Pos = GetPos(Motion.Y(1,:), Params.Dim);
[Xl, Yl, Zl] = GetNeighbors3D(MHist(:,:,1), Pos);

figure('Name','Algorithm 3 – 3D Flocking','Units','normalized','OuterPosition',[0,0,1,1]);
ax = axes; hold(ax, 'on');

P1 = plot3(Xl, Yl, Zl, 'Color',[.6 .6 .6], 'DisplayName','Links');
P2 = scatter3(Pos(:,1), Pos(:,2), Pos(:,3), 36, 'filled', 'DisplayName','Agents');

% Obstacles
[XS, YS, ZS] = sphere(24);
for k = 1:size(Params.Obstacles,1)
    oc = Params.Obstacles(k,1:3);
    r  = Params.Obstacles(k,4);
    surf(oc(1) + r*XS, oc(2) + r*YS, oc(3) + r*ZS, ...
        'EdgeColor','none','FaceAlpha',0.12,'FaceColor',[1 0 0], ...
        'HandleVisibility','off');
end

% Goal
plot3(Params.Goal(1), Params.Goal(2), Params.Goal(3), 'k*', 'MarkerSize', 10, 'DisplayName','Goal');

axis vis3d
xlim([0, Params.arena_size]); ylim([0, Params.arena_size]); zlim([0, Params.arena_size]);
grid on
xlabel('X'); ylabel('Y'); zlabel('Z'); legend
view(3)
title(sprintf('Algo 3 (3D)  |  t = %.1f s', Motion.t(1)))

% First frame
drawnow
frame = getframe(gcf); writeVideo(video, frame);

%% Animate + Write Frames
for i = 2:5:numel(Motion.t)
    Pos = GetPos(Motion.Y(i,:), Params.Dim);
    P2.XData = Pos(:,1); P2.YData = Pos(:,2); P2.ZData = Pos(:,3);

    [Xl, Yl, Zl] = GetNeighbors3D(MHist(:,:,i), Pos);
    set(P1, 'XData', Xl, 'YData', Yl, 'ZData', Zl);

    title(sprintf('Algo 3 (3D)  |  t = %.1f s', Motion.t(i)))
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

function [X, Y, Z] = GetNeighbors3D(Hist, Pos)
    % Build NaN-separated line strips for plot3
    N = size(Hist,1);
    L = sum(Hist,'all') - N;
    if L <= 0, X = NaN; Y = NaN; Z = NaN; return; end
    X = zeros(L*3,1); Y = zeros(L*3,1); Z = zeros(L*3,1);
    c = 1;
    for ii = 1:N
        for jj = 1:N
            if ii~=jj && Hist(ii,jj)
                X(c:c+2) = [Pos(ii,1); Pos(jj,1); NaN];
                Y(c:c+2) = [Pos(ii,2); Pos(jj,2); NaN];
                Z(c:c+2) = [Pos(ii,3); Pos(jj,3); NaN];
                c = c + 3;
            end
        end
    end
end
