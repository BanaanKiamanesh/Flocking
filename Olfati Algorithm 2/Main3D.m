clear
close all
clc
% rng(123)
format compact
format short
addpath("../utils");

%% Parameter Declaration

Params.A = 5;
Params.B = 5;
Params.C = abs(Params.A - Params.B) / sqrt(4 * Params.A*Params.B);
Params.H = 0.2;

Params.Radius   = 20;      % Perception Radius
Params.Distance = 18;      % Distance of Agents
Params.Epsilon  = 0.1;

Params.C1_alpha = 3;
Params.C2_alpha = 2 * sqrt(Params.C1_alpha);
Params.C1_gamma = 5;
Params.C2_gamma = 0.2 * sqrt(Params.C1_gamma);

Params.NumAgents = 200;
Params.Dim = 3;

%% Init
F = Flock(Params);

%% Simulate
SimTime = 120;
Motion  = F.Simulate(SimTime, @Traj3D);
MHist   = F.AdjMatChange(Motion);
% save Motion.mat MHist Motion F Params

% clear;close all;clc
% load Motion.mat

%% Set up video writer
videoFileName = 'motion_simulation.mp4';
video = VideoWriter(videoFileName, 'MPEG-4');
video.FrameRate = 30;
video.Quality = 100;
open(video);

%% Plot Motion
Pos = GetPos(Motion.Y(end, :), Params.Dim);
[X, Y, Z] = GetNeighbors(MHist(:, :, end), Pos);

figure('Name', 'Agents', 'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
P1 = plot3(X, Y, Z, 'DisplayName', 'Links'); hold on
P2 = scatter3(Pos(:, 1), Pos(:, 2), Pos(:, 3), "filled", "ok", 'DisplayName', "Agents");
Traj = Traj3D(Motion.t(1));
TrackPoint = scatter3(Traj(1), Traj(2), Traj(3), 150, 'filled', 'DisplayName', 'TrackPoint');

axis equal
xlim([-1, 1] * 150)
ylim([-1, 1] * 150)
zlim([-1, 1] * 150)
xlabel("X")
ylabel("Y")
zlabel("Z")
grid on
legend("Position", [0.7742 0.8439 0.0572 0.0537])
title(["Double Integrator Agents with Distance = ", num2str(Params.Distance)])

for i = 2:5:numel(Motion.t)
    Pos = GetPos(Motion.Y(i, :), Params.Dim);
    P2.XData = Pos(:, 1);
    P2.YData = Pos(:, 2);
    P2.ZData = Pos(:, 3);

    [X, Y, Z] = GetNeighbors(MHist(:, :, i), Pos);
    P1.XData = X;
    P1.YData = Y;
    P1.ZData = Z;

    Traj = Traj3D(Motion.t(i));
    TrackPoint.XData = Traj(1);
    TrackPoint.YData = Traj(2);
    TrackPoint.ZData = Traj(3);

    frame = getframe(gcf);
    writeVideo(video, frame);

    title(['Double Integrator Agents with Distance = ', num2str(Params.Distance), ' Time = ', num2str(round(Motion.t(i), 1))])
    view(i/5, 30)

    drawnow limitrate
end
close(video);

%% Check Stability
% figure
% plot(Motion.t, Motion.Y)

function Pos = GetPos(X, Dims)

    Idx = 0:(2*Dims):numel(X);
    StartIdx = Idx(1:end - 1);

    Pos = zeros(numel(X) / Dims / 2, Dims);
    for i = 1:size(Pos, 1)
        Pos(i, :) = X(StartIdx(i) + (1:Dims));
    end
end

function [X, Y, Z] = GetNeighbors(Hist, Pos)
    NumAgents = length(Hist);
    NumLines = sum(Hist, 'all') - NumAgents;

    X = zeros(NumLines * 2, 1);
    Y = zeros(NumLines * 2, 1);
    Z = zeros(NumLines * 2, 1);
    
    Cnt = 1;
    for i = 1:NumAgents
        for j = 1:NumAgents
            if (i ~= j) && Hist(i, j)
                X(Cnt:Cnt + 2) = [Pos(i, 1), Pos(j, 1), NaN];
                Y(Cnt:Cnt + 2) = [Pos(i, 2), Pos(j, 2), NaN];
                Z(Cnt:Cnt + 2) = [Pos(i, 3), Pos(j, 3), NaN];
    
                Cnt = Cnt + 3;
            end
        end
    end

    if isempty(X) || isempty(Y) || isempty(Z)
        X = NaN;
        Y = NaN;
        Z = NaN;
    end
end