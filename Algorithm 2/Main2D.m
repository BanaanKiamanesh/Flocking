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

Params.Radius   = 12;      % Perception Radius
Params.Distance = 10;      % Distance of Agents
Params.Epsilon  = 0.1;

Params.C1_alpha = 3;
Params.C2_alpha = 2 * sqrt(Params.C1_alpha);
Params.C1_gamma = 5;
Params.C2_gamma = 0.2 * sqrt(Params.C1_gamma);

Params.NumAgents = 250;
Params.Dim = 2;

%% Init
F = Flock(Params);

%% Simulate
SimTime = 50;
Motion  = F.Simulate(SimTime);
MHist   = F.AdjMatChange(Motion);
% save Motion.mat MHist Motion F Params

%% Set up video writer
videoFileName = 'motion_simulation.mp4';
video = VideoWriter(videoFileName, 'MPEG-4');
video.FrameRate = 30;
video.Quality = 100;
open(video);

%% Plot Motion
Pos = GetPos(Motion.Y(end, :), Params.Dim);
[X, Y] = GetNeighbors(MHist(:, :, end), Pos);

figure('Name', 'Agents', 'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
P1 = plot(X, Y); hold on
P2 = scatter(Pos(:, 1), Pos(:, 2), "filled");
axis([-1 1, -1, 1] * 100)
axis equal
xlabel("X")
ylabel("Y")
title(["Double Integrator Agents with Distance = ", num2str(Params.Distance)])

for i = 2:5:numel(Motion.t)
    Pos = GetPos(Motion.Y(i, :), Params.Dim);
    P2.XData = Pos(:, 1);
    P2.YData = Pos(:, 2);

    [X, Y] = GetNeighbors(MHist(:, :, i), Pos);
    P1.XData = X;
    P1.YData = Y;

    frame = getframe(gcf);
    writeVideo(video, frame);

    title(['Double Integrator Agents with Distance = ', num2str(Params.Distance), ' Time = ', num2str(round(Motion.t(i), 1))])

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

function [X, Y] = GetNeighbors(Hist, Pos)
    NumAgents = length(Hist);
    NumLines = sum(Hist, 'all') - NumAgents;

    X = zeros(NumLines * 2, 1);
    Y = zeros(NumLines * 2, 1);
    Cnt = 1;
    for i = 1:NumAgents
        for j = 1:NumAgents
            if (i ~= j) && Hist(i, j)
                X(Cnt:Cnt + 2) = [Pos(i, 1), Pos(j, 1), NaN];
                Y(Cnt:Cnt + 2) = [Pos(i, 2), Pos(j, 2), NaN];
                Cnt = Cnt + 3;
            end
        end
    end
end