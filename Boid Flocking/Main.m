clear
close all
clc

%% Flock Params
N = 20;

Ksep  = 0.05;
Kali  = 0.1;
Kcoh  = 0.1;
Kgoal = 0.1;

Pgoal1 = MVector( 1,  0, 0);
Pgoal2 = MVector( 0,  1, 0);
Pgoal3 = MVector(-1,  0, 0);
Pgoal4 = MVector( 0, -1, 0);
Pgoal5 = MVector( 1,  0, 0);

Vgoal = MVector(0, 0, 0);

% Trajectory: {time_threshold, Pgoal, Vgoal}
Traj = {{100, Pgoal1, Vgoal}, ...
        {200, Pgoal2, Vgoal}, ...
        {300, Pgoal3, Vgoal}, ...
        {400, Pgoal4, Vgoal}, ...
        {500, Pgoal5, Vgoal}};

%% Simulate Flock
F = Flock(N, Ksep, Kali, Kcoh, Kgoal);
[t, X] = F.Simulate(500, 0.5, Traj);

%% Plotting
s = scatter3(NaN(N, 1), NaN(N, 1), NaN(N, 1));
Room = [-1, 1, -1, 1, -1, 1] * 4;
axis(Room);
grid on
xlabel('x'); ylabel('y'); zlabel('z');

xline(0, '--k', 'Alpha', 0.5);
yline(0, '--k', 'Alpha', 0.5);

vout = VideoWriter('flock.mp4', 'MPEG-4');
open(vout);

for i = 1:size(X, 3)
    x = X(1, :, i);
    y = X(2, :, i);
    z = X(3, :, i);

    s.XData = x;
    s.YData = y;
    s.ZData = z;

    drawnow
    f = getframe(gcf);
    writeVideo(vout, f);
end

close(vout);
