function [P, Q] = Traj2D(t)
    % if t < 30
    %     P = [50; 50];
    %     Q = [0; 0];
    % elseif t < 60
    %     P = [-50; 50];
    %     Q = [0; 0];
    % elseif t < 90
    %     P = [50; -50];
    %     Q = [0; 0];
    % else
    %     P = [0; 0];
    %     Q = [0; 0];
    % end

    % Circle Traj
    StartTime = 8*pi;
    if t < StartTime
        P = [cos(StartTime / 5) * 50; sin(StartTime / 5) * 50];
        Q = [0; 0];
    else
        P = [cos(t/5) * 50; sin(t/5) * 50];
        Q = [0; 0];
    end
end