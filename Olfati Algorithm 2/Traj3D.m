function [P, Q] = Traj3D(t)
    if t < 30
        P = [50; 50; 50];
        Q = [0; 0; 0];
    elseif t < 60
        P = [-50; -50; 50];
        Q = [0; 0; 0];
    elseif t < 90
        P = [50; -50; 50];
        Q = [0; 0; 0];
    else 
        P = [0; 0; 0];
        Q = [0; 0; 0];
    end
end