function D = Distance(Agent1, Agent2)
    % Distance of 2 Agents
    D = norm(Agent1.States - Agent2.States);
end