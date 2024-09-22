function Val = SigmaNorm(z, Epsilon)
    % Equation 8 in the Paper
    Val = (1/Epsilon) * (sqrt(1 + Epsilon * vecnorm(z, 2, 1).^2) - 1);
end