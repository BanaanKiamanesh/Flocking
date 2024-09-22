function Val = SigmaNormGrad(z, Epsilon)
    % Equation 9 in the Paper
    Val = z ./ sqrt(1 + Epsilon * vecnorm(z, 2, 1).^2);
end