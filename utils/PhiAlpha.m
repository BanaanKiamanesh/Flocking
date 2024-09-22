function Val = PhiAlpha(z, R, D, Epsilon, A, B, C, H)
    % Equation 15 in the Paper
    Val = BumpFunc(z ./ SigmaNorm(R, Epsilon), H) .* Phi(z - SigmaNorm(D, Epsilon), A, B, C);
end