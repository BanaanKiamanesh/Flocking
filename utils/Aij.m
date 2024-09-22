function Val = Aij(Qi, Qj, Epsilon, Radius, H)
    % Defined Under Equation 11 in the Paper
    Val = BumpFunc(SigmaNorm(Qj - Qi, Epsilon) ./ SigmaNorm(Radius, Epsilon), H);
end