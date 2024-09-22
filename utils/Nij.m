function Val = Nij(Qi, Qj, Epsilon)
    % Defined Under Equation 23 in the Paper
    Val = SigmaNormGrad(Qj - Qi, Epsilon);
end