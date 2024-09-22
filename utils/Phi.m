function Val = Phi(z, A, B, C)
    % Equation 15 in the Paper
    Val = (1/2) * ((A + B) * Sigma1(z + C)+ (A-B));
end