function Ph = BumpFunc(z, H)
    % Equation 10 in the Paper

    Ph = zeros(size(z));

    % Z in [0, h)
    Ph(z>=0 & z<H) = 1;

    % Z in [h, 1]
    Ph(z>=H & z<=1) = (1/2) * (1 + cos(pi * (z(z>=H & z<=1) - H) / (1 - H)));

    % otherwise (already satisfied)
end