while (1)
    grabnewframe
    D = double(D);
    [Fx Fy] = gradient(D);
    absgrad = hypot(Fx, Fy);
    absgrad(absgrad > 30) = nan;
    image(absgrad*3);
    drawnow;
end