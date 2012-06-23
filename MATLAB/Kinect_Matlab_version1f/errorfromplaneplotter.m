while(1)

    grabnewframe;
    planeenumerator;
    [C,I] = max(sizearray);
    eqn = eqnarray(:,I);
    
    if numel(eqn)==0
        continue
    end

    deltadepth = nan(640, 480);

    for x = 1:640
        for y = 1:480
            deltadepth(x,y) = XYZ(x,y,3) - eqn(1)*XYZ(x,y,1) - eqn(2)*XYZ(x,y,2) - eqn(3);
        end
    end

    deltadepth(abs(deltadepth) > 50) = nan;
    deltadepth = permute(deltadepth,[2 1]);

    image(abs(deltadepth*3));
    drawnow;
end