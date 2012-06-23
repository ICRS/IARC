downsamp = 2;
coeff = nan(64,48,3);

for x = 0:62
    for y = 0:46
        
        pointlist = nan((20/downsamp)^2,3);
        i = 1;
        
        for dx = 1:downsamp:20
            for dy = 1:downsamp:20
                pointlist(i,:) = XYZ(x+dx, y+dy,:);
                i = i + 1;
            end
        end
        
        const = ones((20/downsamp)^2,1);
        coeff(x+1,y+1,:) = [pointlist(:,1) pointlist(:,2) const]\pointlist(:,3);
        
    end
end