
%dbstop if error;

eqnarray = [];
sizearray = [];
hbitmap = false(size(XYZ,1),size(XYZ,2));
downsamp = 2;

%figure(1)
%imshow(double(Dp), [0 3000]);
%colormap('jet');
%figure(2)
%cla;
%axis equal;

%L = plot3(XYZ(:,:,1),XYZ(:,:,2),-XYZ(:,:,3), 'r.');
%set(L,'Markersize',1);

%HACK!!!!
% [eqn planesize hbitmap] = planefindfs([5*downsamp*10,5*downsamp*10], XYZ, downsamp, hbitmap);
% if planesize > 100
%                 eqnarray = [eqnarray eqn];
%                 sizearray = [sizearray planesize];
% end

for x = 12*downsamp:8*downsamp:640-12*downsamp
    for y = 12*downsamp:8*downsamp:480-12*downsamp
        if ~(hbitmap(x,y))
            [eqn planesize hbitmap] = planefindfs([x,y], XYZ, downsamp, hbitmap);
            if planesize > 100
                eqnarray = [eqnarray eqn];
                sizearray = [sizearray planesize];
            end
        end
    end
end

