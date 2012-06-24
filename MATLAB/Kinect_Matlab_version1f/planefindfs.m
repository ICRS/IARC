function [ PlaneEQ planesize hbitmap ] = planefindfs( Seedpoint, XYZ, downsamp, inhbitmap) %, clearbitmap )
%PLANEFINDFS Summary of this function goes here
%   Detailed explanation goes here
hbitmap = inhbitmap;

quantfactors = [3.126e-06,-0.001557,3.709]; %quantization quadratic fit.
derateperdist = downsamp/300; %(x-y)mm per pixel per mm(z)
numsteperrallowed = 1.5;
deratefact = 1.5;

depththresh = numsteperrallowed * quantfactors;
xythresh = derateperdist * deratefact;

x = Seedpoint(1);
y = Seedpoint(2);

% bitmap of points in plane
%if clearbitmap
%    hbitmap = false(size(XYZ,1),size(XYZ,2));
%end
%if (~exist('hbitmap', 'var'))
%    hbitmap = zeros(size(XYZ,1),size(XYZ,2));
%end

%check if seedpoint has real data
if XYZ(x,y,3) == 0
    PlaneEQ = [nan nan nan];
    planesize = 0;
    return;
end

% set initial plane eqn - just random plane through chosen point -- bad
% idea, use below init.
%Coefficients = [ 0; 0; XYZ(x,y,3)];
%Coefficients = [-0.471147297884083;-0.219223020604994;2168.21317806309];
% Coefficients = [XCoeff; YCoeff; CCoeff]
% Using the above variables, z = XCoeff * x + YCoeff * y + CCoeff


% make list of points queued for processing
planepoints = zeros(100000,2);
planepoints(1,:) = [x y];

head = 1;
tail = 2;

%force adding of a 11x11 startsquare
%precalculate plane equation for first set to avoid finding lines instead
%of points
Xtemp = nan((8/downsamp)^2,1);
Ytemp = nan((8/downsamp)^2,1);
Ztemp = nan((8/downsamp)^2,1);
i = 1;
for dx = -8/downsamp:downsamp*2:8
    for dy = -8/downsamp:downsamp*2:8
        point = [x+dx y+dy];
        thePoint = [ XYZ(point(1),point(2),1) XYZ(point(1),point(2),2) XYZ(point(1),point(2),3)];
        Xtemp(i) = thePoint(1); 
        Ytemp(i) = thePoint(2); 
        Ztemp(i) = thePoint(3);
        i = i+1;
    end
end

Const = ones(size(Xtemp)); % Vector of ones for constant term
Coefficients = [Xtemp Ytemp Const]\Ztemp; % Find the coefficients

%preallocate for first iteration
Xcolv = nan(8,1);
Ycolv = nan(8,1);
Zcolv = nan(8,1);

numPointsProcessed = 0;
numPointsThreshold = 8;
numPointsPerimter = 0;
t = cputime; 

while head~=tail
    while numPointsProcessed < numPointsThreshold && head~=tail
        point = [ planepoints(head,1) planepoints(head,2) ];
        
            % calculate error
            thePoint = [ XYZ(point(1),point(2),1) XYZ(point(1),point(2),2) XYZ(point(1),point(2),3)];
            dzdx = Coefficients(1);
            dzdy = Coefficients(2);
            candZerr = thePoint(1) * dzdx + thePoint(2) * dzdy + Coefficients(3) - thePoint(3);

            if (abs(candZerr)< (thePoint(3)*xythresh*(abs(dzdx) + abs(dzdy))) + depththresh*[thePoint(3)^2 thePoint(3) 1]')  %else for this dude makes the shape outline
                % yay - add point
                numPointsProcessed = numPointsProcessed+1;

                % add neighbours
                if point(1)>downsamp && point(1)<=size(XYZ,1)-downsamp && point(2)>downsamp && point(2)<=size(XYZ,2)-downsamp
                        
                    if ~hbitmap(point(1)+downsamp, point(2))
                        planepoints(tail,:) = [point(1)+downsamp point(2)];
                        hbitmap(point(1)+downsamp, point(2)) = true;
                        tail = tail + 1;
                    end

                    if ~hbitmap(point(1), point(2)+downsamp)
                        planepoints(tail,:) = [point(1) point(2)+downsamp];
                        hbitmap(point(1), point(2)+downsamp) = true;
                        tail = tail + 1;
                    end

                    if ~hbitmap(point(1)-downsamp, point(2))
                        planepoints(tail,:) = [point(1)-downsamp point(2)];
                        hbitmap(point(1)-downsamp, point(2)) = true;
                        tail = tail + 1;
                    end

                    if ~hbitmap(point(1), point(2)-downsamp)
                        planepoints(tail,:) = [point(1) point(2)-downsamp];
                        hbitmap(point(1), point(2)-downsamp) = true;
                        tail = tail + 1;
                    end
                end


                if tail>99000
                    planepoints=planepoints(head:tail,:);
                    tail = tail-head+1;
                    head = 1;
                    planepoints=[ planepoints; zeros(100000-tail,2) ];
                end


                Xcolv(numPointsProcessed) = thePoint(1); 
                Ycolv(numPointsProcessed) = thePoint(2); 
                Zcolv(numPointsProcessed) = thePoint(3); 

            else
                numPointsPerimter = numPointsPerimter + 1;
            end
            %hbitmap(point(1),point(2))=1;
        %end


        % remove from todo list
        head = head+1;

    end
    
    if head==tail
        break;
    end
    
    Const = ones(size(Xcolv)); % Vector of ones for constant term
    Coefficients = [Xcolv Ycolv Const]\Zcolv; % Find the coefficients     
    
    %if fringe edge is deminishing, abort (as this is a symptom of
    %bleeding)
    %if numPointsProcessed >= 256 && (tail-head) < 20
    %    break;
    %end
        

    
    %if plane aspect ratio is shit, dont use (as these end up being lines
    %not planes)
    if ((numPointsPerimter/5)^2 > numPointsProcessed)
        highaspect = true;
        hbitmap = inhbitmap;
        PlaneEQ = [nan nan nan];
        planesize = 0;
        return;
    end
    
    Xcolv = [Xcolv; nan(numPointsThreshold,1)]; 
    Ycolv = [Ycolv; nan(numPointsThreshold,1)];
    Zcolv = [Zcolv; nan(numPointsThreshold,1)];
    
    numPointsThreshold = numPointsThreshold*2;
                
end

PlaneEQ = Coefficients;
planesize = numPointsProcessed;
%e = cputime-t

XCoeff = Coefficients(1); % X coefficient
YCoeff = Coefficients(2); % X coefficient
CCoeff = Coefficients(3); % constant term
% Using the above variables, z = XCoeff * x + YCoeff * y + CCoeff

%look only for "large" planes
if CCoeff <= 0 || planesize < 5000/(downsamp^2) % * XYZ(Seedpoint(1), Seedpoint(2), 3)/1000); %This is to track only ABSOLUTLY large planes.
    hbitmap = inhbitmap;
    PlaneEQ = [nan nan nan];
    planesize = 0;
    return;
end
if CCoeff >= 10000
    wtf =  'wtf'
end

%comment return to draw stuff
%return;

%generate colorspace for plotting with random colors
cmap = jet(10);

figure(2)
hold on;
L=plot3(Xcolv,Zcolv,Ycolv,'.'); % Plot the original data points
%set(L,'Markersize',0.2*get(L,'Markersize')) % Making the circle markers larger
set(L,'MarkerEdgeColor',cmap(mod(int32(CCoeff*2),10)+1,:)) % Filling in themarkers

[xx, yy]=meshgrid(min(Xcolv):10:max(Xcolv),min(Ycolv):10:max(Ycolv)); % Generating a regular grid for plotting
zz = XCoeff * xx + YCoeff * yy + CCoeff;
%mesh(xx,-zz,yy) % Plotting the surface
%alpha(0);
title(sprintf('Plotting plane z=(%f)*x+(%f)*y+(%f)',XCoeff, YCoeff, CCoeff))
hold off;

%figure(2)
%hold on
%L=plot3(Xcolv,Ycolv,-Zcolv,'.'); % Plot the original data points
%set(L,'Markersize',0.2*get(L,'Markersize')) % Making the circle markers larger
%set(L,'MarkerEdgeColor',cmap(mod(int32(CCoeff),10)+1,:)) % Filling in themarkers
%hold off

% figure(2)
% X=[];
% Y=[];
% Z=[];
% X(:,:) = XYZ(:,:,1);
% Y(:,:) = XYZ(:,:,2);
% Z(:,:) = XYZ(:,:,3);
% Z(Z==0)=nan;
% mesh(X,Y,3000-Z)


end

