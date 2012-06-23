addpath('Mex')
SAMPLE_XML_PATH='Config/SamplesConfig.xml';

% Start the Kinect Process
filename='Example/SkelShort.oni';
%KinectHandles=mxNiCreateContext(SAMPLE_XML_PATH,filename);

% To use the Kinect hardware use :
KinectHandles=mxNiCreateContext(SAMPLE_XML_PATH);

downsamp = 2;
kernelsize = 10;

D=mxNiDepth(KinectHandles); %D=permute(D,[2 1]);
subplot(1,3,1); h2=imshow(D,[0 6000]); colormap('jet');
normmap = zeros((640-kernelsize*2)/downsamp, (480-kernelsize*2)/downsamp, 3);
traces = zeros((640-kernelsize*2)/downsamp, (480-kernelsize*2)/downsamp);
subplot(1,3,2); h3=image(normmap/2+0.5);
axis equal;
subplot(1,3,3); h4=image(traces);
axis equal;

H = fspecial('gaussian', 10, 1);

while(1)
    %covmat = zeros(639-4, 479-4, 3, 3);
    
    XYZ=mxNiDepthRealWorld(KinectHandles);
    %XYZ(XYZ(:,:,3)==0)=nan;
    filtZ = conv2(XYZ(:,:,3), H);
    XYZ(:,:,3) = filtZ(5:640+5-1, 5:480+5-1);
    
    for x = kernelsize:(640-kernelsize)/downsamp
        for y = kernelsize:(480-kernelsize)/downsamp
            
            tempdata = zeros((kernelsize+1)^2,3);
            i = 1;
            
            for dx = -kernelsize:kernelsize
                for dy = -kernelsize:kernelsize
                  tempdata(i,:) = XYZ(x*downsamp+dx,y*downsamp+dy,:);
                  if tempdata(i,3) == 0
                      tempdata(i,:) = [];
                  end
                  %tempdata(i,:) = XYZfilt(x*downsamp+dx,y*downsamp+dy,:);
                  i = i + 1;
                end
            end
            
            
            
            covmat = cov(tempdata);
            deviation = covmat(3,3);%trace(covmat);
            
            [V,K] = eig(covmat);
            [C,I] = min(diag(K));
            
            N = V(:,I);
            N = N .* sign(N(3));
            
            normmap(x,y,:) = N; %N=permute(N,[2 1 3]);
            traces(x,y) = deviation;
            
            %covmat(x,y,:,:) = cov(tempdata);
        end
    end
                  
                    
    D=mxNiDepth(KinectHandles); %D=permute(D,[2 1]);
    mxNiUpdateContext(KinectHandles);
    set(h3, 'CDATA', normmap/2+0.5);
    set(h2,'CDATA',D);
    set(h4,'CDATA',traces);
    drawnow;
end