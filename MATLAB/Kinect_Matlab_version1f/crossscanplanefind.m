addpath('Mex')
SAMPLE_XML_PATH='Config/SamplesConfig.xml';

% Start the Kinect Process
filename='Example/SkelShort.oni';
%KinectHandles=mxNiCreateContext(SAMPLE_XML_PATH,filename);

% To use the Kinect hardware use :
KinectHandles=mxNiCreateContext(SAMPLE_XML_PATH);

Sobelx = [-1 0 1; -2 0 2; -1 0 1];
Sobely = Sobelx';

Fx = conv2(D, Sobelx);
Fy = conv2(D, Sobely);

H = fspecial('gaussian', 10, 2);

D=mxNiDepth(KinectHandles); D=permute(D,[2 1]);
subplot(1,3,1); h2=imshow(D,[0 6000]); colormap('jet');
subplot(1,3,2); h3=imshow(Fx,[-20 20]); colormap('jet');
subplot(1,3,3); h4=imshow(Fy,[-20 20]); colormap('jet');

while(1)
    XYZ=mxNiDepthRealWorld(KinectHandles);
    D=mxNiDepth(KinectHandles); D=permute(D,[2 1]);
    
    D(D==0)=nan;
    %D = conv2(D, H);
    
    Fx = conv2(D, Sobelx);
    Fy = conv2(D, Sobely);
    
    mxNiUpdateContext(KinectHandles);
    set(h2,'CDATA',D);
    set(h3,'CDATA',Fx);
    set(h4,'CDATA',Fy);
    drawnow;
end