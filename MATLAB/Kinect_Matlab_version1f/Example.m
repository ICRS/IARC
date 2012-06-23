addpath('Mex')
SAMPLE_XML_PATH='Config/SamplesConfig.xml';

% Start the Kinect Process
filename='Example/SkelShort.oni';
%KinectHandles=mxNiCreateContext(SAMPLE_XML_PATH,filename);

% To use the Kinect hardware use :
KinectHandles=mxNiCreateContext(SAMPLE_XML_PATH);

figure;
I=mxNiPhoto(KinectHandles); I=permute(I,[3 2 1]);
D=mxNiDepth(KinectHandles); D=permute(D,[2 1]);
subplot(1,2,1),h1=imshow(I);
%subplot(1,2,1),h1=mesh(abs(fft(D)));
subplot(1,2,2),h2=imshow(D,[0 6000]); colormap('jet');
    
while(1)
    I=mxNiPhoto(KinectHandles); I=permute(I,[3 2 1]);
    D=mxNiDepth(KinectHandles); D=permute(D,[2 1]);
    mxNiUpdateContext(KinectHandles);
    set(h1,'CDATA',I);
    %absfft = abs(fft2(D));
    %subplot(1,2,1),h1=mesh(absfft(1:20,1:20));
    set(h2,'CDATA',D);
    drawnow; 
end

% Stop the Kinect Process
mxNiDeleteContext(KinectHandles);
