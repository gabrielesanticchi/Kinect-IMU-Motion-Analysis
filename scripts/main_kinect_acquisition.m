% Create color and depth kinect videoinput objects.
% https://it.mathworks.com/help/supportpkg/kinectforwindowsruntime/ug/acquire-image-and-body-data-using-kinect-v2.html 


% Clear environment and reset image acquisition hardware connection
clear all;
imaqreset;
imaqmex('feature','-limitPhysicalMemoryUsage',false);

% Create color and depth videoinput objects for Kinect
colorVid = videoinput('kinect', 1);
depthVid = videoinput('kinect', 2);

% Configure depth sensor for body tracking
depthSource = getselectedsource(depthVid);
depthSource.EnableBodyTracking = 'on';

% Set frames per trigger for both color and depth
framesPerTrig = 360;
colorVid.FramesPerTrigger = framesPerTrig;
depthVid.FramesPerTrigger = framesPerTrig;

% Start the video input objects
start([depthVid, colorVid]);
pause(25); % Adjust pause duration if needed

% Retrieve the captured data
% [colorImg] = getdata(colorVid);
[~, ~, metadata] = getdata(depthVid);


% Check Kinect Documentation for joints enumeration.
% https://learn.microsoft.com/en-us/previous-versions/windows/kinect/dn758662%28v%3dieb.10%29
SkeletonConnectionMap = buildSkeletonConnectionMap();

% (Optional) Create GIF from Kinect frames
% Uncomment the below section to enable GIF creation
% filename = 'kinectCapture.gif';
createKinectGif(filename, colorImg, metadata, SkeletonConnectionMap);

% Add additional data processing or saving functions if needed