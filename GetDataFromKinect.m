% Create color and depth kinect videoinput objects.
% https://it.mathworks.com/help/supportpkg/kinectforwindowsruntime/ug/acquire-image-and-body-data-using-kinect-v2.html 


% If you run the project the 2nd time, use these commands previously: 
clear all
imaqreset
imaqmex('feature','-limitPhysicalMemoryUsage',false);


colorVid = videoinput('kinect', 1)
depthVid = videoinput('kinect', 2)

% Look at the device-specific properties on the depth source device,
% which is the depth sensor on the Kinect V2.
% Set 'EnableBodyTracking' to on, so that the depth sensor will
% return body tracking metadata along with the depth frame.
depthSource = getselectedsource(depthVid);
depthSource.EnableBodyTracking = 'on';

% Acquire 100 color and depth frames.
framesPerTrig = 360;
colorVid.FramesPerTrigger = framesPerTrig;
depthVid.FramesPerTrigger = framesPerTrig;

% Start the depth and color acquisition objects.
% This begins acquisition, but does not start logging of acquired data.
% pause(5);
start([depthVid colorVid]);

pause(25);
% Get images and metadata from the color and depth device objects.
[colorImg] = getdata(colorVid);

% while get(vid,'FramesAvailable')<1  %Wait until at least 1 frame is available
[~, ~, metadata] = getdata(depthVid);

% These are the order of joints returned by the kinect adaptor.
%    SpineBase = 1;
%    SpineMid = 2;
%    Neck = 3;
%    Head = 4;
%    ShoulderLeft = 5;
%    ElbowLeft = 6;
%    WristLeft = 7;
%    HandLeft = 8;
%    ShoulderRight = 9;
%    ElbowRight = 10;
%    WristRight = 11;
%    HandRight = 12;
%    HipLeft = 13;
%    KneeLeft = 14;
%    AnkleLeft = 15;
%    FootLeft = 16;
%    HipRight = 17;
%    KneeRight = 18;
%    AnkleRight = 19;
%    FootRight = 20;
%    SpineShoulder = 21;
%    HandTipLeft = 22;
%    ThumbLeft = 23;
%    HandTipRight = 24;
%    ThumbRight = 25;


% Create skeleton connection map to link the joints.
SkeletonConnectionMap = [ [4 3];  % Neck
                          [3 21]; % Head
                          [21 2]; % Right Leg
                          [2 1];
                          [21 9];
                          [9 10];  % Hip
                          [10 11];
                          [11 12]; % Left Leg
                          [12 24];
                          [12 25];
                          [21 5];  % Spine
                          [5 6];
                          [6 7];   % Left Hand
                          [7 8];
                          [8 22];
                          [8 23];
                          [1 17];
                          [17 18];
                          [18 19];  % Right Hand
                          [19 20];
                          [1 13];
                          [13 14];
                          [14 15];
                          [15 16];
                        ];
                    

%%
% filename = 'testAnimated.gif'
% 
% tStart = tic;
% j = 0;
% T_skeleton = zeros(1,length(metadata)); 
% T_image = zeros(1,length(metadata)); 
% for Frame_i = (length(metadata)/10):(5):(length(metadata))
%     j = j+1;
%     tic
% %   Frame_i = framesPerTrig-10;
%     lastframeMetadata = metadata(Frame_i);
%     
%     T_image(j) = toc; 
%     % Find the indexes of the tracked bodies.
%     anyBodiesTracked = any(lastframeMetadata.IsBodyTracked ~= 0);
%     trackedBodies = find(lastframeMetadata.IsBodyTracked);
% 
%     % Find number of Skeletons tracked.
%     nBodies = length(trackedBodies);
% 
%     % Get the joint indices of the tracked bodies with respect to the color
%     % image.
%     colorJointIndices = lastframeMetadata.ColorJointIndices(:, :, trackedBodies);
% 
%     % Extract the 90th color frame.
%     lastColorImage = colorImg(:, :, :, Frame_i);
% 
%     % Marker colors for up to 6 bodies.
%     colors = ['r';'g';'b';'c';'y';'m'];
% 
%     % Display the RGB image.
%     h = figure()
%     imshow(lastColorImage);
%     frame = getframe(h); 
%     
%     im = frame2im(frame); 
%     [imind,cm] = rgb2ind(im,256); 
%     writeAnimation(h,'test.gif','LoopCount',1)
%     
%     % Overlay the skeleton on this RGB frame.
%     for i = 1:24
%          for body = 1:nBodies
%              X1 = [colorJointIndices(SkeletonConnectionMap(i,1),1,body) colorJointIndices(SkeletonConnectionMap(i,2),1,body)];
%              Y1 = [colorJointIndices(SkeletonConnectionMap(i,1),2,body) colorJointIndices(SkeletonConnectionMap(i,2),2,body)];
%              line(X1,Y1, 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', '+', 'Color', colors(body));
%          end
% 
%         hold on;
%     end
% %     Write to the GIF File 
%     if ((Frame_i) == length(metadata)/10) 
%           imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%     else 
%           imwrite(imind,cm,filename,'gif','WriteMode','append'); 
%     end 
% 
%      hold off;
%      T_skeleton(j) =  toc;  
% %     print(i) 
% end 
% tSUM_skeleton = sum(T_skeleton) 
% tSUM_image = sum(T_image) 
% tEND = toc(tStart)
% % Elapsed time is 12.124560 seconds (with 100 Frames) 
% % Elapsed time is 18.399501 seconds (with 200 Frames) 
%     % It depends on inserting the plot of the image, the presence of a
%     % skeleton etc. 
% % imagesc(frame);
% % fName = ['test_image_',num2str(i),'.png'];
% % saveas(gca,fName);