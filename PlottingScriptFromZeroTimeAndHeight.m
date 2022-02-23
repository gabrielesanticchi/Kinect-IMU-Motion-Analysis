% Frame49_1 = frame49(1,:,1)
%frame49 = metadata(49).JointPositions
%Time vector

% j is the joint's index 
%k is the number of bodies tracked 
%in y_position_ground you need to put the height of the kinect

clc;
close all; 
load('4_metadata_skip.mat');
abs_time = zeros(length(metadata),3);


count = 0; 

for i = 1:length(metadata)
close all; 

y_position_ground = 0.90;
    abs_time(i,:) = metadata(i).AbsTime(1,4:6);
    for index = 1:6
        if metadata(i).IsBodyTracked(index) == 1
            count = count + 1; 
            k = index; 
        end 
        if count == 1  
            acquisition_time = metadata(i).AbsTime(1,4:6);
            startingframe = i; 
        end 
    end
end


initial_time = abs_time(1,3); %Time of the first acquired frame 
time = zeros((length(metadata) - (startingframe-1)),1); 

count = 1; 

for i = startingframe:length(metadata)
    

    if abs_time(i,2) ~= abs_time(i-1,2) 
        abs_time(i:end,3) = abs_time(i:end,3) + 60;
    end 

    time(count,1) = abs_time(i,3) - acquisition_time(1, 3); 
    %In order to have a plot which starts from zero I need to null 
    % the time with respect to the first frame in which joint's positions are acquired
    count = count + 1; 

end

%Now I have to define the position of the joints;
x_position = zeros((length(metadata) - (startingframe-1)),1);
y_position = zeros((length(metadata) - (startingframe-1)),1);
z_position = zeros((length(metadata) - (startingframe-1)),1);

count = 1; 
j = 1;

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
y_position_ground = abs(y_position_ground);

for i = startingframe:length(metadata)  
    x_position(count,1) = metadata(i).JointPositions(j,1,k);
    y_position(count,1) = metadata(i).JointPositions(j,2,k) + y_position_ground;
    z_position(count,1) = metadata(i).JointPositions(j,3,k);
    count = count + 1;
end  

freq_by_samp = 1./diff(time);
figure(4)
plot(time(1:end-1),freq_by_samp);

figure(1);
ax1 = subplot(311)
hold on;
plot(time,z_position,'color','green','LineWidth',3)
hold off;
title('z position')
ylabel('Position on z (m)')
grid on; 
% xlabel('Seconds (s)') 
ax2 =subplot(312)
hold on; 
plot(time, x_position,'color','blue','LineWidth',3)
hold off;
title('x position');
grid on; 
% xlabel('Seconds (s)') 
ax3 =subplot(313)
hold on;
plot(time, y_position,'color','red','LineWidth',3)
hold off;
title('y position')
xlabel('Seconds (s)') 
grid on; 

max(x_position)
min(x_position)
z_position(1)


linkaxes([ax1 ax2 ax3],'x')
framesPerTrig = 360; 
%% Trying to plot the skeleton based on the joint position and not on the colorcoordinatesystem
clc;
figure(4);
%for cycle on time
x_track = [];
y_track = [];
z_track = [];

%left_arm = [5,6,7,8];
%right_arm = [9,10,11,12];
%left_leg = [13,14,15,16];
%right_leg = [17,18,19,20];
%chest = [1,2,3,4];

%subplot(222)
% h = animatedline;
% x = linspace(startingframe,length(metadata),length(metadata)-startingframe+1);

for t = startingframe:framesPerTrig %Here you have to insert the number of frame taked into account!! 

    %I achieve the joint position for each frame considered. The index
    %which represents the time is t. 

    %i is the index related to the joint's position.

    for i = 1:25
        x_track(i) = metadata(t).JointPositions(i,1,k);
        

        y_track(i) = metadata(t).JointPositions(i,2,k) + y_position_ground;
    

        z_track(i) = metadata(t).JointPositions(i,3,k);
    end

    %Left Arm
    z1 = z_track(5:8);
    x1 = x_track(5:8);
    y1 = y_track(5:8);
    %Right Arm
    z2 = z_track(9:12);
    x2 = x_track(9:12);
    y2 = y_track(9:12);
    %Left leg
    z3 = z_track(13:16);
    x3 = x_track(13:16);
    y3 = y_track(13:16);
    %Right leg
    z4 = z_track(17:20);
    x4 = x_track(17:20);
    y4 = y_track(17:20);
    %Chest
    x5 = x_track(1:3);
    y5 = y_track(1:3);
    z5 = z_track(1:3);
    %Head 
    x12 = x_track(4);
    y12 = y_track(4);
    z12 = z_track(4);
    %Ankle
    x6 = x_track([17,1,13]);
    y6 = y_track([17,1,13]);
    z6 = z_track([17,1,13]);
    %Shoulder
    x7 = x_track([9,21,5]);
    y7 = y_track([9,21,5]);
    z7 = z_track([9,21,5]);
    %Left Hand 
    x8 = x_track([22,8]);
    y8 = y_track([22,8]);
    z8 = z_track([22,8]);
    x9 = x_track([23,8]);
    y9 = y_track([23,8]);
    z9 = z_track([23,8]);
    %RIght Hand
    x10 = x_track([24,12]);
    y10 = y_track([24,12]);
    z10 = z_track([24,12]);
    x11 = x_track([25,12]);
    y11 = y_track([25,12]);
    z11 = z_track([25,12]);


    %Plot the left and right arm
%     subplot(211)
    plot3(z1,x1,y1,z2,x2,y2,z3,x3,y3,z4,x4,y4,z5,x5,y5,z12,x12,y12,'o',z6,x6,y6,z7,x7,y7,z8,x8,y8,z9,x9,y9,z10,x10,y10,z11,x11,y11,'markersize',23);
    %Attento al sistema di riferimento di matlab, come al solito è sballato
    
    zlim([0 4]);
    zlabel('Y position')
    ylim([-1.5 1.5]);
    ylabel('X position')
    xlim([0 6]);
    xlabel('Z Position')
    
%     drawnow;
%     grid on;
%     pause(0.03)

%     subplot(212)
%     tt = t - (startingframe) +1; 
%     zlim([0 2]);
%     plot(x,x_position);
%     hold on 
%     plot(x,y_position);
%     hold on 
%     plot(x,z_position);
%     legend('x pos','y pos','z pos');
%     
%     addpoints(h,x(tt),z_position(tt));
%     drawnow 
    
%     if isvalid(h)
%         addpoints(h,x(t),z_position(t));
%     else
%         break % stop trying to add points
%     end
     
    
    drawnow;
    grid on;
    pause(0.03) 
    

    %in order to obtain a plot which changes in 10 s i consider a pause equal to 0.033.
    %Due to the fact that kinect acquired 30 frame/s, 1 frame/30frame/s =
    %0.033s. In 0.033s is obtained 1 frame. 

   
end

%% 
x = linspace(startingframe,length(metadata),length(metadata)-startingframe+1);

subplot(311)
h_x = animatedline('Color','r');;
ylim([min(x_position) max(x_position)]);
xlim([0 length(x)]);
grid on
legend('x');

subplot(312)
h_y = animatedline('Color','b');
ylim([min(y_position) max(y_position)]);
xlim([0 length(x)]);
grid on
legend('y');

subplot(313)
h_z = animatedline('Color','g');;
ylim([min(z_position) max(z_position)]);
xlim([0 length(x)]);
grid on
legend('z');

for tt = 1:(framesPerTrig-startingframe+1)   

    addpoints(h_x,x(tt),x_position(tt));
    addpoints(h_y,x(tt),y_position(tt));
    addpoints(h_z,x(tt),z_position(tt));
    drawnow 
    pause(0.10)
end 

