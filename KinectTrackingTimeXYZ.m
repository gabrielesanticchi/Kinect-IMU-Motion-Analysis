function   [frames_bt,time_m,x,y,z]  = KinectTrackingTimeXYZ(metadata,y_position_ground,joint)
%KinectTrackingTimeXYZ.m The function KinectTrackingTimeXYZ, from the metadata of a 
% Kinect acquisition, returns meaningful information about the x,y,z
% position of a joint 
% [frames_bt,time,x,y,z]  = KinectTrackingTimeXYZ(metadata,height,joint)
% INPUT: 
%       metadata: log of a Kinect acquisition 
%       height: height wrt the ground of the Kinect 
%       joint: joint to be analyzed
% OUTPUT: 
%       frames_bt: frames in which at least one body has been tracked
%       time: array of time intervals (s) wrt to the frames_bt(1) data
%       x,y,z: joint position in (m)

% ''''
% Authors
    % Antonia Lopreside <antonia.lopreside@mail.polimi.it>
    % Gabriele Santicchi <gabriele.santicchi@mail.polimi.it>
    % Davide Matichecchia <davide.matichecchia@mail.polimi.it>
% ''''
y_position_ground = abs(y_position_ground); 
abs_time = zeros(length(metadata),3);
%     idx_bt = zeros(length(metadata),1); %keep track of the body index frame-by-frame 
%     freq_by_samp = 1./diff(abs_time); freq_by_samp = freq_by_samp(:,3);
frames_bt = [];
idx_bt = [];
% retrieve first frame in which at least one 'body' has been tracked by
% the Kinect 
flag_found = 0; 
flag_index = 0; 
    
    for i = 1:length(metadata)
        abs_time(i,:) = metadata(i).AbsTime(1,4:6);

        if flag_found > 0 % if already found, search only there 
            if metadata(i).IsBodyTracked(flag_index) == 1                        
                frames_bt = [frames_bt, i]; 
                idx_bt = [idx_bt, flag_index];
            end
        end

        if flag_found == 0 % search for the body, for the first time 
            for index = 1:6
                if metadata(i).IsBodyTracked(index) == 1                        
                    frames_bt = [frames_bt, i]; 
                    idx_bt = [idx_bt, index];
                    flag_found = 1;
                    flag_index = index;
                end
            end
        end 
    end
    
    acquisition_time = metadata(frames_bt(1)).AbsTime(1,4:6);
    startingframe = frames_bt(1); 
    initial_time = abs_time(1,3); %Time of the first acquired frame 
    time_m = zeros(length(frames_bt),1);
    count = 1; 

    for i = startingframe:(startingframe+length(frames_bt)-1)
        if abs_time(i,2) ~= abs_time(i-1,2) 
            abs_time(i:end,3) = abs_time(i:end,3) + 60;
        end 
        time_m(count,1) = abs_time(i,3) - acquisition_time(1, 3); 
        %In order to have a plot which starts from zero I need to null 
        % the time with respect to the first frame in which joint's positions are acquired
        count = count + 1; 
    end

    %Now I have to define the position of the joints;
    x = zeros(length(frames_bt),1);
    y = zeros(length(frames_bt),1);
    z = zeros(length(frames_bt),1);

%     joint = 1;
    for i = 1:length(frames_bt)  
        x(i,1) = metadata(frames_bt(i)).JointPositions(joint,1,idx_bt(i));
        y(i,1) = metadata(frames_bt(i)).JointPositions(joint,2,idx_bt(i)) + y_position_ground;
        z(i,1) = metadata(frames_bt(i)).JointPositions(joint,3,idx_bt(i));
    end  
end 