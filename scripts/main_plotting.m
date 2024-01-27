clc;
close all; 
load('metadata_skip.mat');  % Load metadata

y_position_ground = 0.90;  % Height of Kinect from the ground
abs_time = zeros(length(metadata),3);
time = processMetadataForTime(metadata, abs_time, y_position_ground);

% Joint index for plotting
j = 1;  % Index of joint to track

% Extract joint positions
[x_position, y_position, z_position] = extractJointPositions(metadata, j, y_position_ground);

% Plot joint positions
plotJointPositions(time, x_position, y_position, z_position);

% Plot skeleton based on joint positions
plotSkeleton(metadata, y_position_ground);

% Animated plot of joint positions
animateJointPositions(time, x_position, y_position, z_position);

% Function Definitions

function time = processMetadataForTime(metadata, abs_time, y_position_ground)
    count = 0;
    for i = 1:length(metadata)
        abs_time(i,:) = metadata(i).AbsTime(1,4:6);
        for index = 1:6
            if metadata(i).IsBodyTracked(index)
                count = count + 1;
                if count == 1  
                    acquisition_time = metadata(i).AbsTime(1,4:6);
                    startingframe = i;
                end
            end
        end
    end

    time = zeros(length(metadata) - (startingframe-1), 1); 
    count = 1;
    for i = startingframe:length(metadata)
        if abs_time(i,2) ~= abs_time(i-1,2)
            abs_time(i:end,3) = abs_time(i:end,3) + 60;
        end
        time(count,1) = abs_time(i,3) - acquisition_time(1, 3);
        count = count + 1;
    end
end

function [x_position, y_position, z_position] = extractJointPositions(metadata, j, y_position_ground)
    x_position = zeros(length(metadata), 1);
    y_position = zeros(length(metadata), 1);
    z_position = zeros(length(metadata), 1);

    for i = 1:length(metadata)
        x_position(i) = metadata(i).JointPositions(j,1,1);
        y_position(i) = metadata(i).JointPositions(j,2,1) + y_position_ground;
        z_position(i) = metadata(i).JointPositions(j,3,1);
    end
end

function plotJointPositions(time, x_position, y_position, z_position)
    figure;
    subplot(3,1,1);
    plot(time, z_position, 'color', 'green', 'LineWidth', 3);
    title('Z Position');
    ylabel('Position on Z (m)');
    grid on;

    subplot(3,1,2);
    plot(time, x_position, 'color', 'blue', 'LineWidth', 3);
    title('X Position');
    grid on;

    subplot(3,1,3);
    plot(time, y_position, 'color', 'red', 'LineWidth', 3);
    title('Y Position');
    xlabel('Seconds (s)');
    grid on;

    linkaxes(findall(gcf,'Type','axes'),'x');
end

function plotSkeleton(metadata, y_position_ground)
    figure;
    for t = 1:length(metadata)
        x_track = metadata(t).JointPositions(:,1,1);
        y_track = metadata(t).JointPositions(:,2,1) + y_position_ground;
        z_track = metadata(t).JointPositions(:,3,1);

        plot3(z_track, x_track, y_track, 'o', 'markersize', 23);
        zlim([0 4]);
        ylim([-1.5 1.5]);
        xlim([0 6]);
        zlabel('Y Position');
        ylabel('X Position');
        xlabel('Z Position');
        grid on;
        pause(0.03);
    end
end

function animateJointPositions(time, x_position, y_position, z_position)
    figure;
    h_x = animatedline('Color','r');
    h_y = animatedline('Color','b');
    h_z = animatedline('Color','g');

    for tt = 1:length(time)
        addpoints(h_x, time(tt), x_position(tt));
        addpoints(h_y, time(tt), y_position(tt));
        addpoints(h_z, time(tt), z_position(tt));
        drawnow;
        pause(0.10);
    end
end
