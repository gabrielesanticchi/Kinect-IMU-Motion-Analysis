%% Kinect accuracy evaluation in terms of MAE, MSE, RMSE along the 3 axes. 
clear all 
clc 
close all 

% Create skeleton connection map to link the joints.
SkeletonConnectionMap = buildSkeletonConnectionMap;  % see the function

% insert the filename of the IMU, images and metadata 
load('4_accgyro_skip.mat'); 
% load('1_colorimg_discesagrad.mat'); NON vi consiglio di
% importarle. Provate, ma a me si è quasi bloccato il pc 
load('4_metadata_skip.mat');
%% SYNCHRONIZATION OF SENSOR AND KINECT DATA 

start_kin_time = datetime(  metadata(1).AbsTime(1),  metadata(1).AbsTime(2), metadata(1).AbsTime(3), metadata(1).AbsTime(4), ...
                            metadata(1).AbsTime(5), fix(metadata(1).AbsTime(6)), (metadata(1).AbsTime(6) - fix(metadata(1).AbsTime(6)))*1000, ...
                            'Format', 'yyyy-MM-dd HH:mm:ss.SSS');
end_kin_time   = datetime(  metadata(end).AbsTime(1), metadata(end).AbsTime(2), metadata(end).AbsTime(3), metadata(end).AbsTime(4), ...
                            metadata(end).AbsTime(5), fix(metadata(end).AbsTime(6)), (metadata(end).AbsTime(6) - fix(metadata(end).AbsTime(6)))*1000,...
                            'Format', 'yyyy-MM-dd HH:mm:ss.SSS');
% start_kin_time.Format = 'MMM dd, yyyy HH:mm:ss.SSS'; end_kin_time.Format = 'MMM dd, yyyy HH:mm:ss.SSS'

% take only index where time is between the start and end of kinect logging
idxA = (datenum(Acceleration.Timestamp) > datenum(start_kin_time)) & ...
        (datenum(Acceleration.Timestamp) < datenum(end_kin_time))
idxG = (datenum(AngularVelocity.Timestamp) > datenum(start_kin_time)) & ...
        (datenum(AngularVelocity.Timestamp) < datenum(end_kin_time))
% sum(idx)

%% Preprocessing of Accelerometer and Gyroscope Measurements 
acc = zeros(sum(idxA),4); 
gyro = zeros(sum(idxG),4);
%convert acc and gyro [0,60] seconds in [0,end_test] seconds 
acc(:,1) = second(Acceleration.Timestamp(idxA)); 
for i=2:length(acc)
    if acc(i-1,1) > acc(i,1)
        acc(i:end,1) = acc(i:end,1) + 60; 
    end 
end 
acc(:,1) = acc(:,1) - min(acc(:,1));

gyro(:,1) = second(AngularVelocity.Timestamp(idxG)); 
for i=2:length(gyro)
    if gyro(i-1,1) > gyro(i,1)
        gyro(i:end,1) = gyro(i:end,1) + 60; 
    end 
end 
gyro(:,1) = gyro(:,1) - min(gyro(:,1));

%save the XYZ accceleration (m/s^2) in columns 2,3,4, respectively 
acc(:,2) = Acceleration.X(idxA);  acc(:,3) = Acceleration.Y(idxA); acc(:,4) = Acceleration.Z(idxA); 
gyro(:,2) = AngularVelocity.X(idxG);  gyro(:,3) = AngularVelocity.Y(idxG); gyro(:,4) = AngularVelocity.Z(idxG); 


%% Preprocessing of Metadata information (from Kinect) 
% insert information 
HEIGHT = 0.85;
JOINT = 1; % SpineBase
[frames_bt,time_kin,x,y,z]  = KinectTrackingTimeXYZ(metadata,HEIGHT,JOINT); % see the help


%% First inspection of data
XTICKS = 30;
YTICKS = 10;
MIN_ACC = min([min(acc(:,2)), min(acc(:,3)), min(acc(:,4))]); % to get same scale on XYZ acc. 
MAX_ACC = max([max(acc(:,2)), max(acc(:,3)), max(acc(:,4))]);

fig = figure(1);
ax1 = subplot(321)
plot(time_kin,z,'color','green','LineWidth',3)
xticks(round(linspace(min(time_kin), max(time_kin), XTICKS),2)) % G: togliete se non vi piace
yticks(round(linspace(min(z), max(z), YTICKS),3)) 
grid on 
title('Z-position (Kinect)')

ax2 = subplot(322)
plot(acc(:,1),acc(:,4),'color','green','LineWidth',3)
xticks(round(linspace(min(acc(:,1)), max(acc(:,1)), XTICKS),2)) % G: togliete se non vi piace
% yticks(round(linspace(min(acc(:,4)), max(acc(:,4)), YTICKS),2)) % relative scale 
yticks(round(linspace(MIN_ACC, MAX_ACC, YTICKS),3))
ylim([MIN_ACC-1 MAX_ACC+1])
grid on 
title('Z-acceleration (Sensor, m/s^2)')

ax3 =subplot(323)
plot(time_kin, x,'color','blue','LineWidth',3)
xticks(round(linspace(min(time_kin), max(time_kin), XTICKS),2))
yticks(round(linspace(min(x), max(x), YTICKS),3))
grid on 
title('X-position (Kinect)')

ax4 =subplot(324)
plot(acc(:,1),acc(:,2),'color','blue','LineWidth',3)
xticks(round(linspace(min(acc(:,1)), max(acc(:,1)), XTICKS),2)) % G: togliete se non vi piace
% yticks(round(linspace(min(acc(:,2)), max(acc(:,2)), YTICKS),2)) % relative scale 
yticks(round(linspace(MIN_ACC, MAX_ACC, YTICKS),3))
ylim([MIN_ACC-1 MAX_ACC+1])
grid on 
title('X-acceleration (Sensor, m/s^2)')

ax5 =subplot(325)
plot(time_kin, y,'color','red','LineWidth',3)
xticks(round(linspace(min(time_kin), max(time_kin), XTICKS),2))
yticks(round(linspace(min(y), max(y), YTICKS),3))
grid on 
title('Y-position (Kinect)')
% xlabel('Seconds (s)') 

ax6 =subplot(326)
plot(acc(:,1),acc(:,3),'color','red','LineWidth',3)
xticks(round(linspace(min(acc(:,1)), max(acc(:,1)), XTICKS),2)) % G: togliete se non vi piace
% yticks(round(linspace(min(acc(:,3)), max(acc(:,3)), YTICKS),2)) % relative scale 
yticks(round(linspace(MIN_ACC, MAX_ACC, YTICKS),3))
ylim([MIN_ACC-1 MAX_ACC+1])
grid on 
title('Y-acceleration (Sensor, m/s^2)')
% xlabel('Seconds (s)') 


% Give common xlabel, ylabel and title to your figure
% han=axes(fig,'visible','off'); 
% han.Title.Visible='on';
% han.XLabel.Visible='on';

% xlabel(han,'Seconds (s)');

% linkaxes([ax1 ax2 ax3 ax4 ax5 ax6],'x') non usatelo fino a quando non si
% hanno le stesse 'dimensioni' sulle x
linkaxes([ax1  ax3  ax5 ],'x')
linkaxes([ax2  ax4  ax6 ],'x')

%% Analysis of: TEST_4 (skip) 
acc_orig = acc; % backup 

%% INTERPOLATION OF SENSOR DATA 
acc_interp = acc_orig;
FREQ_IMU_MIN = round(min(1./diff(acc(:,1)))); 
time_eq_acc = linspace(0,max(acc(:,1)),(max(acc(:,1)))*FREQ_IMU_MIN);
acc_interp = [];
acc_interp(:,2) = interp1(acc(:,1),acc(:,2),time_eq_acc,'linear');
acc_interp(:,3) = interp1(acc(:,1),acc(:,3),time_eq_acc,'linear');
acc_interp(:,4) = interp1(acc(:,1),acc(:,4),time_eq_acc,'linear');

figure(1)
plot(acc(:,1),acc(:,3),'color','red','LineWidth',2)
hold on
plot(time_eq_acc,acc_interp(:,3),'color','blue','LineWidth',2)
xticks(round(linspace(min(acc(:,1)), max(acc(:,1)), XTICKS),2)) % G: togliete se non vi piace
yticks(round(linspace(MIN_ACC, MAX_ACC, YTICKS),3))
ylim([MIN_ACC-1 MAX_ACC+1])
hold off
grid on
legend('Original','Interpolated')
title('Interpolation of Y-acceleration (Sensor data, m/s^2)')
%% if ok , replace the data with the one interpolated
acc = [];
acc(:,1) = time_eq_acc;
acc(:,2) = acc_interp(:,2);
acc(:,3) = acc_interp(:,3);
acc(:,4) = acc_interp(:,4);
%% MAX PEAK DETECTION ALGORITHM (SENSOR DATA)
% Discard initial samples, related to the idle phase before the test 

% ************ HYPER PARAMETERS *****************
START_FROM = 0; % second 
END_IN = acc(end,1); % second 
THR_ACC_MAX = 11.0; % detect peak when the acc is over the threshold 
THR_ACC_MIN = 8.0; %(this is more difficult to the multiple presence of min)
FREQ_IMU = ceil(mean(1./diff(acc(:,1))))/3; % sampling frequency of the IMU /3 se 
%ricerca picchi ogni 0.5 sec (non /2, meglio essere più 'stretti' con la
%window'
AXES = 3 % in this case, 'y' -> 3 
% ***********************************************

% acc = acc_orig; 
idx = find((acc(:,1) >= START_FROM) & (acc(:,1) <= END_IN));
temp = [];
for i = 1:size(acc,2)
    temp = [temp, acc(idx,i)]; 
end 
acc_orig = acc; acc = [];  acc = temp; 
acc(:,1) = acc(:,1) - min(acc(:,1)); % start from 0 sec

% search for the max/min each 1 sec <-> each FREQ_IMU samples
% search for the max/min each 0.5 sec <-> each FREQ_IMU/2 samples
max_peak = []; min_peak = []; idx_max_peak = []; idx_min_peak = [];
for i=FREQ_IMU+1:FREQ_IMU:length(acc(:,1))
    [temp,tempidx] = max(acc(i-FREQ_IMU:i,AXES)); % get max and index 
    [temp2,tempidx2] = min(acc(i-FREQ_IMU:i,AXES)); % get min and index
    max_peak = [max_peak, temp];
    idx_max_peak = [idx_max_peak, tempidx+i-FREQ_IMU-1];
    min_peak = [min_peak, temp2];   
    idx_min_peak = [idx_min_peak, tempidx2+i-FREQ_IMU-1];
end 

% select only the ones over the THRESHOLD 
idx_max_peak = idx_max_peak(max_peak > THR_ACC_MAX);
max_peak = max_peak(max_peak > THR_ACC_MAX);
idx_min_peak = idx_min_peak(min_peak < THR_ACC_MIN);
min_peak = min_peak(min_peak < THR_ACC_MIN);

% get rid of the ones which are consecutive (due to windowing on edges) and
% duplicates
rem_idx =[];
for i=2:length(idx_max_peak)
    if((idx_max_peak(i)-FREQ_IMU+1 <= idx_max_peak(i-1)) || (idx_max_peak(i) == idx_max_peak(i-1)))
        rem_idx = [rem_idx, i]
    end
end 
max_peak(rem_idx) = [];
idx_max_peak(rem_idx) = [];

rem_idx =[];
for i=2:length(idx_min_peak)
    if((idx_min_peak(i)-FREQ_IMU+1 <= idx_min_peak(i-1)) || (idx_min_peak(i) == idx_min_peak(i-1)))
        rem_idx = [rem_idx, i]
    end
end 
min_peak(rem_idx) = [];
idx_min_peak(rem_idx) = [];

YTICKS = 30;
figure()
plot(acc(:,1),acc(:,AXES),'color','red','LineWidth',3)
hold on
scatter(acc(idx_max_peak,1),acc(idx_max_peak,AXES),50,'blue','filled')
hold on 
scatter(acc(idx_min_peak,1),acc(idx_min_peak,AXES),50,'green','filled')
xticks([acc(FREQ_IMU+1:FREQ_IMU:length(acc(:,1)),1)])
% xticks(round(linspace(min(acc(:,1)), max(acc(:,1)), XTICKS),2)) % G: togliete se non vi piace
% yticks(round(linspace(min(acc(:,3)), max(acc(:,3)), YTICKS),2)) % relative scale 
yticks(round(linspace(MIN_ACC, MAX_ACC, YTICKS),3))
ylim([MIN_ACC-1 MAX_ACC+1])
xlabel('Window where the min/max has been searched')
ylabel('Acceleration (m/s^2)')
grid on 
title('Y-acceleration (Sensor Data, m/s^2)')

%% Plot the (synchronized) Kinect acquisition and IMU 
YTICKS = 10;
figure()
ax1 = subplot(311)
plot(time_kin, y,'color','red','LineWidth',3)
% xticks(round(linspace(min(time_kin), max(time_kin), XTICKS),2))
% xticks([acc(FREQ_IMU+1:FREQ_IMU:length(acc(:,1)),1)]) % windowing 
xticks([acc((idx_max_peak),1)]) % peak of the sensor 
yticks(round(linspace(min(y), max(y), YTICKS),3))
ylabel('meters')
grid on 
title('Y-position (Kinect)')
% xlabel('Seconds (s)') 

ax2 = subplot(312)
plot(time_kin(2:end), diff(y),'color','red','LineWidth',3)
% xticks(round(linspace(min(time_kin), max(time_kin), XTICKS),2))
% xticks([acc(FREQ_IMU+1:FREQ_IMU:length(acc(:,1)),1)]) % windowing 
xticks([acc((idx_max_peak),1)]) % peak of the sensor 
yticks(round(linspace(min(diff(y)), max(diff(y)), YTICKS),3))
% ylim([MIN_ACC-1 MAX_ACC+1])
grid on 
title('derivative of Y-position (Kinect)')
linkaxes([ax1 ax2 ],'x')

subplot(313)
plot(acc(:,1),acc(:,AXES),'color','red','LineWidth',3)
hold on
scatter(acc(idx_max_peak,1),acc(idx_max_peak,AXES),50,'blue','filled')
hold on 
scatter(acc(idx_min_peak,1),acc(idx_min_peak,AXES),50,'green','filled')
% xticks(round(linspace(min(acc(:,1)), max(acc(:,1)), XTICKS),2))   
% xticks([acc(FREQ_IMU+1:FREQ_IMU:length(acc(:,1)),1)]) % windowing 
xticks([acc((idx_max_peak),1)]) % peak of the sensor 
yticks(round(linspace(MIN_ACC, MAX_ACC, YTICKS),3))
ylim([MIN_ACC-1 MAX_ACC+1])
grid on 
title('Y-acceleration (Sensor Data)')
ylabel(' m/s^2')
xlabel('time (s)')
% xlabel('Seconds (s)') 

%% RECOGNITION OF SKIP STEPS (KINECT DATA)
% This algorithm compute the derivative of the y-position and search for 
% regions where the der is below the threshold. Then, a seach of the first 
% minimum is performed -> that's the point in which the step has occurred. 

% ************ HYPER PARAMETERS *****************
DERIVATIVE_THR = -0.02; % detect step when the derivative is below theshold 
FREQ_KIN_ORIG = round(mean(1./diff(time_kin)));
FREQ_KIN = FREQ_KIN_ORIG/3; % sampling frequency of the KINECT
%mettere /2 se ricerca picchi ogni 0.5 sec
AXES_KIN = y; 
NUMBER_OF_STEPS = 30; %number of peak to be recognized
% ***********************************************
der_AXES_KIN = diff(AXES_KIN);
der_time = time_kin(2:end);
idx = find((der_AXES_KIN < DERIVATIVE_THR));

% search for the minimum each 1 sec <-> each FREQ_KIN samples
min_der = []; idx_der_min = []; step_recognized = 0;
for i = FREQ_KIN+1:FREQ_KIN:length(der_AXES_KIN(:,1))
    if step_recognized < NUMBER_OF_STEPS
    [temp,tempidx] = min(der_AXES_KIN(i-FREQ_KIN:i)); % get min and index 
%     step_recognized = step_recognized+1;
    min_der = [min_der, temp];
    idx_der_min = [idx_der_min, tempidx+i-FREQ_KIN-1];
    end
end 

% select only the ones over the THRESHOLD 
idx_der_min = idx_der_min(min_der < DERIVATIVE_THR);
min_der = min_der(min_der < DERIVATIVE_THR);

% get rid of the ones which are consecutive (due to windowing on edges) and
% duplicates
rem_idx =[];
for i=2:length(idx_der_min)
    if((idx_der_min(i)-FREQ_KIN+1 <= idx_der_min(i-1)) || (idx_der_min(i) == idx_der_min(i-1)))
        rem_idx = [rem_idx, i]
    end
end 
min_der(rem_idx) = [];
idx_der_min(rem_idx) = [];

% YTICKS = 100
figure()
plot(der_time,der_AXES_KIN,'color','green','LineWidth',3)
hold on
scatter(der_time(idx_der_min),der_AXES_KIN(idx_der_min),50,'blue','filled')
hold on
plot(der_time,zeros(length(der_time),1),'color','red','LineWidth',2)
xticks(round(linspace(min(der_time), max(der_time), XTICKS),2))
xticks([der_time(FREQ_KIN+1:FREQ_KIN:length(der_time))])
% yticks(round(linspace(min(der_AXES_KIN), max(der_AXES_KIN), YTICKS),4))
yticks(linspace(min(der_AXES_KIN), max(der_AXES_KIN), 25))
grid on 
title('derivative of Y-position (Kinect)')
xlabel('time (s)')

%%
% now, find the first 2 null derivative between idx_step(i) and idx_step(i+1)
idx_der_null = []; temp = [];
for i = 1:length(idx_der_min)-1 % for each window
    if (i < length(idx_der_min))
        for j = idx_der_min(i):idx_der_min(i+1)    % for each sample in the window 
            if ((der_AXES_KIN(j-1) < 0) && ( der_AXES_KIN(j) > 0))  %if the zero has ben crossed
                temp = [temp, j];                                 % append to temp the index 
            end
            if ((der_AXES_KIN(j-1) > 0) && ( der_AXES_KIN(j) < 0))  %if the zero has ben crossed
                temp = [temp, j];                                 % append to temp the index 
            end
        end
    end 
    
    if (i == length(idx_der_min)-1)
        for j = idx_der_min(i):der_time(end)    % for each sample in the window
            if ((der_AXES_KIN(j-1) < 0) && ( der_AXES_KIN(j) > 0))  %if the zero has ben crossed
                temp = [temp, j];                                 % append to temp the index 
            end
        end
    end 
    
    if size((temp),1) > 0
        idx_der_null = [idx_der_null, temp(1:2)]; % append only first null derivative 
    end 
    temp = [];
end

acc_time_orig = acc(idx_max_peak,1); % backup 
%% Synchronization of Kinect and Sensor Data 
% Compute the average distance between time(idx_peak) and
% der_time(idx_der_null), which represents the same instant; 
% then subtract one to the others in other to get synchronized data
% acc(idx_max_peak,1) = acc_time_orig; 
% shift_in_time = acc(idx_max_peak,1) - der_time(idx_der_null(1:8)); % da mettere a posto 
% acc(:,1) =  acc(:,1) - mean(shift_in_time);

%% Plot synchronized data
figure()

ax1 = subplot(311)
plot(time_kin, AXES_KIN,'color','red','LineWidth',3)
hold on
scatter(time_kin(idx_der_min),AXES_KIN(idx_der_min),50,'magenta','filled')
hold on
scatter(time_kin(idx_der_null),AXES_KIN(idx_der_null),50,'green','filled')
% xticks(der_time(idx_der_null))
xticks(der_time(idx_der_null))
yticks(round(linspace(min(AXES_KIN), max(AXES_KIN), YTICKS),3))
grid on 
title('Y-position (Kinect)')
ylabel('meters')
legend('y position', 'min derivative','derivative null (skip down and up)')
% xlabel('Seconds (s)') 

ax2 = subplot(312)
plot(der_time,der_AXES_KIN,'color','red','LineWidth',3)
hold on
scatter(der_time(idx_der_min),der_AXES_KIN(idx_der_min),50,'magenta','filled')
hold on
scatter(der_time(idx_der_null),der_AXES_KIN(idx_der_null),50,'green','filled')
hold on
plot(der_time,zeros(length(der_time),1),'color','blue','LineWidth',1)
xticks(der_time(idx_der_null))
yticks(linspace(min(der_AXES_KIN), max(der_AXES_KIN), 10))
xlim([0 max(der_time)])
grid on 
title('derivate of Y-position (Kinect)')
legend('Derivative','Derivative minimum','Derivative null (skip down)','line = 0')

linkaxes([ax1 ax2],'x')

subplot(313)
plot(acc(:,1),acc(:,AXES),'color','red','LineWidth',3)
hold on
scatter(acc(idx_max_peak,1),acc(idx_max_peak,AXES),50,'blue','filled')
hold on
scatter(acc(idx_min_peak,1),acc(idx_min_peak,AXES),50,'green','filled')
% xticks(round(linspace(min(acc(:,1)), max(acc(:,1)), XTICKS),2))  
xticks([acc(idx_min_peak,1)]) % skip up
% xticks([acc(idx_max_peak,1)]) % skip down
% xticks(acc(idx_peak,1))
% yticks(round(linspace(min(acc(:,3)), max(acc(:,3)), YTICKS),2)) % relative scale 
yticks(round(linspace(MIN_ACC, MAX_ACC, YTICKS),3)) 
ylim([MIN_ACC-1 MAX_ACC+1])
xlim([0 max(der_time)])
grid on 
title('Y-acceleration (m/s^2) - Sensor data')
% legend('Acceleration values','Peak (skip down)','Peak (skip up)')
legend('Acceleration values','Peak (skip up)','Peak (skip down)')
ylabel(' m/s^2')
xlabel('time (s)')
%% Find the amplitude of the displacement from the acceleration and Kinect
temp = (min(length(max_peak), length(min_peak)));
a_acc = []; a_disp = [];
a_acc = (max_peak(1:temp) - min_peak(1:temp))/2; 
f = 2; % two steps each second
a_disp = mean(a_acc./(2*pi*f)^2);
max_peak_pos = max_peak./((2*pi*f)^2);
min_peak_pos = min_peak./((2*pi*f)^2);

% find the amplitude of displacement in the kinect 
% by computing the difference between even indexes (high peaks) and odd indexes(low peaks)
min_kin =  AXES_KIN(idx_der_null(1:2:length(idx_der_null)));
max_kin =  AXES_KIN(idx_der_null(2:2:length(idx_der_null)));
kin_disp = abs( max_kin - min_kin);
kin_disp = mean(kin_disp)/2;

kin_baseline = mean(min_kin) + kin_disp;
% kin_baseline = mean(max_kin) - kin_disp

% Compute MAE, MSE, RMSE between peaks 
temp =  (min(length(min_kin), length(min_peak)));

MAE_kin_acc = mean((abs(min_kin(1:temp)' - min_peak_pos(1:temp))))
MSE_kin_acc = mean((abs(min_kin(1:temp)' - min_peak_pos(1:temp))).^2)
RMSE_kin_acc = sqrt(MSE_kin_acc)
%% ANALYSIS OF KINECT DATA - 4 
idx = min(idx_der_null):max(idx_der_null);
time_ground_thruth = time_kin(idx); 
amplitude_ground_thruth = a_disp; %take the accelerometer values as gt 
% build the sine function only between peaks which have been found in the kinect data 
Y_GROUND_TRUTH = amplitude_ground_thruth*sin(2*pi*f*time_ground_thruth) + kin_baseline;
Z_GROUND_TRUTH = zeros(length(time_ground_thruth),1) + 3.00; % distance from the kinect
X_GROUND_TRUTH = zeros(length(time_ground_thruth),1); 

ax1 = subplot(311)
plot(time_ground_thruth, Y_GROUND_TRUTH,'LineWidth',2)
hold all 
plot(time_kin(idx), AXES_KIN(idx),'LineWidth',2,'Color','red')
ylim([mean(min_kin)-0.25 mean(min_kin)+0.25])
yticks(linspace(-1,+1,51)) %101a
xticks(time_kin(idx_der_null(1:end)))
grid on
legend('Y GROUND THRUTH(accel data)','Kinect data')
title([ 'MAE: ' ,num2str(MAE_kin_acc), ', MSE: ',num2str(MSE_kin_acc), ', RMSE: ', num2str(RMSE_kin_acc) ])
ylabel('Meters (m)')

ax2 = subplot(312)
plot(time_ground_thruth, Z_GROUND_TRUTH,'LineWidth',2)
hold on
plot(time_kin(idx), z(idx),'LineWidth',2,'Color','red')
hold on
% scatter(time_kin(idx_der_null(1:end)), z(idx_der_null),50,'red','filled')
ylim([min(mean(Z_GROUND_TRUTH),mean(z(idx)))-0.25 max(mean(Z_GROUND_TRUTH),mean(z(idx)))+0.25])
% yticks(linspace(round(mean(z(idx))-0.25,3),round(mean(z(idx))-0.25,3),11)) %101a
% xticks(round(time_kin(idx)),2)
xticks(time_kin(idx_der_null(1:end)))
grid on
legend('Z GROUND THRUTH(const distance)','Kinect data')
ylabel('Meters (m)')

ax3 = subplot(313)
plot(time_ground_thruth, X_GROUND_TRUTH,'LineWidth',2)
hold all 
plot(time_kin(idx), x(idx),'LineWidth',2,'Color','red')
ylim([min(mean(X_GROUND_TRUTH),mean(x(idx)))-0.25 max(mean(X_GROUND_TRUTH),mean(x(idx)))+0.25])
% yticks(linspace(round(mean(z(idx))-0.25,3),round(mean(z(idx))-0.25,3),11)) %101a
% xticks(round(time_kin(idx)),2)
xticks(time_kin(idx_der_null(1:end)))
grid on
legend('X GROUND THRUTH(const distance)','Kinect data')
ylabel('Meters (m)')
xlabel('Seconds (s)')
linkaxes([ax1  ax2  ax3 ],'x')


%%

%% DA FARE: 
% 0. sincronizzare acc e position_kinect. C'è disponibile anche gyro se
% serve, ma non so -> OK 

% 1. Omino che si muove, 
% 2. Se omino funziona, mettere grafico xyz nella stessa figure con la posizione istante per istante
% 3. Calcolo MAE, MSE, RMSE rispetto al ground truth (come lo calcoliamo? a
% mano? 



