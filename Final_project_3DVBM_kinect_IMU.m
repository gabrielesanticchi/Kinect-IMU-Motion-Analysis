%% Kinect accuracy evaluation in terms of amplitude and frequency, with respec to a ground truth (Accelerometer data). 

% Authors
    % Gabriele Santicchi <gabriele.santicchi@mail.polimi.it>
    % Antonia Lopreside <antonia.lopreside@mail.polimi.it>
    % Davide Matichecchia <davide.matichecchia@mail.polimi.it>
% ''''

clear all 
clc 
close all 

% Create skeleton connection map to link the joints.
SkeletonConnectionMap = buildSkeletonConnectionMap;  % see the function

% insert the filename of the IMU, images and metadata 
load('4_accgyro_skip.mat'); 
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
        (datenum(Acceleration.Timestamp) < datenum(end_kin_time));
idxG = (datenum(AngularVelocity.Timestamp) > datenum(start_kin_time)) & ...
        (datenum(AngularVelocity.Timestamp) < datenum(end_kin_time));
% sum(idxA)
% sum(idxG)
%% Preprocessing of Accelerometer and Gyroscope Measurements 

% ************ HYPERPARAMETERS ****************
SAMPLES_MARGIN = 150;    % as the clock timestamp could be a little different, 
                        % consider further samples and then compute cross-correlation to
                        % synchronized the Kinect data with the Sensor one. 
                        % put it to '0' if synchronization is assured. 
% *********************************************

[~,ia,~] = unique(idxA,'first');
idxA((ia(2)-SAMPLES_MARGIN):ia(2)) = (idxA((ia(2)-SAMPLES_MARGIN):ia(2)) | 1);
% [A,ia,~] = unique(idxA,'first') % test 

[~,ia,~] = unique(idxA,'last');
idxA((ia(2)):(ia(2)+SAMPLES_MARGIN)) = (idxA((ia(2)): (ia(2)+SAMPLES_MARGIN)) | 1);
% [A,ia,~] = unique(idxA,'last') % test 

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
set(gcf,'color','w'); %set white background 
ax1 = subplot(321);
plot(time_kin,z,'color','green','LineWidth',3)
xticks(round(linspace(min(time_kin), max(time_kin), XTICKS),2))
yticks(round(linspace(min(z), max(z), YTICKS),3)) 
grid on 
title('Z-position (Kinect)')

ax2 = subplot(322);
plot(acc(:,1),acc(:,4),'color','green','LineWidth',3)
xticks(round(linspace(min(acc(:,1)), max(acc(:,1)), XTICKS),2)) 
% yticks(round(linspace(min(acc(:,4)), max(acc(:,4)), YTICKS),2)) % relative scale 
yticks(round(linspace(MIN_ACC, MAX_ACC, YTICKS),3))
ylim([MIN_ACC-1 MAX_ACC+1])
grid on 
title('Z-acceleration (Sensor, m/s^2)')

ax3 =subplot(323);
plot(time_kin, x,'color','blue','LineWidth',3)
xticks(round(linspace(min(time_kin), max(time_kin), XTICKS),2))
yticks(round(linspace(min(x), max(x), YTICKS),3))
grid on 
title('X-position (Kinect)')

ax4 =subplot(324);
plot(acc(:,1),acc(:,2),'color','blue','LineWidth',3)
xticks(round(linspace(min(acc(:,1)), max(acc(:,1)), XTICKS),2)) 
% yticks(round(linspace(min(acc(:,2)), max(acc(:,2)), YTICKS),2)) % relative scale 
yticks(round(linspace(MIN_ACC, MAX_ACC, YTICKS),3))
ylim([MIN_ACC-1 MAX_ACC+1])
grid on 
title('X-acceleration (Sensor, m/s^2)')

ax5 =subplot(325);
plot(time_kin, y,'color','red','LineWidth',3)
xticks(round(linspace(min(time_kin), max(time_kin), XTICKS),2))
yticks(round(linspace(min(y), max(y), YTICKS),3))
grid on 
title('Y-position (Kinect)')
% xlabel('Seconds (s)') 

ax6 =subplot(326);
plot(acc(:,1),acc(:,3),'color','red','LineWidth',3)
xticks(round(linspace(min(acc(:,1)), max(acc(:,1)), XTICKS),2)) 
% yticks(round(linspace(min(acc(:,3)), max(acc(:,3)), YTICKS),2)) % relative scale 
yticks(round(linspace(MIN_ACC, MAX_ACC, YTICKS),3))
ylim([MIN_ACC-1 MAX_ACC+1])
grid on 
title('Y-acceleration (Sensor, m/s^2)')
% xlabel('Seconds (s)') 

linkaxes([ax1  ax3  ax5 ],'x')
linkaxes([ax2  ax4  ax6 ],'x')

%% Analysis of: TEST_4 (skip) 
acc_orig = acc; % backup 
x_orig = x;
y_orig = y;
z_orig = z;
%% INTERPOLATION OF KINECT AND SENSOR DATA 

acc_interp = acc_orig; 
x = x_orig; y = y_orig; z = z_orig;
FREQ_KIN_MIN = round(min(1./diff(time_kin)));
FREQ_IMU_MIN = round(min(1./diff(acc(:,1))));
FREQ_KIN_MAX = round(max(1./diff(time_kin)));
FREQ_IMU_MAX = round(max(1./diff(acc(:,1))));

% ***********************************************************
% Interpolate data with the max between the two sampling frequency, 
% in order to avoid loss of information     
FREQ_INTERP = FREQ_IMU_MAX;
% ***********************************************************

time_eq_acc = linspace(0,max(acc(:,1)),(max(acc(:,1)))*FREQ_INTERP);
time_eq_kin = linspace(0,max(time_kin),(max(time_kin))*FREQ_INTERP);
acc_interp = [];
acc_interp(:,2) = interp1(acc(:,1),acc(:,2),time_eq_acc,'linear');
acc_interp(:,3) = interp1(acc(:,1),acc(:,3),time_eq_acc,'linear');
acc_interp(:,4) = interp1(acc(:,1),acc(:,4),time_eq_acc,'linear');
x_interp = interp1(time_kin,x,time_eq_kin,'linear');
y_interp = interp1(time_kin,y,time_eq_kin,'linear');
z_interp = interp1(time_kin,z,time_eq_kin,'linear');

figure(1)
set(gcf,'color','w'); %set white background 
subplot(211)
plot(acc(:,1),acc(:,3),'color','red','LineWidth',2)
hold on
plot(time_eq_acc,acc_interp(:,3),'color','blue','LineWidth',1)
xticks(round(linspace(min(acc(:,1)), max(acc(:,1)), XTICKS),2)) 
yticks(round(linspace(MIN_ACC, MAX_ACC, YTICKS),3))
ylim([MIN_ACC-1 MAX_ACC+1])
hold off
grid on
legend('Original','Interpolated')
title('Interpolation of Y-acceleration (Sensor data, m/s^2)')

subplot(212)
plot(time_kin,y,'color','red','LineWidth',2)
hold on
plot(time_eq_kin,y_interp,'color','blue','LineWidth',1)
xticks(round(linspace(min(time_eq_kin), max(time_eq_kin), XTICKS),2)) 
yticks(round(linspace(min(y), max(y), YTICKS),3))
ylim([min(y) max(y)])
hold off
grid on
legend('Original','Interpolated')
title('Interpolation of Y-position (Kinect data, m/s^2)')
%% if ok , replace the data with the one interpolated
acc = [];
acc(:,1) = time_eq_acc;
acc(:,2) = acc_interp(:,2);
acc(:,3) = acc_interp(:,3);
acc(:,4) = acc_interp(:,4);

time_kin = time_eq_kin;
x = x_interp';
y = y_interp';
z = z_interp';
clear acc_interp x_interp y_interp z_interp time_eq_kin time_eq_acc

%% INTEGRAL OF SENSOR ACCELERATION DATA (with detrend filtering) 

%First, remove the mean from acc data. Then compute the integral 
vel = cumtrapz(acc - mean(acc));

% ***********************************************************
%Secondly, remove the trend of the signal (use Detrend or filtfilt func)
d1 = designfilt('highpassiir','FilterOrder',2, ...
    'HalfPowerFrequency',0.05,'DesignMethod','butter');
% vel_detrend = filtfilt(d1,vel);
vel_detrend = detrend(cumtrapz(acc - mean(acc)),5);
% vel_detrend = vel;
AXES = 3; % in this case, 'y' -> 3 
% ***********************************************************

vel(:,1) = acc(:,1);
vel_detrend(:,1) = acc(:,1);
% MIN_VEL = min([min(vel(:,2)), min(vel(:,3)), min(vel(:,4))]); % to get same scale on XYZ acc. 
% MAX_VEL = max([max(vel(:,2)), max(vel(:,3)), max(vel(:,4))]);
MIN_VEL = min(vel(:,AXES)); % to get same scale on XYZ acc. 
MIN_VEL = max(vel(:,AXES)); % to get same scale on XYZ acc. 
% MAX_VEL = max([max(vel(:,2)), max(vel(:,3)), max(vel(:,4))]);

%% CROSS-CORRELATION BETWEEN KINECT AND SENSOR DATA 

% ***********************************************************
SHIFT = 0;
SHIFT_KIN = 94; %best match 94
% ***********************************************************

FACTOR = 40; %for plotting purposes
FACTOR2 = 20000; %for plotting purposes
[c_vel,lags_vel] = xcorr(vel_detrend(SHIFT+1:end,3),diff(y(1:end))/mean(diff(acc(:,1)))); % cross-correlation between velocity
[max_c_vel, idx_c_vel] = max(c_vel);
[c_acc,lags_acc] = xcorr(acc(SHIFT+1:end,3),diff(diff(y(1:end)))/mean(diff(acc(:,1)))); % cross-correlation between acceleration
[max_c_acc, idx_c_acc] = max(c_acc);

figure()
set(gcf,'color','w'); %set white background 
ax1= subplot(411)
stem(lags_vel,c_vel)
title('Cross-correlation between VELOCITY by Kinect and sensor data')
xlabel('lag (samples)')
ylabel('Correlation')
xticks(round(linspace(lags_vel(1),lags_vel(end), length(lags_vel)/20)))
ylim([-max_c_vel-max_c_vel/5 max_c_vel+max_c_vel/5])
grid on 
hold on 
scatter(lags_vel(idx_c_vel), max_c_vel, 100,'blue','filled')

ax2 = subplot(412)
stem(lags_acc,c_acc, 'Color', 'magenta')
title('Cross-correlation between ACCELERATION by Kinect and sensor data')
xlabel('lag (samples)')
ylabel('Correlation')
xticks(round(linspace(lags_acc(1),lags_acc(end), length(lags_acc)/20)))
grid on 
hold on 
ylim([-max_c_acc-max_c_acc/5 max_c_acc+max_c_acc/5])
scatter(lags_acc(idx_c_acc), max_c_acc, 100,'magenta','filled')

temp = length(diff(y(1:end)))
ax3 = subplot(413)
plot(vel_detrend(SHIFT+1:end,1),vel_detrend(SHIFT+1:end,3)/FACTOR, 'Color', 'red', 'LineWidth',2)
hold on 
plot(acc(SHIFT_KIN+2:temp+SHIFT_KIN+1), diff(y(1:end))/mean(diff(acc(:,1))), 'Color', 'black','LineWidth',1) %*FACTOR
title('VELOCITY by Kinect and sensor data')
xlabel('time (sec)')
ylabel('m/s')
legend('vel sensor','vel kin')
xticks(linspace(acc(SHIFT+1,1),acc(end,1), (length(acc(:,1))- SHIFT+1)/20))
grid on 

% scatter(lags_acc(idx_c_acc), max_c_acc, 100,'red','filled')
temp = length(diff(diff(y(1:end))))
ax4 = subplot(414)
plot(acc(SHIFT+1:end,1),acc(SHIFT+1:end,3)-mean(acc(1:end,3)), 'Color', 'red', 'LineWidth',2)
hold on 
plot(acc(SHIFT_KIN+3:temp+SHIFT_KIN+2),diff(diff(y(1:end)))/(mean(diff(acc(:,1)))^2), 'Color', 'black','LineWidth',1)
title('ACCELERATION by Kinect and sensor data')
xlabel('time (sec)')
ylabel('m/s^2')
legend('acc sensor','acc kin')
xticks(linspace(acc(SHIFT+1,1),acc(end,1), (length(acc(:,1))- SHIFT+1)/20))
yticks(round(linspace(min(acc(1:end,3))*2,max(acc(1:end,3))*2, YTICKS/1.5),0))
grid on 

linkaxes([ax1 ax2],'x')

%% SYNCHRONIZATION
%  'cut' the longest time series to synchronize data. In this case: 
temp = []; temp = acc; acc = [];
acc(:,1) = temp(SHIFT_KIN+1:SHIFT_KIN+length(time_kin),1) - temp(SHIFT_KIN+1,1);
acc(:,2) = temp(SHIFT_KIN+1:SHIFT_KIN+length(time_kin),2);
acc(:,3) = temp(SHIFT_KIN+1:SHIFT_KIN+length(time_kin),3);
acc(:,4) = temp(SHIFT_KIN+1:SHIFT_KIN+length(time_kin),4);

temp = []; temp = vel_detrend; vel_detrend = [];
vel_detrend(:,1) = temp(SHIFT_KIN+1:SHIFT_KIN+length(time_kin),1) - temp(SHIFT_KIN+1,1);
vel_detrend(:,2) = temp(SHIFT_KIN+1:SHIFT_KIN+length(time_kin),2);
vel_detrend(:,3) = temp(SHIFT_KIN+1:SHIFT_KIN+length(time_kin),3);
vel_detrend(:,4) = temp(SHIFT_KIN+1:SHIFT_KIN+length(time_kin),4);

temp = []; temp = vel; vel = [];
vel(:,1) = temp(SHIFT_KIN+1:SHIFT_KIN+length(time_kin),1) - temp(SHIFT_KIN+1,1);
vel(:,2) = temp(SHIFT_KIN+1:SHIFT_KIN+length(time_kin),2);
vel(:,3) = temp(SHIFT_KIN+1:SHIFT_KIN+length(time_kin),3);
vel(:,4) = temp(SHIFT_KIN+1:SHIFT_KIN+length(time_kin),4);

%% MAX PEAK DETECTION ALGORITHM (SENSOR DATA)
% Discard initial samples, related to the idle phase before the test 

% ************ HYPER PARAMETERS *****************
START_FROM = 0.01; % second by trial 
END_IN = acc(end,1); % second 
THR_ACC_MAX = 11.0; % detect peak when the acc is over the threshold 
THR_ACC_MIN = 8.0; %(this is more difficult to the multiple presence of min)
FREQ_IMU = ceil(FREQ_INTERP/3); % sampling frequency of the IMU /3 se 
%ricerca picchi ogni 0.5 sec (non /2, meglio essere più 'stretti' con la
%window'
AXES = 3; % in this case, 'y' -> 3 
% ***********************************************

% acc = acc_orig; 
idx = find((acc(:,1) >= START_FROM) & (acc(:,1) <= END_IN));
temp = [];
for i = 1:size(acc,2)
    temp = [temp, acc(idx,i)]; 
end 
acc_orig = acc; acc = [];  acc = temp; 
acc(:,1) = acc(:,1) - min(acc(:,1)); % start from 0 sec

% search for the max/min each 1/x sec <-> each FREQ_IMU/x samples
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
    if((idx_max_peak(i)-FREQ_IMU+ceil(FREQ_IMU/5) <= idx_max_peak(i-1)) || (idx_max_peak(i) == idx_max_peak(i-1)))
        rem_idx = [rem_idx, i];
    end
end 
max_peak(rem_idx) = [];
idx_max_peak(rem_idx) = [];

rem_idx =[];
for i=2:length(idx_min_peak)
    if((idx_min_peak(i)-FREQ_IMU+ceil(FREQ_IMU/5) <= idx_min_peak(i-1)) || (idx_min_peak(i) == idx_min_peak(i-1)))
        rem_idx = [rem_idx, i]
    end
end 

min_peak(rem_idx) = [];
idx_min_peak(rem_idx) = [];

YTICKS = 30;
figure()
set(gcf,'color','w'); %set white background 
plot(acc(:,1),acc(:,AXES),'color','red','LineWidth',3)
hold on
scatter(acc(idx_max_peak,1),acc(idx_max_peak,AXES),50,'blue','filled')
hold on 
scatter(acc(idx_min_peak,1),acc(idx_min_peak,AXES),50,'green','filled')
xticks(acc(FREQ_IMU+1:FREQ_IMU:length(acc(:,1)),1))
% xticks(round(linspace(min(acc(:,1)), max(acc(:,1)), XTICKS),2)) 
% yticks(round(linspace(min(acc(:,3)), max(acc(:,3)), YTICKS),2)) % relative scale 
yticks(round(linspace(MIN_ACC, MAX_ACC, YTICKS),3))
ylim([MIN_ACC-1 MAX_ACC+1])
xlabel('Window where the min/max has been searched')
ylabel('Acceleration (m/s^2)')
grid on 
title('Y-acceleration (Sensor Data, m/s^2)')

%% RECOGNITION OF SKIP STEPS (KINECT DATA)
% This algorithm compute the derivative of the y-position and search for 
% regions where the der is below the threshold. Then, a seach of the first 
% minimum is performed -> that's the point in which the step has occurred. 

% ************ HYPER PARAMETERS *****************
DERIVATIVE_THR = -0.009; % detect step when the derivative is below theshold 
% FREQ_KIN_ORIG = round(mean(1./diff(time_kin)));
FREQ_KIN = ceil(FREQ_INTERP/2.5); % sampling frequency of the KINECT
%mettere > /2 se ricerca picchi ogni 0.5 sec
AXES_KIN = y; 
NUMBER_OF_STEPS = 30; %number of peak to be recognized
% ***********************************************

der_AXES_KIN = diff(AXES_KIN)/mean(diff(acc(:,1)));
der2_AXES_KIN = diff(der_AXES_KIN)/mean(diff(acc(:,1)));
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
    if((idx_der_min(i)-FREQ_KIN+ceil(FREQ_KIN/10) <= idx_der_min(i-1)) || (idx_der_min(i) == idx_der_min(i-1)))
        rem_idx = [rem_idx, i];
    end
end 
min_der(rem_idx) = [];
idx_der_min(rem_idx) = [];

% YTICKS = 100
figure()
set(gcf,'color','w'); %set white background 
plot(der_time,der_AXES_KIN,'color','black','LineWidth',3)
hold on
scatter(der_time(idx_der_min),der_AXES_KIN(idx_der_min),50,'blue','filled')
hold on
plot(der_time,zeros(length(der_time),1),'color','red','LineWidth',2)
xticks(round(linspace(min(der_time), max(der_time), XTICKS),2))
xticks(der_time(FREQ_KIN+1:FREQ_KIN:length(der_time)))
% yticks(round(linspace(min(der_AXES_KIN), max(der_AXES_KIN), YTICKS),4))
yticks(linspace(min(der_AXES_KIN), max(der_AXES_KIN), 25))
grid on 
title('derivative of Y-position (Kinect)')
xlabel('time (s)')
ylabel('m/s')


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


%% Plot synchronized data

% ***********************************************
YTICKS = 10;
% ***********************************************
figure()
set(gcf,'color','w'); %set white background 
ax1 = subplot(231)
plot(time_kin, AXES_KIN,'color','black','LineWidth',3)
hold on
scatter(time_kin(idx_der_min),AXES_KIN(idx_der_min),50,'magenta','filled')
hold on
scatter(time_kin(idx_der_null),AXES_KIN(idx_der_null),50,'cyan','filled')
% xticks(der_time(idx_der_null))
xticks(acc(idx_min_peak,1)) 
yticks(round(linspace(min(AXES_KIN), max(AXES_KIN), YTICKS),3))
grid on 
title('Y-position (Kinect)')
ylabel('m')
legend('y position', 'min deriv','derive null (skip down and up)')
% xlabel('Seconds (s)') 

ax2 = subplot(232)
plot(der_time,der_AXES_KIN,'color','black','LineWidth',3)
hold on
scatter(der_time(idx_der_min),der_AXES_KIN(idx_der_min),50,'magenta','filled')
hold on
scatter(der_time(idx_der_null),der_AXES_KIN(idx_der_null),50,'cyan','filled')
hold on
plot(der_time,zeros(length(der_time),1),'color','blue','LineWidth',1)
% xticks(der_time(idx_der_null))
xticks(acc(idx_min_peak,1)) 
yticks(linspace(min(der_AXES_KIN), max(der_AXES_KIN), 10))
xlim([0 max(der_time)])
grid on 
title('derivate of Y-position (Kinect)')
ylabel(' m/s')
legend('Derivative','Deriv minimum','Deriv null (skip down)','line = 0')

ax3 = subplot(233);
plot(der_time(2:end),der2_AXES_KIN,'color','black','LineWidth',3)
hold on
% scatter(der_time(idx_der_min),der2_AXES_KIN(idx_der_min),50,'magenta','filled')
hold on
% scatter(der_time(idx_der_null),der_AXES_KIN(idx_der_null),50,'cyan','filled')
hold on
% plot(der_time(2:end),zeros(length(der_time(2:end)),1),'color','blue','LineWidth',1)
% xticks(der_time(idx_der_null))
xticks(acc(idx_min_peak,1)) 
yticks(linspace(min(diff(der_AXES_KIN)/mean(diff(acc(:,1)))), max(diff(der_AXES_KIN)/mean(diff(acc(:,1)))), 10))
xlim([0 max(der_time)])
grid on 
title('derivate^2 of Y-position (Kinect)')
ylabel(' m/s^2')
% legend('Acceleration K.','Deriv minimum','Deriv null (skip down)','line = 0')
legend('Acceleration K.')


ax5 = subplot(235)
plot(vel(:,1),vel(:,AXES),'color','red','LineWidth',1)
hold on
plot(vel_detrend(:,1),vel_detrend(:,AXES),'color','red','LineWidth',3)
xticks(acc(idx_min_peak,1)) % skip up
yticks(round(linspace(min(vel(:,AXES)), max(vel(:,AXES)), YTICKS),3)) 
ylim([min(vel(:,AXES))-1 max(vel(:,AXES))+1])
xlim([0 max(acc(:,1))])
grid on 
title('Y-speed, integration of acceleration (m/s) - Sensor data')
% legend('Acceleration values','Peak (skip down)','Peak (skip up)')
% legend('speed','speed (detrend, 5th)','skip down','skip up')
legend('speed','speed (filt butt 2nd)','skip down','skip up')
ylabel(' m/s')
xlabel('time (s)')


ax6 = subplot(236)
plot(acc(:,1),acc(:,AXES),'color','red','LineWidth',3)
hold on
scatter(acc(idx_max_peak,1),acc(idx_max_peak,AXES),50,'blue','filled')
hold on
scatter(acc(idx_min_peak,1),acc(idx_min_peak,AXES),50,'green','filled')
% xticks(round(linspace(min(acc(:,1)), max(acc(:,1)), XTICKS),2))  
xticks(acc(idx_min_peak,1)) % skip up
% xticks([acc(idx_max_peak,1)]) % skip down
% xticks(acc(idx_peak,1))
% yticks(round(linspace(min(acc(:,3)), max(acc(:,3)), YTICKS),2)) % relative scale 
yticks(round(linspace(MIN_ACC, MAX_ACC, YTICKS),3)) 
ylim([MIN_ACC-1 MAX_ACC+1])
xlim([0 max(acc(:,1))])
grid on 
title('Y-acceleration (m/s^2) - Sensor data')
% legend('Acceleration values','Peak (skip down)','Peak (skip up)')
legend('Acceleration','skip down','skip up')
ylabel(' m/s^2')
xlabel('time (s)')

linkaxes([ax1 ax2 ax3 ax5 ax6],'x')
%% Find the amplitude of the displacement from the acceleration and Kinect

% ***********************************************
temp = (min(length(max_peak), length(min_peak)));
a_acc = []; a_disp = [];
a_acc = (max_peak(1:temp) - min_peak(1:temp))/2; 
f_ideal = 2; % two steps each second
f_acc = mean(1./diff(acc(idx_max_peak, 1)));
f_kin = mean(1./diff(der_time(idx_der_min)));
a_disp = mean(a_acc./(2*pi*f_acc)^2);
max_peak_pos = max_peak./((2*pi*f_kin)^2);
min_peak_pos = min_peak./((2*pi*f_kin)^2);
% ***********************************************

% find the amplitude of displacement in the kinect 
% by computing the difference between even indexes (high peaks) and odd indexes(low peaks)
min_kin =  AXES_KIN(idx_der_null(1:2:length(idx_der_null)));
max_kin =  AXES_KIN(idx_der_null(2:2:length(idx_der_null)));
kin_disp = abs( max_kin - min_kin);
kin_disp = mean(kin_disp)/2;

kin_baseline = mean(min_kin) + kin_disp;
% kin_baseline = mean(max_kin) - kin_disp

% Compute MAE, MSE, RMSE between peaks 
% temp =  (min(length(min_kin), length(min_peak)));
% 
% MAE_kin_acc = mean((abs(min_kin(1:temp)' - min_peak_pos(1:temp))));
% MSE_kin_acc = mean((abs(min_kin(1:temp)' - min_peak_pos(1:temp))).^2);
% RMSE_kin_acc = sqrt(MSE_kin_acc);
ERROR_KIN_ACC = abs(a_disp - kin_disp);
% ERROR_PERC_KIN_ACC = abs(a_disp - kin_disp)*100;

%% ANALYSIS OF KINECT DATA - 4 

% ***********************************************
idx = min(idx_der_null):max(idx_der_null);
time_ground_thruth = time_kin(idx); 
amplitude_ground_thruth = a_disp; %take the accelerometer values as gt 
% build the sine function only between peaks which have been found in the kinect data 
% Y_GROUND_TRUTH = amplitude_ground_thruth*sin(2*pi*f*time_ground_thruth) + kin_baseline;
% Y_GROUND_TRUTH = amplitude_ground_thruth*sin(2*pi*f_kin*time_ground_thruth) + kin_baseline;
Y_GROUND_TRUTH = amplitude_ground_thruth*sin(2*pi*f_acc*time_ground_thruth) + kin_baseline;
Z_GROUND_TRUTH = zeros(length(time_ground_thruth),1) + 3.00; % distance from the kinect
X_GROUND_TRUTH = zeros(length(time_ground_thruth),1) + mean(x(idx)); % consider offset 
% ***********************************************


figure()
set(gcf,'color','w'); %set white background 
ax1 = subplot(311);
plot(time_ground_thruth, Y_GROUND_TRUTH,'LineWidth',2,'Color','red')
hold all 
plot(time_kin(idx), AXES_KIN(idx),'LineWidth',2,'Color','black')
ylim([mean(min_kin)-0.25 mean(min_kin)+0.25])
yticks(linspace(-1,+1,51)) %101a
xticks(time_kin(idx_der_null(1:end)))
grid on
legend('Y GROUND THRUTH(accel data)','Kinect data')
ylabel('Meters (m)')
title([ 'ERROR: ' ,num2str(ERROR_KIN_ACC), 'm , ERROR (%): ',num2str(ERROR_KIN_ACC*100),  ...
        ';                      Ideal f_S: ' ,num2str(f_ideal), ...
        ',    Sens f_S: ' ,num2str(f_acc), ...
        ',    Kin f_S: ' ,num2str(f_kin) ])
    
ax2 = subplot(312);
plot(time_ground_thruth, Z_GROUND_TRUTH,'LineWidth',2,'Color','red')
hold on
plot(time_kin(idx), z(idx),'LineWidth',2,'Color','black')
hold on
% scatter(time_kin(idx_der_null(1:end)), z(idx_der_null),50,'red','filled')
ylim([min(mean(Z_GROUND_TRUTH),mean(z(idx)))-0.25 max(mean(Z_GROUND_TRUTH),mean(z(idx)))+0.25])
% yticks(linspace(round(mean(z(idx))-0.25,3),round(mean(z(idx))-0.25,3),11)) %101a
% xticks(round(time_kin(idx)),2)
xticks(time_kin(idx_der_null(1:end)))
grid on
legend('Z GROUND THRUTH(const distance)','Kinect data')
ylabel('Meters (m)')

ax3 = subplot(313);
plot(time_ground_thruth, X_GROUND_TRUTH,'LineWidth',2,'Color','red')
hold all 
plot(time_kin(idx), x(idx),'LineWidth',2,'Color','black')
ylim([min(mean(X_GROUND_TRUTH),mean(x(idx)))-0.25 max(mean(X_GROUND_TRUTH),mean(x(idx)))+0.25])
% yticks(linspace(round(mean(z(idx))-0.25,3),round(mean(z(idx))-0.25,3),11)) %101a
% xticks(round(time_kin(idx)),2)
xticks(time_kin(idx_der_null(1:end)))
grid on
legend('X GROUND THRUTH(const distance)','Kinect data')
ylabel('Meters (m)')
xlabel('Seconds (s)')
linkaxes([ax1  ax2  ax3 ],'x')

