# README for 'main_processing.m'

This script provides a comprehensive analysis of Kinect's position tracking accuracy compared to accelerometer data. It includes preprocessing, synchronization, peak detection, and comparison phases, offering insights into the relative performance of the Kinect sensor.

## Workflow Overview

The script is divided into several key sections, each handling a specific part of the analysis. 
All the Hyperparameters to be set from the user are inserted into a paragraph starting and ending with '*' symbols. For example:
% ***********************************************************
% Interpolate data with the max between the two sampling frequency, 
% in order to avoid loss of information     
FREQ_INTERP = FREQ_IMU_MAX;
% ***********************************************************


### 1. Initialization
- Clearing the MATLAB environment.
- Loading the skeleton connection map and data files.

### 2. Synchronization of Sensor and Kinect Data
- Extracting start and end times from Kinect metadata.
- Filtering indexes based on the Kinect logging timeframe.

### 3. Preprocessing of Accelerometer and Gyroscope Measurements
- Applying hyperparameters for synchronization.
- Preprocessing IMU data (accelerometer and gyroscope) to synchronize with Kinect data.

### 4. Preprocessing of Kinect Metadata
- Setting parameters like height and joint of interest.
- Extracting Kinect tracking data for the specified joint.

### 5. First Inspection of Data
- Visualizing initial data from Kinect and sensor for comparison.
- Plotting X, Y, Z positions and accelerations.

### 6. Interpolation of Kinect and Sensor Data
- Calculating minimum and maximum frequencies for both Kinect and IMU data.
- Interpolating data to synchronize Kinect and sensor readings.

### 7. Integral of Sensor Acceleration Data
- Computing the velocity from accelerometer data.
- Applying detrend filtering and cross-correlation analysis between Kinect and sensor data.

### 8. Synchronization
- Adjusting data lengths based on synchronization analysis.

### 9. Max Peak Detection Algorithm
- Identifying max and min peaks in the accelerometer data.
- Detecting skip steps in Kinect data based on derivative analysis.

### 10. Plotting Synchronized Data
- Visualizing the synchronized data from both Kinect and sensor.
- Comparing the amplitude of displacement and frequency.

### 11. Analysis of Kinect Data
- Building and comparing a ground truth model based on accelerometer data.

