# Kinect Data Acquisition Guide

## Overview
This document provides detailed instructions on how to use the `GetDataFromKinect` script for acquiring Kinect V2 data. The script is a modified version of the [program available from MathWorks Italia](https://it.mathworks.com/help/imaq/view-skeletal-data-from-kinect-for-windows-v2.html) specifically tailored for the `Final_project_3DVBM_kinect_IMU` project. In the version here provided you need only to run the script to start collecting the data. 

## Using the `GetDataFromKinect` Script
The `GetDataFromKinect` script is designed to capture and store Kinect data for analysis. When you run this script, it will store frames in `colorImg` and joint positions in `metadata`, which is the main structure to consider for further analysis.

### Steps for Acquisition
1. **Running the Script:** Simply execute the `GetDataFromKinect` script in MATLAB to start acquiring data from Kinect V2.
2. **Storing Data:** The script will automatically store the captured frames and joint positions. Ensure you save this data before proceeding with any other scripts in the project.

### Customizing Data Acquisition
- **Change Acquisition Time:** You can adjust the duration of data capture by changing the number of frames in the script.
  - In line 22 of the script, set the `framePerTrig` value considering that Kinect's frame rate is approximately 30 frames per second. For example, for a 10-second acquisition, set `framePerTrig = 300`.
- **Troubleshooting Errors:** If you encounter errors during data acquisition, please check the following:
  - Ensure that you have at least MATLAB R2021b installed.
  - If the Kinect is not providing frames correctly, increase the pause duration of the script. This adjustment allows the Kinect to stabilize its frame delivery to the PC.

### Additional Functionality
- **Creating a GIF from Kinect Frames:** there is an optional section that enables you to create a GIF from the frames captured by Kinect. This feature is useful for visually reconstructing the activity performed during data acquisition.
    - To enable this feature, uncomment lines >38 and run the script.
    - To disable this feature, comment out lines >38 and run the script.