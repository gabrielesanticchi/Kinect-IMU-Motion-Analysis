# README for PlottingScript
The PlottingScript is designed to visualize the movements of joints captured by a Kinect sensor over time. It plots the X, Y, Z positions of a specified joint, starting from the moment the Kinect recognizes a skeleton. This script is particularly useful for analyzing joint movements post data acquisition, providing insights into the dynamics and timing of physical activities.
### Script Features
- Extraction of joint positions over time.
- Plotting of X, Y, Z positions.
- Analysis of acquisition frequency.
- Animation of the skeleton based on joint positions for a comprehensive visual representation.

---

## Workflow Description

### 1. Initialization and Data Loading
- The script starts by clearing the console and closing all figures.
- Kinect metadata is loaded from a `.mat` file.

### 2. Time Synchronization
- The script synchronizes the time of each frame with the acquisition time.
- Time is adjusted to start from zero at the first frame where joint positions are acquired.

### 3. Position Extraction
- The script extracts the X, Y, Z positions of a specific joint (defined by index `j`) from the Kinect metadata.
- The Y position is adjusted based on the height of the Kinect from the ground (`y_position_ground`).

### 4. Frequency Analysis
- The script calculates the acquisition frequency for each sample and plots it.

### 5. Plotting Joint Positions
- Three subplots are created to show the X, Y, Z positions of the joint over time.
- The positions are plotted against the synchronized time vector.

### 6. Skeleton Animation
- The script animates the skeleton based on joint positions over a specified range of frames.
- It includes the visualization of different body parts such as arms, legs, and the chest.
- The animation is displayed in a 3D plot, and the script pauses briefly between frames to create the animation effect.

### 7. Additional Animated Plot
- An animated line plot is created for X, Y, Z positions over the selected frame range.

---

## Usage Instructions
- Load the script in MATLAB.
- Ensure the required `.mat` file with Kinect metadata is in the same directory as the script or specify the correct path.
- Run the script to visualize the joint movements and skeleton animation.
- Adjust the joint index `j` and `y_position_ground` as needed for different analyses.
