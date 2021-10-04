# Least-Squares-Odometry-Calibration
A calibration method for odometry of mobile robots based on nonlinear Least-squares technique.

## Dependencies
- Boost
- Eigen
## Running Instructions

- `git clone git@github.com:sManohar201/Least-Squares-Odometry-Calibration.git`
- `cd Least-Squares-Odometry-Calibration`
- `mkdir build && cd build`
- `cmake .. && make`
- `./least_squares_calibraion`

## Brief Explanation
Calibration is the process of find the extrinsic and intrinsic parameters of the sensor that causes it's readings to deviate from ground truth measurements. So, calibration is an essential step in preparing the sensors for real-world applications. There are different techniques to achieve calibration, one such is using Non-linear least squares minimization. This is an offline calibration technique which requires groud truth data. 

The steps involved in a least squares minimization is shown in the figure below. 
<img heigth="100" src="resources/LS.png">

Final results for the given data is plotted in the image below.
<img height="550" width="800" src="resources/odometry-calibration-result.png">
