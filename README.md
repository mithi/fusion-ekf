# Introduction
This is an extended Kalman Filter implementation in C++ for fusing lidar and radar sensor measurements.
A Kalman filter can be used anywhere you have uncertain information about some dynamic system, 
and you want to make an educated guess about what the system is going to do next. 

**In this case, we have two 'noisy' sensors:**
- A lidar sensor that measures our position in cartesian-coordinates `(x, y)`
- A radar sensor that measures our position and velocity in polar coordinates `(rho, phi, drho)`

**We want to predict our position, and how fast we are going in what direction at any point in time:**
- In essence: the position and velocity of the system in cartesian coordinates: `(x, y, vx, vy)`
- Note that we are assuming a **constant velocity model (CV)** for this particular system

**This extended kalman filter does just that.** 
- NOTE: You can also check my (almost the same) implementation in Python and Jupyter Notebook below:
- [Fusion-EKF-Python](https://github.com/mithi/Fusion-EKF-Python)

-----
# Table of Contents
- Dependencies
- Basic Usage
- IMPORTANT NOTES FOR REVIEWER
- RUBRIC CHECK LIST

-----

# Dependencies
This project was tested on a Macbook Pro with **Mac0S Sierra 10.12.4** with the following:
- cmake version 3.7.2
- GNU Make 4.2.1
- gcc GNU 6.3.0

### Here's a sample recipe:
- Install xcode and xcode command line tools, if you haven't yet
```
$xcode-select --install
```

- Install homebrew if you haven't yet
```
$ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

- Install make 
```
$brew install homebrew/dupes/make --with-default-names
```

- Install cmake 
```
$brew install cmake
```

- check if you've have the correct version or higher 
```
$gcc-6 --version
$make --version
$cmake --version
```

-----
# Basic Usage
- Clone this repository 
```
$git clone https://github.com/mithi/Fusion-EKF-CPP/
```
- Go inside the `build` folder and compile: 
```
$cd build
$CC=gcc-6 cmake .. && make
```

- To execute inside `build` folder use the following format: 

```
$./extendedKF /PATH/TO/INPUT/FILE /PATH/TO/OUTPUT/FILE
$./ExtendedKF ../data/data-1.txt ../data/out-1.txt
```

- Please use the following format for your input file
```
[L(for lidar)] [m_x] [m_y] [t] [r_x] [r_y] [r_vx] [r_vy]
[R(for radar)] [m_rho] [m_phi] [m_drho] [t] [r_px] [r_py] [r_vx] [r_vy]

Where:
(m_x, m_y) - measurements by the lidar
(m_rho, m_phi, m_dho) - measurements by the radar in polar coordinates
(t) - timestamp in unix/epoch time the measurements were taken
(r_x, r_y, r_vx, r_vy) - the real ground truth state of the system

Example:
R 8.60363 0.0290616 -2.99903  1477010443399637  8.6 0.25  -3.00029  0
L 8.45  0.25  1477010443349642  8.45  0.25  -3.00027  0 
```

- The program outputs the predictions in the following format in your input file
```
[p_x] [p_y] [p_vx] [p_vy] [m_x] [m_y] [r_px] [r_py] [r_vx] [r_vy]

Where:
(p_x, p_y, p_vx, p_vy) - the predicted state of the system by FusionEKF
(m_x, m_y) - the position value as measured by the sensor converted to cartesian coordinates
(r_x, r_y, r_vx, r_vy) - the real ground truth state of the system

Example:
4.53271 0.279 -0.842172 53.1339 4.29136 0.215312  2.28434 0.226323
43.2222 2.65959 0.931181  23.2469 4.29136 0.215312  2.28434 0.226323
```

# IMPORTANT NOTES FOR REVIEWER
- Please check the PDF in the `docs` folder for important findings and explanations
- [Variance improvement and visualizations PDF (ax, ay = 5, 5)](https://github.com/mithi/Fusion-EKF-CPP/blob/master/docs/FusionEKF-variances-visualization.pdf)
- [Variance improvement and visualizations PDF 2 (ax, ay = 9, 9)](https://github.com/mithi/Fusion-EKF-CPP/blob/master/docs/FusionEKF-variances-visualization-B.pdf)

- Please also check the rubric checklist below this section for additional information 
- NOTE: this code is written from scratch
- The file structure suggested and starter code by Udacity is *not* used
- [Udacity starter code](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)


-----

# RUBRIC CHECK LIST
1. CHECK: Code must compile 
- Please take a look at [CMakeLists.txt](https://github.com/mithi/Fusion-EKF-CPP/blob/master/CMakeLists.txt)
2. CHECK: The required RMSE for the two provided data sets must be met
- Please check the following images for proof
- [Terminal Output](https://github.com/mithi/Fusion-EKF-CPP/blob/master/images/Terminal-Output.png)
- this is for `ax, ay = 5, 5` with the following specs:
- - First run: Used `data-1.txt`, with my own measurement covariances matrices
- - 2nd run: Used `data-2.txt`, with my own measurement covariances matrices
- - 3rd run: Used `data-1.txt`, with my Udacity's measurement covariances matrices
- - 4th run: Used `data-2.txt`, with my Udacity's measurement covariances matrices
- [Terminal Output-2](https://github.com/mithi/Fusion-EKF-CPP/blob/master/images/Terminal-Output-2.png)
- this is for `ax, ay = 9, 9` with the following specs:
- - First run: Used `data-1.txt`, with my own measurement covariances matrices
- - 2nd run: Used `data-2.txt`, with my own measurement covariances matrices
- - 3rd run: Used `data-1.txt`, with my Udacity's measurement covariances matrices
- - 4th run: Used `data-2.txt`, with my Udacity's measurement covariances matrices
3. CHECK: The general sensor fusion algorithm flow as taught in the lessons must be used
- Please check ```line 76 to 112``` of the following file...
- to see methods ```process()``` and ```update()``` [src/fusionekf.cpp](https://github.com/mithi/Fusion-EKF-CPP/blob/master/src/fusionekf.cpp)
4. CHECK: The algorithm handles the first measurement correctly
- Please check ```line 68 to 74``` of [src/fusionekf.cpp](https://github.com/mithi/Fusion-EKF-CPP/blob/master/src/fusionekf.cpp)
5. CHECK: The algorithm predicts before updating
- Please check ```line 83 and 107``` of [src/fusionekf.cpp](https://github.com/mithi/Fusion-EKF-CPP/blob/master/src/fusionekf.cpp)
6. CHECK: The algorithm handles radar and lidar measurements correctly 
- Please check the following lines of the following files:
- ```lines 26 to 47``` of [src/datapoint.cpp](https://github.com/mithi/Fusion-EKF-CPP/blob/master/src/datapoint.cpp)
- ```lines 3 to 42``` of [src/tools.cpp](https://github.com/mithi/Fusion-EKF-CPP/blob/master/src/tools.cpp)
- ```lines 45 to 74``` of [src/tools.cpp](https://github.com/mithi/Fusion-EKF-CPP/blob/master/src/tools.cpp)
- ```lines 16 to 37 and 92 to 105``` of [src/fusionekf.cpp](https://github.com/mithi/Fusion-EKF-CPP/blob/master/src/fusionekf.cpp)
7. CHECK: The algorithm should avoid unnecessary calculations
- Yes I think I did this. 

---


