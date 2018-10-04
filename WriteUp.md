# Estimation Project #

## The Goal of this Porject ##
In this project, you will be developing an estimator to be used by your controller to successfully fly a desired flight path using realistic sensors. This project is built on the same simulator you should now be familiar with from the Controls C++ project.

[image6]: ./data/6.gif
[image7]: ./data/7.gif
[image8]: ./data/8.gif
[image9]: ./data/9.gif
[image10]: ./data/10.gif
[image11_1]: ./data/11_1.gif
[image11_2]: ./data/11_2.gif
[image11_3]: ./data/11_3.gif


## The Tasks ##

Once again, you will be building up your estimator in pieces.  At each step, there will be a set of success criteria that will be displayed both in the plots and in the terminal output to help you along the way.

Project outline:

 - [Step 1: Sensor Noise](#step-1-sensor-noise)
 - [Step 2: Attitude Estimation](#step-2-attitude-estimation)
 - [Step 3: Prediction Step](#step-3-prediction-step)
 - [Step 4: Magnetometer Update](#step-4-magnetometer-update)
 - [Step 5: Closed Loop + GPS Update](#step-5-closed-loop--gps-update)
 - [Step 6: Adding Your Controller](#step-6-adding-your-controller)



### Step 1: Sensor Noise ###

First I runed the test scenario of '06_SensorNoise' to get the graph1 and graph2 data. From these data, I got the standard deviations of 'GPSPosXY' and 'AccelXY'.
The value of 'GPSPosXY' is 0.712 and the value of 'AccelXY' is 0.481. I wrote them to '06_SensorNoise.txt' config file. And I executed again.

![alt text][image6]

I saved the graph1,2 data files in folder named config/log_backup/06/.


### Step 2: Attitude Estimation ###

I implemented the code of Attitude Estimation in UpdateFromIMU function.

![alt text][image7]

### Step 3: Prediction Step ###

Prediction is composed of followed steps.
1) Predict State
2) Predict

![alt text][image8]

I tuned the QVelXYStd value from 0.05 to 0.2 to close to estimated VX.

![alt text][image9]

### Step 4: Magnetometer Update ###

I implemented the code of Magnetometer Update in UpdateFromMag function.

![alt text][image10]

To pass, I set the value of MagYawStd .1 -> .5 and QYawStd .05 -> .08

### Step 5: Closed Loop + GPS Update ###

I implemented the code of GPS Update in UpdateFromGPS function.

Case 1) Ideal estimator
Quad.UseIdealEstimator = 1
SimIMU.AccelStd = 0,0,0
SimIMU.GyroStd = 0,0,0
![alt text][image11_1]

Case 2) Realistic estimator with ideal IMU
Quad.UseIdealEstimator = 0
SimIMU.AccelStd = 0,0,0
SimIMU.GyroStd = 0,0,0
![alt text][image11_2]

Case 3) Realistic estimator with realistic IMU
Quad.UseIdealEstimator = 0
#SimIMU.AccelStd = 0,0,0
#SimIMU.GyroStd = 0,0,0
![alt text][image11_3]


### Step 6: Adding Your Controller ###

I added the control codes of previous project. and I got the all pass.

