###Writeup

###Step 1: Sensor Noise
Took the standard deviation from the IMU and GPS sensor data and added them to 06_SensorNoise.txt file.

'''
MeasuredStdDev_GPSPosXY = 0.7211
MeasuredStdDev_AccelXY = 0.5092
'''

'''
PASS: ABS(Quad.GPS.X-Quad.Pos.X) was less than MeasuredStdDev_GPSPosXY for 68% of the time
PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 69% of the time
'''

###Step2: Attitude Estimation
We had to replace the existing linear integration of the IMU data to the gyro. I did the non-linear integration with the help of the following transformation from the lectures in order to get a better estimate for larger angles:

'''
Mat3x3F rotation = Mat3x3F::Zeros();
rotation(0,0) = 1;
rotation(0,1) = sin(rollEst) * tan(pitchEst);
rotation(0,2) = cos(rollEst) * tan(pitchEst);
rotation(1,1) = cos(rollEst);
rotation(1,2) = - sin(rollEst);
rotation(2,1) = sin(rollEst) / cos(pitchEst);
rotation(2,2) = cos(rollEst) / cos(pitchEst);
V3F phi_theta_dot = rotation * gyro;

float predictedRoll = rollEst + phi_theta_dot.x * dtIMU;
float predictedPitch = pitchEst + phi_theta_dot.y * dtIMU;
ekfState(6) = ekfState(6) + phi_theta_dot.z * dtIMU;

// normalize yaw to -pi .. pi
if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;
'''

'''
PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
'''

###Step3: Prediction Step
First i used equation (52) from the Estimation for Quadrotors paper to get the RbgPrime values.

'''
float SPhi = sin(roll);
float SPsi = sin(yaw);
float STheta = sin(pitch);
float CPhi = cos(roll);
float CPsi = cos(yaw);
float CTheta = cos(pitch);

RbgPrime(0, 0) = - CTheta * SPsi;
RbgPrime(0, 1) = - SPhi * STheta * SPsi - CTheta * CPsi;
RbgPrime(0, 1) = - CPhi * STheta * SPsi + SPhi * CPsi;
RbgPrime(1, 0) = CTheta * CPsi;
RbgPrime(1, 1) = SPhi * STheta * CPsi - CPhi * SPsi;
RbgPrime(1, 2) = CPhi * STheta * CPsi + SPhi * SPsi;
'''

and g prime and covariance are calculated with the equations from the lecture a and the Algorithm 2 table from the Estimation for Quadrotors paper.
'''
MatrixXf v(3,1);
v(0,0) = accel.x * dt;
v(1,0) = accel.y * dt;
v(2,0) = accel.z * dt;

gPrime(0,3) = dt;
gPrime(1,4) = dt;
gPrime(2,5) = dt;

MatrixXf n = RbgPrime * v;
gPrime(3,6) = n(0,0);
gPrime(4,6) = n(1,0);
gPrime(5,6) = n(2,0);

ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;
'''

###Step 4: Magnomether Update
Implementing the Magnetometer equations (56-58) from the Estimation for Quadrotors paper.

'''
PASS: ABS(Quad.Est.E.Yaw) was less than 0.120000 for at least 10.000000 seconds
PASS: ABS(Quad.Est.E.Yaw-0.000000) was less than Quad.Est.S.Yaw for 77% of the time
'''

###Step 5: Closed Loop + GPS Update
Implementing the Magnetometer equations (53-55) from the Estimation for Quadrotors paper.

'''
zFromX(0) = ekfState(0);
zFromX(1) = ekfState(1);
zFromX(2) = ekfState(2);
zFromX(3) = ekfState(3);
zFromX(4) = ekfState(4);
zFromX(5) = ekfState(5);

for (int i = 0; i < 6; i++)
{
    hPrime(i,i) = 1;
}
'''

'''
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
'''
###Step 6: Adding Your Controller.
I added the controller from the Control project and without larger paremter detuning the drone passed all tests again.
