Writeup

Step 1: Sensor Noise
Took the standard deviation from the sensor data and added them to 06_Sensor Noise file.

Step2: Attitude Estimation
Implemented the improved integration scheme with non-linear estimation.

Step3: Prediction Step
Implementing the correct estimation step in the PredictState function. Calculation with the Rgb prime matrix and the state. The step prediction follows the EKF equation from the Estimation for Quadrotors paper. The accelerations need to be converted into the global system and the gravity has to be taken into account.

Step 4: Magnomether Update
Implementing the Magnetometer equation from the Estimation for Quadrotors paper.

Step 5: Closed Loop + GPS Update
From the mentioned paper i used the GPS equation and implemented it.

Step 6: Adding Your Controller.
I added the controller from the Controll project and without larger paremter detuning the drone passed all tests from step 1 to 5.
