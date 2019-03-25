## Implemented Controller

### Implemented body rate control in C++.

This part is implemented in [QuadControl::BodyRateControl](./QuadControl.cpp#L104-L106):

```cpp
V3F pqrError = pqrCmd - pqr;
V3F u_bar = pqrError * kpPQR; // rad/s * 1/s (dimension kpPQR from u_bar = k * dpqr and u_bar = M/Ixx = p_dot)
momentCmd = u_bar * V3F(Ixx,Iyy,Izz);
```
### Implement roll pitch control in C++.

This part is implemented in [QuadControl::RollPitchControl](./QuadControl.cpp#L135-L158):

```cpp
float c = collThrustCmd / mass;
float R11 = R(0,0);
float R12 = R(0,1);
float R21 = R(1,0);
float R22 = R(1,1);
float R33 = R(2,2);
float b_x = R(0,2);
float b_y = R(1,2);

float b_x_c = CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
float b_y_c = CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);

if (collThrustCmd < 0)
{
    b_x_c = 0.f;
    b_y_c = 0.f;
}

float b_dot_x_c = kpBank * (b_x_c - b_x);
float b_dot_y_c = kpBank * (b_y_c - b_y);

pqrCmd.x = (1/R33) * (R21 * b_dot_x_c - R11 * b_dot_y_c);
pqrCmd.y = (1/R33) * (R22 * b_dot_x_c - R12 * b_dot_y_c);
pqrCmd.z = 0;
```

### Implement altitude controller in C++.

This part is implemented in [QuadControl::AltitudeControl](./QuadControl.cpp#L187-L198):

```cpp
float R22 = R(2, 2);

float z_error = posZCmd - posZ;
velZCmd = -CONSTRAIN(-velZCmd, -maxDescentRate, maxAscentRate);
float z_dot_error = velZCmd - velZ;

integratedAltitudeError += KiPosZ * z_error * dt;

float u_bar = kpPosZ * z_error + kpVelZ * z_dot_error + accelZCmd + integratedAltitudeError;
float c = (u_bar - CONST_GRAVITY) / R22;

thrust = mass * -c;
```

### Implement lateral position control in C++.

This part is implemented in [QuadControl::LateralPositionControl](./QuadControl.cpp#L234-L244):

```cpp
velCmd[0] = CONSTRAIN(velCmd[0], -maxSpeedXY,maxSpeedXY);
velCmd[1] = CONSTRAIN(velCmd[1], -maxSpeedXY,maxSpeedXY);

V3F accelCmd;
accelCmd.x = kpPosXY*(posCmd[0] - pos[0]) + kpVelXY*(velCmd[0] - vel[0]) + accelCmdFFV[0];
accelCmd.y = kpPosXY*(posCmd[1] - pos[1]) + kpVelXY*(velCmd[1] - vel[1]) + accelCmdFFV[1];

accelCmd.x = -CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
accelCmd.y = -CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);

accelCmd.z = 0;
```

### Implement yaw control in C++.

This part is implemented in [QuadControl::YawControl](./cpp/src/QuadControl.cpp#L265-L267):

```cpp
float yawError = yawCmd -yaw;
yawError = fmodf(yawError, F_PI*2.f);
yawRateCmd = kpYaw * yawError;
```

### Implement calculating the motor commands given commanded thrust and moments in C++.

This part is implemented in [QuadControl::GenerateMotorCommands](./QuadControl.cpp#L135-L158):

```cpp
float l_ = L / sqrtf(2.f);
float thrust0 = (collThrustCmd + momentCmd.x / l_ + momentCmd.y / l_ + momentCmd.z / kappa)/4.f;
float thrust1 = (collThrustCmd - momentCmd.x / l_ + momentCmd.y / l_ - momentCmd.z / kappa)/4.f;
float thrust3 = (collThrustCmd - momentCmd.x / l_ - momentCmd.y / l_ + momentCmd.z / kappa)/4.f;
float thrust2 = (collThrustCmd + momentCmd.x / l_ - momentCmd.y / l_ - momentCmd.z /kappa)/4.f;

cmd.desiredThrustsN[0] = CONSTRAIN(thrust0, minMotorThrust, maxMotorThrust);
cmd.desiredThrustsN[1] = CONSTRAIN(thrust1, minMotorThrust, maxMotorThrust);
cmd.desiredThrustsN[2] = CONSTRAIN(thrust2, minMotorThrust, maxMotorThrust);
cmd.desiredThrustsN[3] = CONSTRAIN(thrust3, minMotorThrust, maxMotorThrust);
```

## Flight Evaluation

### Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.

The implementation passes scenarios 1 to 5:

```
# Scenario 1
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
# Scenario 2
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
# Scenario 3
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds
# Scenario 4
PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
# Scenario 5
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
```
