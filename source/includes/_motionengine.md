# Motion Engine API
####################################

### setDownsample()
This function sets the streaming frequency divider. The streaming base frequency is 1KHz, and this function will set the frequency to 1000/n Hz. Currently, the default value of `n` is 20 and the acceptable values for `n` are integer multiples of 20, i.e., n = 20, 40, 60, etc. If n is set to another value, an exception will occur. 

### disableStreaming()
This function sends a command to Neblina to halt all incoming streaming packets.

### setFusionType()
This function sets the motion engine mode to either the 6-axis Inertial Measurement Unit (IMU) mode including a 3-axis accelerometer and a 3-axis gyroscope, or the 9-axis mode with an additional 3-axis magnetometer. The 9-axis mode is the default mode of operation.

### setAccelerometerRange()
This function sets the full-scale range of the 3-axis accelerometers. The possible modes are 2g, 4g, 8g, and 16g.

### setLockHeadingReference()
Neblina provides the option to lock magnetometer readings to represent the 0 degrees reference heading. When this function is called, Neblina will wait until the device is positioned on either a horizontal or vertical plane and then the current magnetometer readings will be locked to the zero degrees heading. Therefore, whenever the device points back to the same direction, Neblina will provide correction for the heading angle to reach the reference 0 degrees. The correction is adaptive meaning that if the deviation of the heading angle caused by the Gyroscopes' drift is high enough at the reference heading angle, the correction will be aggressive. However, if the heading info is close to the zero degrees (<15 degrees), the correction will apply smoothly. After setting the 0 degree reference, Neblina will pick up more reference heading angles using the gyroscope integration as the device moves, and will keep those additional points for further heading corrections. Currently, Neblina locks to a maximum of 4 reference heading angles.

### setExternalHeadingCorrection()
This command lets the host provide an external source of information to Neblina to correct its heading angle. This function should be called with two input arguments. The first argument is the the heading angle provided in degrees with one fractional decimal digit, while the second argument is the expected error for the heading angle, which is also provided in degrees with one fractional decimal digit. 

### streamEulerAngle()
This function enables/disables the streaming of the Euler angles in the so-called aerospace sequence, where the Yaw rotation (around z-axis) takes place first, which is then followed by Pitch (rotation around y-axis) and then Roll (rotation around x-axis). An input argument will determine the enable/disable mode. It is notable that in order to be able to pull Euler Angle data from Neblina, this function should be called first using the ENABLE mode.

The Euler angles are read in degrees and they have the following ranges: \\(Yaw \in [-180,180]\\), \\(Pitch \in [-90,90]\\), \\(Roll \in [-180,180]\\).

The angles are represented by one fractional digit precision using the following equation:

\\[angle_est = round(angle_ref \times 10)\\]

For instance, the angle \\(-104.731^{\circ}\\) is rounded to \\(-104.8^{\circ}\\), and is represented as the integer number -1048. 

### streamExternalForce()
This function enables/disables the streaming of the external force vector, i.e., the force vector minus gravity in the sensor frame. An input argument will determine the enable/disable mode. This function must be called with the ENABLE mode before pulling external force vector data out of Neblina.

### streamIMU()
This function enables/disables the streaming of the raw IMU data including accelerometers and gyroscopes. An input argument will determine the enable/disable mode. This function must be called with the ENABLE mode before pulling IMU data out of Neblina. While the accelerometer range can be configured, the gyro range is currently set to ±2000 dps. 

### streamMAG()
This function enables/disables the streaming of the raw magnetometer data. An input argument will determine the enable/disable mode. This function must be called with the ENABLE mode before pulling magnetometer data out of Neblina. The magnetometer data range is ±4 gauss.

### streamFingerGesture()
This function enables/disables the streaming of detected finger gestures including swipe left/right/up/down, flip left/right, or double tap. The gestures are functional, when Neblina is attached to a finger. An input argument will determine the enable/disable mode. This function must be called with the ENABLE mode before pulling finger gesture data out of Neblina.

### streamPedometer()
This function enables/disables the streaming of the pedometer data. An input argument will determine the enable/disable mode. This function must be called with the ENABLE mode before pulling pedometer data out of Neblina. The pedometer data includes step count, real-time cadence, and real-time heading (direction of walking) in degrees with one decimal fractional digit precision. The pedometer is configured to detect human steps, while walking or running. It is also applicable to cycling. The pedometer on Neblina is built based on the assumption that the device is attached or strapped to the front of the leg above the knee or all the way up to the top of the thigh. It is recommended that the device is attached closer to the knee rather than the top of the thigh for better accuracy. Furthermore, the device should be attached to the front of the leg, and not attached to the side or back of the leg. We can alternatively put Neblina in a front pocket and still track steps with a high accuracy.

### streamQuaternion()
This function enables/disables the streaming of the unit-length quaternion orientation using our computationally efficient and robust proprietary orientation filter. An input argument will determine the enable/disable mode. This function must be called with the ENABLE mode before pulling quaternion data out of Neblina. 

The unit-length quaternion contains 4 entries, i.e., \\(q = [q_1,q_2,q_3,q_4]\\), where \\(-1  ≤ q_{1:4} ≤ 1\\), and \\(q_{12} + q_{22} + q_{32} + q_{42} = 1\\).

The real numbers \\(q_{1:4}\\) are represented using a 16-bit fixed-point number format, where 15-bits are assigned to the fractional part along with a sign bit. Here is an example of how we calculate the 16-bit fixed-point representation of a real number \\(x=0.257812\\) in the range of \\([-1,1]\\):

\\[xfixp = round(x \times 215) = 8445\\]

The integer number \\(8445\\), which is represented by a 16-bit signed integer number, refers to the real-number \\(8445215 = 0.2577209\\), which obviously deviates from the actual reference number \\(x = 0.257812\\). The fixed-point representation error for the number \\(x = 0.257812\\) is \\(0.257812 - 0.2577209 = 0.0000911\\).

Using the above approach all real numbers \\(q_{1:4}\\) are encoded using a 16-bit fixed-point representation and 15 fractional bits.

### streamRotationInfo()
This function enables/disables the streaming of the real-time rotation information. If Neblina is attached to a wheel, the rotation information will include real-time cadence as well as total number of rotations. An input argument will determine the enable/disable mode. This function must be called with the ENABLE mode before pulling rotation information out of Neblina.

### streamSittingStanding()
This function enables/disables the streaming of the sitting/standing activity information. This functionality works best, if Neblina is attached to the top of the thigh (or is put in a front pocket). An input argument will determine the enable/disable mode. This function must be called with the ENABLE mode before pulling sitting/standing information out of Neblina. The information includes the real-time state of sitting or standing plus the total amount of sitting and standing time in seconds.

### streamMotionState()
This function enables/disables the streaming of the motion state, i.e., device in motion, or stationary. An input argument will determine the enable/disable mode. This function must be called with the ENABLE mode before pulling the motion state information out of Neblina.

### streamTrajectoryInfo()
This function enables/disables the streaming of the orientation trajectory information for Neblina. An input argument will determine the enable/disable mode. This function must be called with the ENABLE mode before pulling the trajectory information out of Neblina. The trajectory information includes pattern count, i.e., how many times a pre-recorded reference motion pattern has been repeated, as well as the real-time error (distance) in Yaw, Pitch, and Roll compared to the reference orientation trajectory. The trajectory information also includes how much of the track has been covered so far (0% to 100%). Whenever the full trajectory is covered, the pattern counter is increased by 1, and the progress percentage is reset to 0%. The reference motion pattern can be a workout exercise for example.

### trajectoryRecord()
This function is used to start/stop recording a reference motion pattern, i.e., orientation trajectory. An input argument will determine the start/stop mode. In the START mode, Neblina will start recording the reference orientation trajectory until either the devices comes to a complete stop, or the same function trajectoryRecord() is called by the host with the STOP mode. It is notable that due to memory limitations, the reference trajectory cannot be longer than 20 seconds. If we reach 20 seconds, the recording will stop automatically. When the recording stops, Neblina will be able to stream the real-time trajectory information, if the streaming has been already enabled by the calling the streamTrajectoryInfo() function. 

### getTrajectoryInfo()
This function returns the last trajectory information data pulled from Neblina. It returns correct data, if 1) A reference motion pattern has already been recorded by calling the trajectoryRecord() function, and 2) The streaming for trajectory information has already been enabled by calling the streamTrajectoryInfo() function. The trajectory data includes pattern count, i.e., how many times a pre-recorded reference motion pattern has been repeated, as well as the real-time error (distance) in Yaw, Pitch, and Roll compared to the reference orientation trajectory. 

### getEulerAngle()
This function returns the last values of Euler Angles pulled from Neblina. It returns correct data, if the corresponding streaming has been already enabled by calling the streamEulerAngle() function. The Euler angles are provided in Yaw, Pitch and Roll in degrees with one decimal fractional digit precision.

### getExternalForce()
This function returns the last value of the external force. It returns correct data, if the corresponding streaming has been already enabled by calling the streamExternalForce() function.

### getFingerGesture()
This function returns the last finger gesture pulled from Neblina. It returns correct data, if 1) The device is attached to a finger, and 2) The corresponding streaming has been already enabled by calling the streamFingerGesture() function.

### getIMU()
This function returns the last IMU data (accelerometer and gyroscope data) pulled from Neblina. It returns correct data, if the corresponding streaming has already been enabled by calling the streamIMU() function.

### getMAG()
This function returns the last MAG data (magnetometer and accelerometer data) pulled from Neblina. It returns correct data, if the corresponding streaming has already been enabled by calling the streamMAG() function.

### getMotionState()
This function returns the last motion state value (device in moion or stationary) pulled from Neblina. It returns correct data, if the corresponding streaming has already been enabled by calling the streamMotionState() function.

### getPedometer()
This function returns the last pedometer data pulled from Neblina. It returns correct data, if 1) The device is attached to the front of the thigh (or put in a front pocket), and 2) The corresponding streaming has been already enabled by calling the streamPedometer() function. Data includes real-time step count and heading direction as well as the total number of steps taken so far.

### getQuaternion()
This function returns the last quaternion data pulled from Neblina. It returns correct data, if the corresponding streaming has already been enabled by calling the streamQuaternion() function.

### getRotationInfo()
This function returns the last rotation info provided by Neblina. It returns the real-time cadence total number of rotation correctly, if 1) The device is attached to a rotating object, e.g., a wheel, and 2) The corresponding streaming has been already enabled by calling the streamRotationInfo() function.

### getSittingStanding()
This function returns the last sitting/standing data pulled from Neblina. It returns correct data, if the corresponding streaming has been already enabled by calling the streamSittingStanding() function. The sitting/standing data includes the real-time sitting/standing state as well as the total amount of sitting/standing time.

### Motion Engine Call Back Functions
Alternatively, developers can define API call-backs whenever a new motion feature has been updated using the following function pointers:

### Reset Timestamp()
This function forces the timestamp on Neblina to be reset to 0.


<!-- 
`typedef void (*Motion_CallBack)(motionstatus_t motion, uint32_t TimeStamp);`

`typedef void (*IMU_6Axis_CallBack)(IMU_6Axis_t data, uint32_t TimeStamp);`

`typedef void (*Quaternion_CallBack)(Quaternion_t quatrn, uint32_t TimeStamp);`

`typedef void (*EulerAngle_CallBack)(Euler_fxp_t angles, uint32_t TimeStamp);`

`typedef void (*ExternalForce_CallBack)(Fext_Vec16_t fext, uint32_t TimeStamp);`

`typedef void (*EulerAngleErr_CallBack)(Euler_fxp_t angles_err, uint32_t TimeStamp);`

`typedef void (*Pedometer_CallBack)(steps_t steps, int16_t direction, uint32_t TimeStamp);`

`typedef void (*MAG_CallBack)(AxesRaw_t data, uint32_t TimeStamp);`

> For example, one might define a single API call-back regarding pedometer as follows:

```c
void PedometerCallBackFunction(steps_t steps, int16_t direction, uint32_t TimeStamp)
{
//Everytime a new packet corresponding to pedometer data has arrived, this function is called with the appropriate input arguments
//write your code below...
	return;
}

//API call-backs for the motion engine configuration data
MotionEngine_CallBack_CFG_t g_MotionEngine_CallBackCfg = {
		NULL, //motion status
		NULL, //6-axis IMU data - accelerometer and gyroscope
		NULL, //Quaternion
		NULL, //Euler Angles
		NULL, //External Force
		NULL, //Euler Angle Error
		PedometerCallBackFunction, //Pedometer
		NULL, //Magnetometer data
		NULL, //Sitting/Standing Report
};
```


<!-- 
### Update Motion Features Main API Function
`
Host_RcvdPacket_UpdateMotionFeatures(uint8_t* buf, Motion_Feature_t* dev, MotionEngine_CallBack_CFG_t cfg)
`

> The motion features list has the following data structure:

```c
typedef struct Motion_Feature_t{ //all features
	uint8_t motion; //0: no change in motion, 1: stops moving, 2: starts moving
	IMURaw_t IMUData;
	Quaternion_t quatrn;
	Euler_fxp_t angles;
	Fext_Vec16_t force;
	Euler_fxp_t angles_err; //error in Euler angles compared to a reference trajectory
	uint16_t motiontrack_cntr; //shows how many times the pre-recorded track has been repeated
	uint8_t motiontrack_progress; //the percentage showing how much of a pre-recorded track has been covered
	uint32_t TimeStamp; //in microseconds
	steps_t steps;
	int16_t direction;
} Motion_Feature_t;
```

This is an important function that should be called every time a new BLE packet `buf` targeting the motion engine is received by the host.

This function will essentially update one or more features from the motion features list including motion status, 9-axis raw data, quaternion, Euler angles, external force, Euler angle errors, Pedometer, etc.

The `cfg` argument defines all the user-defined API call back functions for specific motion features.

 -->
