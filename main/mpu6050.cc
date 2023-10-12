// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//		2023-03-10 - Fit to esp-idf v5
//		2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//		2016-04-18 - Eliminated a potential infinite loop
//		2013-05-08 - added seamless Fastwire support
//				   - added note about gyro calibration
//		2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//		2012-06-20 - improved FIFO overflow handling and simplified read process
//		2012-06-19 - completely rearranged DMP initialization code and simplification
//		2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//		2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//		2012-06-05 - add gravity-compensated initial reference frame acceleration output
//				   - add 3D math helper file to DMP6 example sketch
//				   - add Euler output and Yaw/Pitch/Roll output formats
//		2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//		2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//		2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "mpu6050.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "math.h"
	
static const char *TAG = "IMU";

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

//#include "MPU6050.h" // not necessary if using MotionApps include file
#include "MPU6050_6Axis_MotionApps20.h"

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;	// set true if DMP init was successful
uint8_t mpuIntStatus;	// holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;			// [w, x, y, z]			quaternion container
VectorInt16 aa;			// [x, y, z]			accel sensor measurements
VectorInt16 aaReal;		// [x, y, z]			gravity-free accel sensor measurements
VectorInt16 aaWorld;	// [x, y, z]			world-frame accel sensor measurements
VectorFloat gravity;	// [x, y, z]			gravity vector
float euler[3];			// [psi, theta, phi]	Euler angle container
float ypr[3];			// [yaw, pitch, roll]	yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// display quaternion values in easy matrix form: w x y z
void getQuaternion() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	printf("quat x:%6.2f y:%6.2f z:%6.2f w:%6.2f\n", q.x, q.y, q.z, q.w);
}

// display Euler angles in degrees
void getEuler() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetEuler(euler, &q);
	printf("euler psi:%6.2f theta:%6.2f phi:%6.2f\n", euler[0] * RAD_TO_DEG, euler[1] * RAD_TO_DEG, euler[2] * RAD_TO_DEG);
}

// display Euler angles in degrees
void getYawPitchRoll() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

	//printf("ypr roll:%3.1f pitch:%3.1f yaw:%3.1f\n",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
	//printf("%3.1f,%3.1f,%3.1f\n",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
	//ESP_LOGI(TAG, "roll:%f pitch:%f yaw:%f",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
}

// display real acceleration, adjusted to remove gravity
void getRealAccel() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	printf("areal x=%d y:%d z:%d\n", aaReal.x, aaReal.y, aaReal.z);
}

// display initial world-frame acceleration, adjusted to remove gravity
// and rotated based on known orientation from quaternion
void getWorldAccel() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
	printf("aworld x:%d y:%d z:%d\n", aaWorld.x, aaWorld.y, aaWorld.z);
}

void mpu6050_begin(){
	// i2c begin
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = GPIO_NUM_21 ;
	conf.scl_io_num = GPIO_NUM_22 ;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	conf.clk_flags = 0 ;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
  	ESP_LOGI(TAG, "I2C begin succeed !");


	// Initialize mpu6050
	mpu.initialize();

	ESP_LOGW(TAG, "MPU initialize");

	// Get Device ID
	uint8_t buffer[1];
	I2Cdev::readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, buffer);
	ESP_LOGI(TAG, "getDeviceID=0x%x", buffer[0]);

	// Initialize DMP
	devStatus = mpu.dmpInitialize();
	ESP_LOGI(TAG, "devStatus=%d", devStatus);
	if (devStatus != 0) {
		ESP_LOGE(TAG, "DMP Initialization failed [%d]", devStatus);
		while(1) {
			vTaskDelay(1);
		}
	}

	ESP_LOGW(TAG, "MPU setting offset");

	// This need to be setup individually
	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXAccelOffset(-790);
	mpu.setYAccelOffset(-1202);
	mpu.setZAccelOffset(796);
	mpu.setXGyroOffset(12);
	mpu.setYGyroOffset(-33);
	mpu.setZGyroOffset(38);

	ESP_LOGW(TAG, "MPU calibration time");

	// Calibration Time: generate offsets and calibrate our MPU6050
	mpu.CalibrateAccel(1);
	mpu.CalibrateGyro(1);
	mpu.setDMPEnabled(true);

	ESP_LOGI(TAG, "MPU init all succeed");
}

void mpu6050_read(Pose_t* pose){
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
		getYawPitchRoll();
		pose->roll = ypr[2] * RAD_TO_DEG ;
		pose->pitch = ypr[1] * RAD_TO_DEG ;
		pose->yaw = ypr[0] * RAD_TO_DEG ; 
	}
}


void task_mpu6050_update_past_buffer(void* pvParam){
	ESP_LOGI(TAG, "IMU Task Start");
	static int count = 0 ;
	
	Pose_t* pose_array = (Pose_t*) pvParam ;
	
	while(1){
		Pose_t curr_pose;
		mpu6050_read(&curr_pose);

		if(count < CONFIG_IMU_PAST_REFERENCE_BUFFER_SIZE){
			pose_array[count] = curr_pose; 
			count++;
		} else {
			for(int i = CONFIG_IMU_PAST_REFERENCE_BUFFER_SIZE - 2; i >= 0; i--){
				pose_array[i + 1] = pose_array[i]; 
			}
			pose_array[0] = curr_pose;
		}

		//printf("update:  %.2f, %.2f, %.2f\n", curr_pose.roll, curr_pose.pitch, curr_pose.yaw);
		//printf("IMU Task Updated\n");
		vTaskDelay(50/portTICK_PERIOD_MS);
	}
}
int mpu6050_judge_is_rotate(Pose_t* pose_array){
	float roll_change_rate = abs(pose_array[0].roll - pose_array[CONFIG_IMU_PAST_REFERENCE_BUFFER_SIZE-1].roll) ;
	float pitch_change_rate = abs(pose_array[0].pitch - pose_array[CONFIG_IMU_PAST_REFERENCE_BUFFER_SIZE-1].pitch) ;
	float yaw_change_rate = abs(pose_array[0].yaw - pose_array[CONFIG_IMU_PAST_REFERENCE_BUFFER_SIZE-1].yaw) ;
	
	if(yaw_change_rate > 20){
		return ROTATION;
	}else{
		return CLICK;
	}
}

void debug_show_past_data(Pose_t* pose_array){
	for(int i = 0 ; i < CONFIG_IMU_PAST_REFERENCE_BUFFER_SIZE ; i++){
		printf("Past rotation: %d : %.2f, %.2f, %.2f\n",i ,
		pose_array[i].roll, 
		pose_array[i].pitch, 
		pose_array[i].yaw);
	}
}
