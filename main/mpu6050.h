#ifndef _MPU_H_
#define _MPU_H_

typedef struct  {
    float pitch;
    float roll;
    float yaw ;
} Pose_t;

#define ROTATION 0
#define CLICK 1
#define UNKNOWN 2

void mpu6050_begin(); // I2C and mpu settup
void mpu6050_read(Pose_t* pose); // updata pose_t once

void task_mpu6050_update_past_buffer(void* pvParam); // updata buffer array in background
int mpu6050_judge_is_rotate(Pose_t* pose_array); // get whether past buffer reach the change rate, return boolean
void debug_show_past_data(Pose_t* pose_array);

#endif 