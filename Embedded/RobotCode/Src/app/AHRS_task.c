#include "AHRS_task.h"
#include "AHRS.h"
#include <cmsis_os.h>
#include <string.h>

extern fp32 INS_gyro[3];
extern fp32 INS_accel[3];
extern fp32 INS_mag[3];

fp32 AHRS_quat[4];

void AHRSTaskInit(){
    memset(&AHRS_quat, 0, sizeof(AHRS_quat));
    AHRS_init(AHRS_quat, INS_gyro, INS_mag);
}

float AHRSRoll = 0;
float AHRSPitch = 0;
float AHRSYaw = 0;

void AHRSTaskUpdate(void *arguments){
    for(;;){
        AHRS_update(AHRS_quat, 0.001f, INS_gyro, INS_accel, INS_mag);
        AHRSRoll = get_roll(AHRS_quat);
        AHRSYaw = get_yaw(AHRS_quat);
        AHRSPitch = get_pitch(AHRS_quat);
    }

    vTaskDelete(NULL);
}
