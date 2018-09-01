#include "ev3api.h"
#include "CalibrationGyro.h"
#include "../../BaseHardware/GyroSensor.h" 

using namespace AppliedHardware::VehicleHardware;
using namespace BaseHardware;

void CalibrationGyro::SetGyroOffset(){
    GyroSensor* gyroSensor = GyroSensor::GetInstance();

    float gMn = 1000.0, gMx = -100.0, gSum = 0.0;
    
    for (int i = 0; i < 200; ++i) {
        float gyro = gyroSensor->GetValue();
        gSum += gyro;

        if (gyro > gMx) gMx = gyro;
        if (gyro < gMn) gMn = gyro;

        tslp_tsk(4);
    }

    float offset = !(gMx - gMn < 2) ? (gSum / 200.0f) : 0;
    printf("Gyro offset %f\n", offset);
    gyroSensor->SetOffset(offset);
}