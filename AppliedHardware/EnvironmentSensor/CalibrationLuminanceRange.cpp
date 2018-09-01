#include "CalibrationLuminanceRange.h"

using namespace AppliedHardware::EnvironmentSensor;
using namespace BaseHardware;

CalibrationLuminanceRange::CalibrationLuminanceRange(){
    colorSensor = ColorSensor::GetInstance();
}

void CalibrationLuminanceRange::SetWhite(){
    white = colorSensor->GetValue();
}

void CalibrationLuminanceRange::SetBlack(){
    black = colorSensor->GetValue();
}

void CalibrationLuminanceRange::SetRange(){
    colorSensor->SetLuminanceRange(white, black);
}

void CalibrationLuminanceRange::SetRange(int white, int black){
    this->white = white;
    this->black = black;
    colorSensor->SetLuminanceRange(white, black);
}