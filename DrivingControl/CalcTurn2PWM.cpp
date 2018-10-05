#include "CalcTurn2PWM.h"

using namespace DrivingControl;

CalcTurn2PWM::CalcTurn2PWM(){

}

void CalcTurn2PWM::CalcPWMValue(){}
void CalcTurn2PWM::CalcPWMValue(float power, float turn, signed char* leftPWM, signed char* rightPWM){
    turn = turn>100 ? 100:(turn<-100 ? -100:turn);

    float tmp_left = turn>=0 ? power : ((turn/100)*2.0+1.0)*power;
    *leftPWM  = tmp_left>100 ? 100 : (tmp_left<-100 ? -100 : (char)tmp_left);

    float tmp_right = turn<=0 ? power : (1.0-(turn/100)*2.0)*power;
    *rightPWM  = tmp_right>100 ? 100 : (tmp_right<-100 ? -100 : (char)tmp_right);
}
