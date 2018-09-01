#include "CalcTurn2PWM.h"

using namespace DrivingControl;

CalcTurn2PWM::CalcTurn2PWM(){

}

void CalcTurn2PWM::CalcPWMValue(){}
void CalcTurn2PWM::CalcPWMValue(float power, float turn, signed char* leftPWM, signed char* rightPWM){
    turn = turn>100 ? 100:(turn<-100 ? -100:turn);
    *leftPWM  = turn>=0 ? power : ((turn/100)*2.0+1.0)*power;
    *rightPWM = turn<=0 ? power : (1.0-(turn/100)*2.0)*power;
}
