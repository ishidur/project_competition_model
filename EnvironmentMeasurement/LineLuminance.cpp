#include "LineLuminance.h"

using namespace EnvironmentMeasurement;
using namespace AppliedHardware::EnvironmentSensor;
using namespace Utilities;

LineLuminance::LineLuminance(){
    environmentViewer = EnvironmentViewer::GetInstance();
    pidCalc = new PIDCalculation("/ev3rt/res/luminance/pid_params.txt");
    filter = new ExponentialSmoothingFilter(0.0);
}

void LineLuminance::CalcTurnValue(){
    float luminance = filter->GetValue(environmentViewer->GetLuminance());
    turn = -1.0*pidCalc->GetPIDValue(luminance, 50);
}

void LineLuminance::CalcTurnValueByRGB(){
    int r, g, b;
    environmentViewer->GetRGB(&r, &g, &b);
    float col = (filter->GetValue(g)-0.0)/(15.0-0.0) * 100.0;
    //    float col = (filter->GetValue(g)-10.0)/(28.0-10.0) * 100.0;
    turn = pidCalc->GetPIDValue(col, 50);
}

float LineLuminance::GetTurn(){
    return turn;
}
