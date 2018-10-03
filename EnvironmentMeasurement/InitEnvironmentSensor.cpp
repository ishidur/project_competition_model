#include "ev3api.h"
#include "stdlib.h"
#include "string.h"

#include "InitEnvironmentSensor.h"

using namespace EnvironmentMeasurement;
using namespace AppliedHardware::EnvironmentSensor;

InitEnvironmentSensor::InitEnvironmentSensor(){
    environmentViewer = EnvironmentViewer::GetInstance();
}

void InitEnvironmentSensor::Initialize(){}

void InitEnvironmentSensor::Calibration(){
    int white=100, black=0;

    if(!LoadCalibrationLight(white, black)){
        SetColor(0);
        SetColor(1);
    }
    calibrationLuminanceRange.SetRange(white, black);
    while(environmentViewer->GetTouch());   // 離されるまで
}

void InitEnvironmentSensor::SetColor(int type){
    // float luminance = 0.0;

    while(environmentViewer->GetTouch());   // 離されるまで
    while(true) { // 押されるまで待機
        // luminance = environmentViewer->GetLuminance();

        // if(bt!=NULL){
        //     char mes[255] = {0};
        //     // bt->GetMessage(mes);
        //     if(environmentViewer->GetTouch() || mes[0]=='s') break;
        // }else{
            if(environmentViewer->GetTouch())break;
        // }

        tslp_tsk(4);
    }

    if(type==0){
        calibrationLuminanceRange.SetWhite();
    }else{
        calibrationLuminanceRange.SetBlack();
    }

    ev3_speaker_play_tone(NOTE_C4, 100);
    tslp_tsk(100);
    ev3_speaker_play_tone(NOTE_C4, 100);
}


bool InitEnvironmentSensor::LoadCalibrationLight(int &white, int &black){
    bool calib_flg = false;

    char param_name[255] = {'\0'};
    int param = 0;
    FILE* option_file = fopen("ev3rt/res/course/calibration.txt","r");
    assert(option_file != NULL);

    while(fscanf(option_file,"%s %d", &param_name[0], &param)!=EOF){
        printf("%s : %d\n", param_name, param);
        if(strcmp(param_name, "Option")==0){
            calib_flg = (bool)param;
            printf("Calibration: %s\n", calib_flg ? "True":"False");
            if(calib_flg==false) return false;
        }else if(strcmp(param_name, "White")==0){
            white = param;
            printf("Calib@White: %d\n", white);
        }else if(strcmp(param_name, "Black")==0){
            black = param;
            printf("Calib@Black: %d\n", black);
        }
    }
    fclose(option_file);

    return true;
}

