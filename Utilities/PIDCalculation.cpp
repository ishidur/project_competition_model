#include "PIDCalculation.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace Utilities;

PIDCalculation::PIDCalculation() {
}

PIDCalculation::PIDCalculation(const char* filename): errorBefore(0.0),errorISum(0.0) {
    ReadPIDParam(filename);
}

PIDCalculation::PIDCalculation( float _p, float _i, float _i_sum, float _d, float _err_befor, float _err_i_sum ):PParam(_p),IParam(_i),ISumParam(_i_sum),DParam(_d),errorBefore(_err_befor),errorISum(_err_i_sum) {
}

float PIDCalculation::GetPIDValue(float input, float target) {
    float error = target - input;
    this->errorISum = error + this->errorISum * this->ISumParam;

    float value = this->PParam * error + this->IParam * this->errorISum + this->DParam * (error - this->errorBefore);
    this->errorBefore = error;

    return value;
}

int PIDCalculation::ReadPIDParam(const char* filename) {
    FILE* pid_param_file = fopen(filename,"r");
    if(pid_param_file==NULL) return 0;

    char param_name[255] = {'\0'};
    float param;

    while(fscanf(pid_param_file,"%s %f", &param_name[0], &param)!=EOF){
        if(!strcmp(param_name,"P:")){
                this->PParam = param;
        }else if(!strcmp(param_name,"I:")){
                this->IParam = param;
        }else if(!strcmp(param_name,"I_sum:")){
                this->ISumParam = param;
        }else if(!strcmp(param_name,"D:")){
                this->DParam = param;
        }
    }
    printf("load pid params\n");
    printf("P: %f\n", this->PParam);
    printf("I: %f\n", this->IParam);
    printf("I_sum: %f\n", this->ISumParam);
    printf("D: %f\n", this->DParam);

    fclose(pid_param_file);
    return 1;
}
