#include "ExponentialSmoothingFilter.h"
#include <stdio.h>
#include <stdlib.h>

using namespace Utilities;

ExponentialSmoothingFilter::ExponentialSmoothingFilter(float AParam){
    this->AParam = AParam;
}

ExponentialSmoothingFilter::ExponentialSmoothingFilter(const char* filename){
    ReadAParam(filename);
}

float ExponentialSmoothingFilter::GetValue(float input){
    float value = this->AParam*(this->valueBefore - input) + input;
    this->valueBefore = value;
	return value;
}

int ExponentialSmoothingFilter::ReadAParam(const char* filename){
    FILE* option_file = fopen(filename,"r");
    if(option_file==NULL) return 0;
    
    char param_name[255] = {'\0'};
    fscanf(option_file,"%s %f", &param_name[0], &AParam);
    
    fclose(option_file);
    return 1;
}