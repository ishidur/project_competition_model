#include "PhaseGarage.h"
#include <stdio.h>

using namespace Phase;

PhaseGarage::PhaseGarage(){
}

void PhaseGarage::Execute(){
	printf("PhaseGarage Execute done\n");
    
	finFlg = true;
	printf("PhaseGarage Execute done\n");
}

bool PhaseGarage::IsFinish(){
	return false;
}