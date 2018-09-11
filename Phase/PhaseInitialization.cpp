#include "PhaseInitialization.h"

using namespace Phase;

PhaseInitialization::PhaseInitialization(){}

void PhaseInitialization::Execute(){
	printf("PhaseInitialization Execute\n");
	Calibration();

	finFlg = true;
	
    ev3_speaker_play_tone(NOTE_C4, 100);
    tslp_tsk(100);
    ev3_speaker_play_tone(NOTE_C4, 100);
	printf("PhaseInitialization Execute done\n");
}

void PhaseInitialization::Calibration(){
	printf("PhaseInitialization initVehicleHardware Initialize\n");
	initVehicleHardware.Initialize();
	printf("PhaseInitialization initEnvironmentSensor Initialize\n");
	initEnvironmentSensor.Initialize();

	printf("PhaseInitialization initVehicleHardware Calibration\n");
	// initVehicleHardware.Calibration();
	printf("PhaseInitialization initEnvironmentSensor Calibration\n");
	initEnvironmentSensor.Calibration();
}