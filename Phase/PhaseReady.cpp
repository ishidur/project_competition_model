#include "PhaseReady.h"
#include "../BaseHardware/Bluetooth.h"

using namespace Phase;
using namespace	DrivingControl;
using namespace AppliedHardware::Communication;
using namespace AppliedHardware::EnvironmentSensor;
using namespace BaseHardware;

PhaseReady::PhaseReady(){
	com = Communication::GetInstance();
	envViewer = EnvironmentViewer::GetInstance();
}

void PhaseReady::Execute(){
	printf("PhaseReady Execute\n");
	PoseDrivingControl poseDrivingControl;
	com->SetProtocol(new Bluetooth());
	com->Connect();

	poseDrivingControl.SetStop(false, true, true);

    tailControl.SetTargetAngle(85);
    while (true) {
        tailControl.RotateTowardTarget();
        if (IsGetStartCommand()) {
            break;
        }
        
        tslp_tsk(4);
    }
    com->Disconnect();

	finFlg = true;
	printf("PhaseReady Execute Done\n");
}

bool PhaseReady::IsGetStartCommand(){
    if(com->IsConnect()){
        unsigned char mes[255] = {0};
        com->GetReceiveMessage(mes);
        return envViewer->GetTouch() || mes[0]=='s';
    }else{
        return envViewer->GetTouch();
    }
}