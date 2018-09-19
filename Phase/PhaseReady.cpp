#include "PhaseReady.h"
#include <string.h>
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
    printf("BT Connext:%d\n", com->IsConnect());
    if(com->IsConnect()){
        unsigned char mes[255] = {};
        memset(mes,'\0',255);
        com->GetReceiveMessage(mes);
        if(mes[0]!='\0') printf("BT:%s\n",mes);
        return envViewer->GetTouch() || mes[0]=='s';
    }else{
        return envViewer->GetTouch();
    }
}