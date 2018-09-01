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
	PoseDrivingControl poseDrivingControl;
	com->SetProtocol(new Bluetooth());
	com->Connect();

	poseDrivingControl.SetStop(true, true, true);

    while (true) {
        if (IsGetStartCommand()) {
            break;
        }
        
        tslp_tsk(4);
    }
    com->Disconnect();

	finFlg = true;
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