#include "Bluetooth.h"

using namespace BaseHardware;

Bluetooth::Bluetooth(){
    this->comHandle = NULL;
}

void Bluetooth::Connect(){
    this->comHandle = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(this->comHandle != NULL);
    isConnect = true;
}

void Bluetooth::Disconnect(){
    if(isConnect){
        fclose(this->comHandle);
        isConnect = false;
    }
}

bool Bluetooth::IsConnect(){
    return ev3_bluetooth_is_connected();
}