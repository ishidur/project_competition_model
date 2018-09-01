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

unsigned char Bluetooth::GetChar(){
    return fgetc(this->comHandle);
}

void Bluetooth::SendChar(unsigned char c){
    fputc(c, this->comHandle);
}

void Bluetooth::Disconnect(){
    if(isConnect){
        fclose(this->comHandle);
        isConnect = false;
    }
}