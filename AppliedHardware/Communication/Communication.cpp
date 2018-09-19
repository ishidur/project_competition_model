#include "string.h"
#include "ev3api.h"
#include "Communication.h"

using namespace AppliedHardware::Communication;

Communication* Communication::singletonInstance = nullptr;

Communication::Communication() : taskStop(false), count(0){
    memset(this->receivedBuffer, '\0', 255);
}

Communication* Communication::GetInstance(){
	if (singletonInstance == nullptr){
		singletonInstance = new Communication();
	}
	return singletonInstance;
}

void Communication::SetProtocol(BaseHardware::CommunicationProtocol* protocol){
	this->protocol = protocol;
}

void Communication::Connect(){
    if(!protocol->IsConnect()){
        protocol->Connect();
        act_tsk(COM_TASK);
        printf("Connect\n");
    }else{
        printf("Already connect!\n");
    }
}

void Communication::ReceiveTask(){
    if(!taskStop){
        uint8_t c = protocol->GetChar();
        this->receivedBuffer[count] = c;
        count = (count+1)%255;
        protocol->SendChar(c);
    }
}

void Communication::GetReceiveMessage(unsigned char* message){
    taskStop = true;
    unsigned char* cp = &(this->receivedBuffer[0]);
    while (*cp != '\0') *message++ = *cp++;
    memset(this->receivedBuffer, '\0', 255);
    count = 0;
    taskStop = false;    
}

void Communication::Disconnect(){
	if(protocol->IsConnect()){
		protocol->Disconnect();
		ter_tsk(COM_TASK);
		printf("Disconnect\n");
	}
}

bool Communication::IsConnect(){
    return protocol->IsConnect();
}

bool Communication::GetTaskStop(){
    return taskStop;
}

void Communication::SendString(char* mes){

}