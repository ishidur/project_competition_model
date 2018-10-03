#ifndef BASEHARDWARE_COMMUNICATION_PROTOCOL_H
#define BASEHARDWARE_COMMUNICATION_PROTOCOL_H

namespace BaseHardware{
	class CommunicationProtocol{
		protected:
			FILE* comHandle;
			bool isConnect;

		public:
			CommunicationProtocol():isConnect(false){}
			virtual void Connect()=0;
			unsigned char GetChar(){ return fgetc(this->comHandle); }
			void SendChar(unsigned char c){ fputc(c, this->comHandle); }
			void SendString(char* c){ fprintf(this->comHandle, "%s", c); }
			virtual void Disconnect()=0;
			bool IsConnect(){ return isConnect; }
	};

}  // namespace BaseHardware
#endif
