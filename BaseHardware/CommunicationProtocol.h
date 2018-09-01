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
			virtual unsigned char GetChar()=0;
			virtual void SendChar(unsigned char c)=0;
			virtual void Disconnect()=0;
			bool IsConnect(){ return isConnect; }
	};

}  // namespace BaseHardware
#endif
