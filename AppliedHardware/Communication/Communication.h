#ifndef APPLIEDHARDWARE_COMMUNICATION_COMMUNICATION_H
#define APPLIEDHARDWARE_COMMUNICATION_COMMUNICATION_H

#include "../../BaseHardware/CommunicationProtocol.h"

namespace AppliedHardware{
	namespace Communication{

		class Communication{
			private:
				static Communication* singletonInstance;

				BaseHardware::CommunicationProtocol* protocol;
				unsigned char receivedBuffer[255];

				bool taskStop;
				int count;

			public:
				Communication();
				static Communication* GetInstance();

				void SetProtocol(BaseHardware::CommunicationProtocol* protocol);
				void Connect();
				void ReceiveTask();
				void GetReceiveMessage(unsigned char* message);
				void Disconnect();
				bool IsConnect();
				bool GetTaskStop();
				void SendString(char* mes);
		};

	}  // namespace Communication
}  // namespace AppliedHardware
#endif
