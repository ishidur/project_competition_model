#ifndef BASEHARDWARE_BLUETOOTH_H
#define BASEHARDWARE_BLUETOOTH_H

#include "ev3api.h"
#include "BaseHardware/CommunicationProtocol.h"

namespace BaseHardware{

	class Bluetooth : public CommunicationProtocol{
		public:
			Bluetooth();
			void Connect() override;
			void Disconnect() override;
			bool IsConnect();
	};

}  // namespace BaseHardware
#endif
