#ifndef APPLIEDHARDWARE_VEHICLEHARDWARE_POWER_SOURCE_H
#define APPLIEDHARDWARE_VEHICLEHARDWARE_POWER_SOURCE_H

#include "../../BaseHardware/Battery.h"

namespace AppliedHardware{
	namespace VehicleHardware{

		class PowerSource{
			private:
				BaseHardware::Battery* battery;

			public:
				PowerSource();
				float GetVoltage();
		};

	}  // namespace VehicleHardware
}  // namespace AppliedHardware
#endif
