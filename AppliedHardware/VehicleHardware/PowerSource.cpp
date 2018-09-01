#include "PowerSource.h"

using namespace AppliedHardware::VehicleHardware;
using namespace BaseHardware;

PowerSource::PowerSource(){
	battery = Battery::GetInstance();
}

float PowerSource::GetVoltage(){
	return battery->GetValue();
}
