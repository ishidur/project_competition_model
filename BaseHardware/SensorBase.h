#ifndef BASEHARDWARE_SENSOR_BASE_H
#define BASEHARDWARE_SENSOR_BASE_H

namespace BaseHardware{
	
	class SensorBase{
		public:
			virtual float GetValue()=0;
	};

}  // namespace BaseHardware
#endif
