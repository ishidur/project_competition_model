#ifndef POSITIONING_LOCALIZATION_CALC_SELF_POS_WITH_ODMETRY_H
#define POSITIONING_LOCALIZATION_CALC_SELF_POS_WITH_ODMETRY_H

#include "../../Utilities/Vector2D.h"
#include "../../AppliedHardware/VehicleHardware/DriveWheels.h"

using namespace AppliedHardware::VehicleHardware;
using namespace Utilities;

namespace Positioning{
	namespace Localization{
		class CalcSelfPosWithOdmetry{
			private:
				float rightAngle_bf;
				float leftAngle_bf;
				float theta;
				float thetaInit;
				Vector2D vSelfInit;
				Vector2D v_mean;
				DriveWheels* wheels;
				int mean_cnt;
				bool is_mean;
				float phi;
				float CalcTheta( Vector2D& _vSelfVel );
			public:
				CalcSelfPosWithOdmetry();
				void Start( Vector2D& _vSelfInit,float _theta_init);
				Vector2D DeadReckoningWithOdmetry( Vector2D& _vSelf, Vector2D& _vSelfVel );
				float GetTheta();
				Vector2D GetMean();
				float GetPhi();
		};
	}  // namespace Localization
}  // namespace Positioning
#endif
