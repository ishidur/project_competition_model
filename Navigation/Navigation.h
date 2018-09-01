#ifndef NAVIGATION_NAVIGATION_H
#define NAVIGATION_NAVIGATION_H

//#include "EnvironmentMeasurement/LineLuminance.h"
#include "Navigation/Route.h"
#include "Navigation/TargetPos.h"
#include "Positioning/Localization/SelfPos.h"

using namespace Positioning::Localization;

namespace Navigation{
	class Navigation{
		private:
		//	EnvironmentMeasurement::LineLuminance lineLuminance;
			Route* route;
			Positioning::Localization::SelfPos* selfPos;
			TargetPos* targetPos;
			int update_cnt;
		public:
			Navigation();
			void Start();
			void Stop();
			void Update();
			float GetTurn();
			void CalcTurn();
			bool IsFinish();

		};

}  // namespace Navigation
#endif
