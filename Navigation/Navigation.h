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
			Vector2D near;
		public:
			Navigation();
			void Start();
			void Stop();
			void Update();
			float GetTurn();
			float GetEndNodeTurn();
			void CalcTurn();
			float GetForward();
			void CalcForward();
			bool IsNearGoalSpeedUp();
			bool IsArriveGoalSpeedDown();
			bool IsAfterGoalSpeedDown();
			bool IsEndNode();
			bool IsFinish();
			float GetDbg();
			Vector2D GetDbgV();
			Vector2D GetNode();
			Vector2D GetDir();
			float GetRho();
			Vector2D GetNear();
			Vector2D GetTgt();
			float GetTurnBase();
			float GetCross();
		};

}  // namespace Navigation
#endif



