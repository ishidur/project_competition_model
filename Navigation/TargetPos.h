#ifndef NAVIGATION_TARGET_POS_H
#define NAVIGATION_TARGET_POS_H

#include "../Utilities/Vector2D.h"
#include "../Utilities/PIDCalculation.h"

using namespace Utilities;

namespace Navigation{
	class TargetPos{
		private:
			float turn;
			float turn_max;
			float length_max;

			Vector2D vNearest;
			Vector2D veTarget;
			Vector2D v_now;
			Vector2D v_pre;

			float targetDistRatio;
			PIDCalculation* pIDCalculation;

		public:
			TargetPos();
			Vector2D GetTgtVe();
			Vector2D GetNowVe();
			void Set( Vector2D& _v, Vector2D& _ve, Vector2D& _v_now );
			void CalcTurn( Vector2D& _v_near, Vector2D& _v_e, Vector2D& _v_self_pos );
			float CalcLinetraceTurn( float _length );
			float GetTurn();
			void LoadTgtRatioParam();
	};
}  // namespace Navigation
#endif
