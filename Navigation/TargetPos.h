#ifndef NAVIGATION_TARGET_POS_H
#define NAVIGATION_TARGET_POS_H

#include "../Utilities/Vector2D.h"
#include "../Utilities/PIDCalculation.h"

using namespace Utilities;

typedef enum{
	ENUM_FORWARD_TURN_TERM_RHO,
	ENUM_FORWARD_TURN_TERM_TURN_BASE_OFFSET,
	ENUM_FORWARD_TURN_TERM_FORWARD_VALUE,
	ENUM_FORWARD_TURN_TERM_NUM
}ENUM_FORWARD_TURN_TERM;

namespace Navigation{
	class TargetPos{
		private:
			float turn;
			float turn_max;
			float length_max;
			float forward;
			float turn_base;
			float now_rho;
			float cross;
			float trun_theta;
			Vector2D vNearest;
			Vector2D veTarget;
			Vector2D v_now;
			Vector2D v_pre;
			Vector2D v_vel_now;
			Vector2D v_vel_pre;
			Vector2D tgt;
			float targetDistRatio;
			PIDCalculation* pIDCalculationLength;
			PIDCalculation* pIDCalculationCross;

		public:
			TargetPos();
			Vector2D GetTgtVe();
			Vector2D GetNowVe();
			void Set( Vector2D& _v, Vector2D& _ve, Vector2D& _v_now );
			void CalcTurn( Vector2D& _v_near, Vector2D& _v_e, Vector2D& _v_self_pos, float _rho );
			float CalcLinetraceTurn( float _length, float _rho );
			void CalcTurnBaseOffset( float _rho );
			float GetTurn();
			float GetValue( ENUM_FORWARD_TURN_TERM _term, float _rho );
			void CalcForward( float _rho );
			float CalcLinetraceForward( float _rho );
			float GetForward();
			Vector2D GetTgt();
			float GetTurnBase();
			float GetCross();
			void LoadTgtRatioParam();
	};
}  // namespace Navigation
#endif
