#ifndef DRIVINGCONTROL_FIXED_TURN_H
#define DRIVINGCONTROL_FIXED_TURN_H

#include "../DrivingControl/CalcTurn.h"

namespace DrivingControl{

	class FixedTurn : public CalcTurn{
		private:
			float turn;

		public:
			void CalcTurnValue() override;
			float GetTurn();
			void SetTurn(float turn);
	};

}  // namespace DrivingControl
#endif
