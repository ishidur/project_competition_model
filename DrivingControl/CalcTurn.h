#ifndef DRIVINGCONTROL_CALC_TURNS_H
#define DRIVINGCONTROL_CALC_TURNS_H

namespace DrivingControl{
	class CalcTurn{
		private:
			float turn;

		public:
			virtual void CalcTurnValue()=0;
			virtual float GetTurn()=0;
	};

}  // namespace DrivingControl
#endif
