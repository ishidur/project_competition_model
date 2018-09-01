#ifndef PHASE_PHASE_LOOK_UP_GATE_H
#define PHASE_PHASE_LOOK_UP_GATE_H

#include "../Positioning/Localization/SelfPos.h"
#include "PhaseBase.h"

namespace Phase{

	class PhaseLookUpGate : public PhaseBase{
		private:
			Positioning::Localization::SelfPos* pos; 

		public:
			PhaseLookUpGate();
			void Execute() override;
			bool IsFinish();
	};

}  // namespace Phase
#endif
