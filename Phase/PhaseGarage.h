#ifndef PHASE_PHASE_GARAGE_H
#define PHASE_PHASE_GARAGE_H

#include "PhaseBase.h"

namespace Phase{
	class PhaseGarage : public PhaseBase{
		public:
			PhaseGarage();
			void Execute() override;
			bool IsFinish();
	};

}  // namespace Phase
#endif
