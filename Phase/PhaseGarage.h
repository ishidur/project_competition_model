#ifndef PHASE_PHASE_GARAGE_H
#define PHASE_PHASE_GARAGE_H

#include "../Positioning/Localization/SelfPos.h"
#include "PhaseBase.h"

namespace Phase{
	class PhaseGarage : public PhaseBase{
        private:
            Positioning::Localization::SelfPos* pos;
            FILE* file;
		public:
			PhaseGarage();
			void Execute() override;
			bool IsFinish();
	};

}  // namespace Phase
#endif
