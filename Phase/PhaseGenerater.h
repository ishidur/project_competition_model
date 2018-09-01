#ifndef PHASE_PHASE_GENERATER_H
#define PHASE_PHASE_GENERATER_H

#include "PhaseBase.h"
#include "PhaseInitialization.h"
#include "PhaseReady.h"
#include "PhaseNavigation.h"
#include "PhaseLookUpGate.h"
#include "PhaseGarage.h"
#include "PhaseSeesaw.h"

namespace Phase{
	class PhaseGenerater{
		private:
			int nowPhaseNum;
			PhaseBase** phaseList;

			char course;

		public:
			PhaseGenerater();
			PhaseBase* GetNextPhase();
			bool IsFinishAllPhase();

		private:
			int ReadCourse(const char* filename);
	};

}  // namespace Phase
#endif
