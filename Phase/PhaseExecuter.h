#ifndef PHASE_PHASE_EXECUTER_H
#define PHASE_PHASE_EXECUTER_H

#include "PhaseGenerater.h"

namespace Phase{
	
	class PhaseExecuter{
		private:
			PhaseGenerater phaseGenerater;

		public:
			void ExecutePhases();
	};

}  // namespace Phase
#endif
