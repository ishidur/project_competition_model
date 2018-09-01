#ifndef PHASE_PHASE_H
#define PHASE_PHASE_H

namespace Phase{

	class PhaseBase{
		protected:
			bool finFlg;

		public:
			PhaseBase() : finFlg(false) {};
			virtual void Execute()=0;
			bool IsFinish(){ return finFlg; };
	};

}  // namespace Phase
#endif
