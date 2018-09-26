#include "PhaseExecuter.h"

using namespace Phase;

void PhaseExecuter::ExecutePhases(){
    while(!phaseGenerater.IsFinishAllPhase()){
        PhaseBase* phase = phaseGenerater.GetNextPhase();

        while(!phase->IsFinish()){
            phase->Execute();
        }
    }
}
