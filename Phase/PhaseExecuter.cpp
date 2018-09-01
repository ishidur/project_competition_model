#include "PhaseExecuter.h"

using namespace Phase;

void PhaseExecuter::ExecutePhases(){
    printf("PhaseExecuter ExecutePhases\n");
    while(!phaseGenerater.IsFinishAllPhase()){
        printf("PhaseExecuter GetNextPhase\n");        
        PhaseBase* phase = phaseGenerater.GetNextPhase();
        printf("PhaseExecuter GetNextPhase done\n");    

        printf("PhaseExecuter phase Execute\n");      
        while(!phase->IsFinish()){
            phase->Execute();
        }
        printf("PhaseExecuter phase done\n");   
    }
    printf("PhaseExecuter ExecutePhases done\n");
}
