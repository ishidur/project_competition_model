#include "ev3api.h"
#include "PhaseGenerater.h"

using namespace Phase;
PhaseGenerater::PhaseGenerater(): nowPhaseNum(0) {
	const char* filename = "/ev3rt/res/course/course.txt";
	if(!ReadCourse(filename)){
		printf("no file %s!\n", filename);
		ter_tsk(MAIN_TASK);
	}
	printf("get course: %c\n", course);   

	if(course=='R'){
		phaseList = new PhaseBase*[5]{
			new PhaseInitialization(), 
			new PhaseReady(),
			new PhaseNavigation(),
			new PhaseSeesaw(),
			nullptr
		};
		printf("phaseList(R)\n");   
	}else if(course=='L'){
		phaseList = new PhaseBase*[6]{
			new PhaseInitialization(), 
			new PhaseReady(),
			new PhaseNavigation(),
			new PhaseLookUpGate(),
			new PhaseGarage(),
			nullptr
		};
		printf("phaseList(L)\n");  
	}else{
		phaseList = new PhaseBase*[4]{
			new PhaseInitialization(), 
			new PhaseReady(),
			new PhaseNavigation(),
			nullptr
		};
		printf("no phase: %c\n",course);  
	}
}

PhaseBase* PhaseGenerater::GetNextPhase(){
	PhaseBase* p = phaseList[nowPhaseNum];
	if(p!=nullptr) nowPhaseNum++; 
	return p;
}

bool PhaseGenerater::IsFinishAllPhase(){
	return phaseList[nowPhaseNum]==nullptr;
}

int PhaseGenerater::ReadCourse(const char* filename){
	FILE* option_file = fopen(filename,"r");
	if(option_file==NULL) return 0;
	
	char param_name[255] = {'\0'};
	fscanf(option_file,"%s %c", &param_name[0], &course);
	
	fclose(option_file);
	return 1;
}