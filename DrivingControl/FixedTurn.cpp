#include "FixedTurn.h"

using namespace DrivingControl;

void FixedTurn::CalcTurnValue(){
}

float FixedTurn::GetTurn(){
	return turn;
}

void FixedTurn::SetTurn(float turn){
	this->turn = turn;
}
