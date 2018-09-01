#include "ev3api.h"
#include "Clock.h"

#include "Timer.h"

using namespace BaseHardware;

Timer* Timer::singletonInstance = nullptr;

Timer::Timer(){
	cl = new ev3api::Clock();
	resetTime = GetValue();
}

Timer* Timer::GetInstance(){
	if (singletonInstance == nullptr){
		singletonInstance = new Timer();
	}
	return singletonInstance;
}

float Timer::GetValue(){
	return (float)(cl->now());
}

float Timer::Now(){
	return GetValue()-resetTime;
}

void Timer::Reset(){
	this->resetTime = GetValue();
}