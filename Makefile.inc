APPL_COBJS += balancer.o balancer_param.o

APPL_CXXOBJS += Communication.o CalibrationLuminanceRange.o EnvironmentViewer.o CalibrationGyro.o DriveWheels.o PostureSensor.o PowerSource.o Tail.o Battery.o Bluetooth.o ColorSensor.o GyroSensor.o Motor.o Timer.o TouchSensor.o UltraSonicSensor.o CalcBalanceTurn2PWM.o CalcTurn2PWM.o FixedTurn.o InitVehicleHardware.o PoseDrivingControl.o TailControl.o TumbleStop.o TurnDrivingControl.o InitEnvironmentSensor.o LineLuminance.o Navigation.o Route.o TargetPos.o PhaseExecuter.o PhaseGarage.o PhaseGenerater.o PhaseInitialization.o PhaseLookUpGate.o PhaseNavigation.o PhaseReady.o PhaseSeesaw.o CalcSelfPosWithOdmetry.o SelfPos.o Circle.o Edge.o FieldMap.o Line.o Node.o VirtualMap.o ExponentialSmoothingFilter.o ParticleFilter.o PIDCalculation.o Vector2D.o

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif