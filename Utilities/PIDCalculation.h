#ifndef UTILITIES_P_I_D_CALCULATION_H
#define UTILITIES_P_I_D_CALCULATION_H

namespace Utilities{

	class PIDCalculation{
		private:
			float PParam;
			float IParam;
			float ISumParam;
			float DParam;
			float errorBefore;
			float errorISum;
			bool isSetErrorBefore;
		public:
			PIDCalculation();
			PIDCalculation(const char* filename);
			PIDCalculation( float _p, float _i, float _i_sum, float _d, float _err_before, float _err_i_sum );
			void PIDReStart();
			float GetPIDValue(float input, float target);

		private:
			int ReadPIDParam(const char* filename);
	};

}  // namespace Utilities
#endif
