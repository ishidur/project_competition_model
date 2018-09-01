#ifndef UTILITIES_EXPONENTIAL_SMOOTHING_FILTER_H
#define UTILITIES_EXPONENTIAL_SMOOTHING_FILTER_H

#include "SmoothingFilter.h"

namespace Utilities{

	class ExponentialSmoothingFilter : public SmoothingFilter{
		private:
			float AParam;
			float valueBefore;

		public:
			ExponentialSmoothingFilter(float AParam);
			ExponentialSmoothingFilter(const char* filename);
			float GetValue(float input) override;

		private:
			int ReadAParam(const char* filename);
	};

}  // namespace Utilities
#endif
