#ifndef UTILITIES_SMOOTHING_FILTER_H
#define UTILITIES_SMOOTHING_FILTER_H

namespace Utilities{
	
	class SmoothingFilter{
		public:
			virtual float GetValue(float input)=0;
	};

}  // namespace Utilities
#endif
