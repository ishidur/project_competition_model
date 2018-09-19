#ifndef POSITIONING_LOCALIZATION_SELF_POS_H
#define POSITIONING_LOCALIZATION_SELF_POS_H

#include "Positioning/MapInfomation/FieldMap.h"
#include "Utilities/Vector2D.h"
#include "CalcSelfPosWithOdmetry.h"

using namespace Utilities;

namespace Positioning{
	namespace Localization{
		class SelfPos{
			private:
				static SelfPos* singletonInstance;

				Vector2D vSelf;
				float thetaSelf;

				Vector2D vSelfVel;
				Vector2D vSelfInit;

				Positioning::MapInfomation::FieldMap fieldMap;
				Positioning::Localization::CalcSelfPosWithOdmetry* Odmetry;

			public:
				SelfPos();
				static SelfPos* GetInstance();

				void Start();
				void UpdateSelfPosComp( Vector2D& _vSelf, Vector2D& _vSelfVel );
				void UpdateSelfPos();
				Vector2D GetSelfPos();
				float GetTheta();
				Vector2D GetSelfVelocity();
				Vector2D GetMean();
				float GetPhi();
				private:
					int ReadFirstPos(const char* course_filename, const char* pos_filename);

		};
	}  // namespace Localization
}  // namespace Positioning
#endif
