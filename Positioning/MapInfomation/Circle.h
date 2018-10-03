#ifndef POSITIONING_MAPINFOMATION_CIRCLE_H
#define POSITIONING_MAPINFOMATION_CIRCLE_H

#include "Positioning/MapInfomation/Edge.h"
#include "../../Utilities/Vector2D.h"

using namespace Utilities;

namespace Positioning{
	namespace MapInfomation{
		class Circle : public Edge{
			private:
				Vector2D posCenter;	//�~�̒��S
				float r;			//���a
				float direction;	//��]������(1 or -1�Ƃ���)
				float rho;			//�ȗ�

				Vector2D ve;		//�i�s�����P�ʃx�N�g��

			public:
				Circle( Vector2D& _vc, float _r, float _direction, float _rho );
				float GetNearestPosDistance();
				Vector2D GetNearestPos( Vector2D& _v1 );
				Vector2D GetVe();
				float GetRho();

		};
	}  // namespace MapInfomation
}  // namespace Positioning
#endif
