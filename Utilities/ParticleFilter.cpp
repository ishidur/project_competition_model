#include <math.h>
#include <stdlib.h>
#include "ParticleFilter.h"

using namespace Utilities;
using namespace Positioning::MapInfomation;
using namespace AppliedHardware::VehicleHardware;

ParticleFilter::ParticleFilter(FieldMap mapLikelihood){
	this->mapLikelihood = mapLikelihood;
}

float ParticleFilter::GetValue(){
	return 0;
}

void ParticleFilter::Initialize(Vector2D pos, float theta){
    for(int k=0;k<NP;k++){
        px[k]=pos.x;
        py[k]=pos.y;
        ptheta[k]=theta;
        pw[k]=0.0+1.0/NP;
    }
}

void ParticleFilter::Localization(Vector2D* posEst, float* thetaEst, Vector2D posDiff, float thetaDiff, float input){
	for(int ip=0; ip<NP; ip++){
		float x = px[ip];
		float y = py[ip];
		float theta = ptheta[ip];
		float w = pw[ip];
		
		x += cos(*thetaEst)*posDiff.x + RandNormal(0.0, QX);
		if (x<0) x=0; else if (x>mapLikelihood.GetWidth()-1) x=mapLikelihood.GetWidth()-1;

		y += sin(*thetaEst)*posDiff.y+ RandNormal(0.0, QY);
		if (y<0) y=0; else if (y>mapLikelihood.GetHeight()-1) y=mapLikelihood.GetHeight()-1;

		theta = *thetaEst + thetaDiff + RandNormal(0.0, QTHETA);

		float g = Gauss(input/WHITE,(float)(mapLikelihood.GetMapValue((int)x,(int)y))/WHITE, RR); // calc likelihood
		w *= g;

		px[ip] = x;
		py[ip] = y;
		ptheta[ip] = theta;
		pw[ip] = w;
	}

	// Normalize
	float sum=0.0;
	for(int i=0;i<NP;i++) sum += pw[i];

	if(sum!=0.0){
		for(int i=0;i<NP;i++) pw[i] /= sum;
	}else{
		for(int i=0;i<NP;i++) pw[i] = 1.0/NP;
	}

	// Resampling: Low Variance Sampling, Roulette Sampling
	Resampling(px,py,ptheta,pw);
	posEst->x=0.0;
	posEst->y=0.0;
	*thetaEst=0.0;
	for(int i=0;i<NP;i++){
		posEst->x += px[i]*pw[i];
		posEst->y += py[i]*pw[i];
		*thetaEst += ptheta[i]*pw[i];
	}
}

//////////////////////////////////////////////////////////////////

float Resampling(float* xx,float* yy, float* tt, float* ww){
   float Neff=0.0;
   float wcum[NP]={0.0};
   float base[NP];
   float ppx[NP],ppy[NP],ppt[NP];
   float resampleID[NP];
   
   for(int i=0;i<NP;i++){
        Neff=Neff+*(ww+i)*(*(ww+i));
   }
   Neff=1.0/Neff;
   
   if(Neff<NTH){
        wcum[0]=ww[0]; // cumsum
        for(int i=1;i<NP;i++){
            wcum[i]=wcum[i-1]+*(ww+i);
        }

        for(int i=0;i<NP;i++){
            base[i]=(float)i/NP;
            ppx[i]=*(xx+i);
            ppy[i]=*(yy+i);
            ppt[i]=*(tt+i);
            //resampleID[i]=base[i]+(double)rand()/RAND_MAX/NP;
            resampleID[i]=base[i]+(double)Uniform()/NP;
        }

        int ind=0;
        for(int ip=0;ip<NP;ip++){
            while(resampleID[ip]>wcum[ind]){
                ind++;
            }

            *(xx+ip)=ppx[ind];
            *(yy+ip)=ppy[ind];
            *(tt+ip)=ppt[ind];
            *(ww+ip)=1.0/NP;
        }    
   }
   return 0;
}


float Gauss(float x,float u,float sigma){ // ガウス分布の確率密度を計算する関数
    return 1.0/sqrt(2.0*M_PI*sigma*sigma)*exp(-(x-u)*(x-u)/(2.0*sigma*sigma));
}

double Uniform( void ){
    return ((double)rand()+1.0)/((double)RAND_MAX+2.0);
}

double RandNormal( double mu, double sigma ){
    double z = sqrt(-2.0*log(Uniform())) * sin(2.0*M_PI*Uniform());
    return mu + sigma*z;
}

