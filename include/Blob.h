#ifndef __BLOB__
#define __BLOB__
#include <cv.h>
#include <highgui.h>
//#include <Mosaic.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <opencv2/contrib/contrib.hpp>
#include <time.h>

using namespace std;
using namespace cv;

//Parámetros pasa la erosión
int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;

class Descriptor
{
public:
    int area;
    int label;
    vector<Point2i> vCoordenadas;
    Point2f centroid;
    Point2i maxPix;//Coordenada del pixel más alto del blob
    Point2i minPix;
    Mat Cov;
    Mat eval;
    Mat evec;
    int ejeMayor;
    Descriptor(int *etiqueta, int *area1);
    void findMaxMin();
};

Descriptor::Descriptor(int *etiqueta, int *area1)
{
  area=*area1;
  label=*etiqueta;
  centroid=Point2f(0., 0.);
  Cov=Mat::zeros(Size(2,2), CV_32FC1);
  ejeMayor=0;
}

void Descriptor::findMaxMin()
{
   maxPix=vCoordenadas[0];
   minPix=vCoordenadas[0];
  for(int i=1; i<vCoordenadas.size(); i++)
  {
    if(vCoordenadas[i].y>maxPix.y)
    {
      maxPix=vCoordenadas[i];
    }
    if(vCoordenadas[i].y<minPix.y)
    {
      minPix=vCoordenadas[i];
    }
  }

}

void computeParamsBlob(Mat& labels, vector<int>& vAreaEtiquetas, vector<bool>& vLabelFlags,vector<Descriptor*>& vDescriptores, vector<int>& vMappingDes)
{
  /*if(vDescriptores.size()!=0)
  {
    cout<<"Tiene "<<vDescriptores.size()<<" descriptores"<<endl;
    //vMappingDes[1]-1
    vDescriptores[0]->vCoordenadas.push_back(Point(65, 12));
    cout<<"label "<<vDescriptores[0]->label<<" area: "<<vDescriptores[0]->area<<endl<<"Coordenadas: "<<
    vDescriptores[0]->vCoordenadas.back()<<endl;
  }*/

int idx;
  for(int i=0; i<labels.rows; i++)
  {
    for(int j=0; j<labels.cols; j++)
    {
      if(vLabelFlags[labels.at<int>(i,j)]==true)
      {
        idx=vMappingDes[labels.at<int>(i,j)]-1;
          vDescriptores[idx]->vCoordenadas.push_back(Point(j, i));
          vDescriptores[idx]->centroid+=Point2f(vDescriptores[idx]->vCoordenadas.back().x, vDescriptores[idx]->vCoordenadas.back().y);
          //vDescriptores[idx]->centroid+=Point2f(vDescriptores[idx]->vCoordenadas.back());
      }
    }
  }

  float a, b;
  //Calcula el centroide de cada blob y su matriz de covarianza
  for(int nBlob=0; nBlob<vDescriptores.size(); nBlob++)
  {
    vDescriptores[nBlob]->centroid.x/=vDescriptores[nBlob]->vCoordenadas.size();
    vDescriptores[nBlob]->centroid.y/=vDescriptores[nBlob]->vCoordenadas.size();
    for(int pt=0; pt<vDescriptores[nBlob]->vCoordenadas.size(); pt++)
    {
      a=vDescriptores[nBlob]->vCoordenadas[pt].x-vDescriptores[nBlob]->centroid.x;
      b=vDescriptores[nBlob]->vCoordenadas[pt].y-vDescriptores[nBlob]->centroid.y;
      vDescriptores[nBlob]->Cov+=(Mat_<float>(2, 2)<<a*a, a*b, a*b, b*b);
    }
    vDescriptores[nBlob]->Cov/=(vDescriptores[nBlob]->vCoordenadas.size()-1);
    eigen (vDescriptores[nBlob]->Cov, vDescriptores[nBlob]->eval, vDescriptores[nBlob]->evec);
    vDescriptores[nBlob]->ejeMayor=sqrt(vDescriptores[0]->eval.at<float>(0,0));
  }
}

void Erosion( Mat& src, Mat & erosion_dst)
{
  int erosion_type=MORPH_RECT;
  erosion_size=2;
  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

  /// Apply the erosion operation
  dilate( src, erosion_dst, element );
  //imshow( "Erosion Demo", erosion_dst );
  	
}
int maxBlob(vector<Descriptor*>& vDescriptores)
{
  float maxTemp=vDescriptores[0]->area;
  int maxId=0;
  for(int i=1; i<vDescriptores.size(); i++)
  {
    if(vDescriptores[i]->area >maxTemp)
    {
        maxTemp=vDescriptores[i]->area;
        maxId=i;
    }
  }
 return maxId; 
}
 
#endif



/*
Point2f v0, v1, e[4], e1;
float s0, s1, fact=1., angle, angleRads, cOpuesto, cAdya, hipotenusa,
angle2, angleRads2, cOpuesto2, cAdya2, hipotenusa2;
if(vDescriptores.size()>0)
{

    v0=Point2f(vDescriptores[0]->evec.at<float>(0,0), vDescriptores[0]->evec.at<float>(0,1));
    v1=Point2f(vDescriptores[0]->evec.at<float>(1,0), vDescriptores[0]->evec.at<float>(1, 1));
    s0=fact*sqrt(vDescriptores[0]->eval.at<float>(0,0));
    s1=fact*sqrt(vDescriptores[0]->eval.at<float>(0,1));
    //angle=atan2(v0.y, v0.x)*180/M_PI;
    angleRads=atan2(v0.y, v0.x);
    angle = (angleRads*180/M_PI);
    hipotenusa=s0/2;
    cOpuesto=sin(angleRads)*hipotenusa;
    cAdya=sqrt(hipotenusa*hipotenusa-(cOpuesto*cOpuesto));
    cout<<"Ángulo del rectángulo: "<<angle<<endl<<
    "Width = "<<s0<<" height = "<<s1<<endl<<"hipotenusa = "<<hipotenusa
    <<" Centro = "<<vDescriptores[0]->centroid<<endl
    <<"Adyactente = "<<cAdya<<endl<<"Opuesto: "<<cOpuesto<<endl;
    RotatedRect rRect(vDescriptores[0]->centroid, Size2f(1*s0, 1*s1), angle);
    rRect.points(e);



    //vDescriptores[0]=rgbFrame
}



if(vDescriptores.size()>0)
{
  circle(rgbFrame, vDescriptores[0]->centroid, 7, Scalar(100,10,100));
  circle(rgbFrame, e[0], 3, Scalar(40,10,200));
  circle(rgbFrame, e[1], 3, Scalar(140,100,30));
  circle(rgbFrame, e[2], 3, Scalar(40,10,200));
  circle(rgbFrame, e[3], 3, Scalar(140,100,30));
  if((angle>=90 and angle<=180) or (angle<(-91) and angle>=(-180)))
    arrowedLine(rgbFrame, vDescriptores[0]->centroid, Point(cAdya+vDescriptores[0]->centroid.x, -cOpuesto+vDescriptores[0]->centroid.y), Scalar(140,100,30));
  else
    arrowedLine(rgbFrame, vDescriptores[0]->centroid, Point(-cAdya+vDescriptores[0]->centroid.x, -cOpuesto+vDescriptores[0]->centroid.y), Scalar(140,100,30));

}

*/
