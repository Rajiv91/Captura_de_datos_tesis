#ifndef __ETIQUETACION__
#define __ETIQUETACION__
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

//Estructura de pila utilizado para almacenar logar que falta visitar en el
//algoritmo de etiquetación. La pila no es dinámica, sino estática por razones
//de velocidad.
struct stack
{
    struct coor
    {
        int x, y;
    };

    coor *S;
    int sz, idx;
    stack (int t)
    {
        sz = t;
        idx = 0;
        if (sz > 0)
            S = new coor[sz];
        else
            S = 0;
    }
    ~stack()
    {
       if (S)
        delete[] S;
    }
    bool push (int x, int y)
    {
        if (idx < sz)
        {
            S[idx].x = x;
            S[idx].y = y;
            idx++;
            return true;
        }
        return false;
    }
    bool pop(int &x, int &y)
    {
        if (idx > 0)
        {
            idx--;
            x = S[idx].x;
            y = S[idx].y;
            return true;
        }
        return false;
    }
    bool empty()
    {
        return !idx;
    }
};

//Algoritmo no recursivo de etiquetación.
int Etiquetacion (Mat &I, Mat &O)
{
    int i, j, k, l, x, y, xl, yk, label;

    stack S(I.rows*I.cols);

    label = 0;
    O = Mat::zeros(I.size(), CV_32SC1);

    for (i=0;i<I.rows;++i)
    {
        for (j=0;j<I.cols;++j)
        {
            if (I.at<uchar>(i, j) != 0 && O.at<int>(i,j) == 0)
            {
                label++;
                O.at<int>(i, j) = label;
                S.push(j, i);
                do
                {
                    S.pop(x, y);
                    for (k=-1;k<2;++k)
                    {
                        yk = y + k;
                        if (yk>=0 && yk < I.rows)
                            for (l=-1;l<2;++l)
                            {
                                if (!k && !l)
                                    continue;
                                xl = x + l;
                                if (xl>=0 && xl < I.cols)
                                    if (I.at<uchar>(yk, xl) != 0 && O.at<int>(yk, xl) == 0)
                                    {
                                        O.at<int>(yk, xl) = label;
                                        S.push(xl, yk);
                                    }
                            }
                    }
                } while (!S.empty());
            }
        }
    }
    return label;
}

//Estructura necesaria para el algoritmos shuffleImage.
struct val
{
    double n;
    int idx;
};


//Funciones de comparación utilizada en shuffleImage,
int cmp (const void *a, const void *b)
{
    if ( ((val *)a)->n > ((val *)b)->n)
        return 1;
    if (((val *)a)->n < ((val *)b)->n)
        return -1;
    return 0;
}

//Esta función revuelve los valores de los pixeles en la imagen.
//Se utiliza para el despliegue de las etiquetas; hace mas facil diferenciar
//etiquetas diferente al lograra que bloques contiguos tengan colores menos
//similares.
template <typename X>
void shuffleImage(Mat &I, int N)
{
    val *lst;
    int i;
    X *apu, *end;

    srand48(time(NULL));
    lst = new val[N];
    for (i=0;i<N;++i)
    {
        lst[i].n = drand48();
        lst[i].idx = i;
    }
    qsort((void *)lst, N, sizeof (val), cmp);

    for (i=0;i<I.rows;++i)
    {
        apu = I.ptr<X>(i);
        end = apu + I.cols;
        for (;apu < end;++apu)
        {
            if (*apu)
                *apu = lst[*apu].idx;
        }
    }
    delete[] lst;
}

int computeAreas(Mat &labels, vector<int>& vAreaEtiquetas, vector<bool>& vLabelFlags, int percentageArea, int percentageArea2)
{
  int i, j, cont=0;

  for(i=0; i<labels.rows; i++)
  {
    for(j=0; j<labels.cols; j++)
    {
      vAreaEtiquetas[labels.at<int>(i,j)]++;//La etiqueta cero es el fondo
    }
  }
  for(int id=1; id<vAreaEtiquetas.size(); id++)
  {
      if(vAreaEtiquetas[id]>percentageArea and vAreaEtiquetas[id]<percentageArea2)
      {
        //cout<<"id: "<<id<<" area: "<<vAreaEtiquetas[id]<<endl;
        vLabelFlags[id]=true;
        cont++;
      }
  }
  return cont;
}

void blobFilter(Mat& labels, Mat& vFrameMask, vector<bool>& vLabelFlags)
{
  for(int i=0; i<labels.rows; i++)
  {
    for(int j=0; j<labels.cols; j++)
    {
      if(labels.at<int>(i,j)!=0 and vLabelFlags[labels.at<int>(i,j)]==false)//Se pinta con cero su etiqueta
      {
        vFrameMask.at<uchar>(i, j)=0;
        //cout<<"Antes: "<<labels.at<int>(i, j)<<endl;
        labels.at<int>(i, j)=0;
        //cout<<"Después: "<<labels.at<int>(i, j)<<endl;
      }
    }
  }
}

#endif
