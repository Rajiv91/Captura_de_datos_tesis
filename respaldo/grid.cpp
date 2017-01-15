#include <iostream> 
#include <cstdlib>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <highgui.h>
#include "opencv2/opencv.hpp"
#include "time.h"
#include <opencv2/highgui/highgui.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#define ROWS_GRID 1000//1000
#define COLS_GRID 1780//1560
#define CIRCLE_RAD 20
#define HD 1
#define IM_WIDTH_VGA 640
#define IM_HEIGHT_VGA 480
#define IM_WIDTH_HD 1280
#define IM_HEIGHT_HD 720
//Dimensiones de la pantalla
#define W 1.095
#define L 0.615
#define CY 0.028


using namespace std;
using namespace cv;
float dimXPix=W/COLS_GRID;
float dimYPix=L/ROWS_GRID;

char currentPath[100]= "/home/rajiv/Documentos/seminario/Grid/capturas_19Dic/";
void *getCoordinate(void *ptr);
void *getFrame(void *ptr);
pthread_mutex_t mutex1= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  condition_var   = PTHREAD_COND_INITIALIZER;

bool flagExit=false;
bool flagChange=false;

int fd1;
char* buff;
int rd=0;
int nbytes,tries;
Mat frame;
VideoCapture capture;
void paintGrid(Mat &Grid, Point2f coordinate, int width, int height);
void getCoordinate(Point2f& coordinates, string line);

int main(int argc, char **argv)
{
  cout<<"dimXPix: "<<dimXPix<<endl;
  if(argc<3)
  {
    cout<<"Faltan parámetros!!"<<endl;
    exit(0);
  }

  int width, height;

  width =COLS_GRID/100;
  height=ROWS_GRID/ 100;
  cout<<"Dimensiones: "<<width<<endl<<height<<endl;
  ifstream in(argv[1]);
  ofstream out(argv[2]);
  
  Point2f coordinate;
  char key; 
  pthread_t thread1;
  Mat Grid;
  Grid =Mat::zeros(ROWS_GRID,COLS_GRID, CV_8UC3);
  cout<<Grid.cols<<endl;
  cvNamedWindow("Grid",WINDOW_NORMAL);

    //*************Cámara**********
    cvNamedWindow("frame",WINDOW_NORMAL);//WINDOW_AUTOSIZE );
    //VideoCapture capture;
    //Mat frame;
    double scaleX, scaleY;
    if (!capture.open(1))
    {
      cout<<"no pudo abrir la camara, se usara la webcam integrada"<<endl; //si no se puede acceder a la cámara USB usa la webcam de la laptop
      capture.open(0);
    }
    if(!capture.isOpened())
    {
      cout<<"No abre"<<endl;
      return -1;
    }
        cout<<"Se abrió la cámara"<<endl;
    #if HD
    capture.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    #else
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    #endif

    capture>>frame;
    flip(frame, frame, 1);
  const char *message1="Iniciando thread 1...";
  pthread_create(&thread1, NULL, &getFrame, (void *)message1);
  int contFrames=0;
  char fileNameTemp[100];
  char fileName[20]= "Imagen";
  char ext[]=".jpg";
  string line;
  Mat localFrame;
  while(getline(in, line))
  {
    //capture>>frame;
      localFrame=frame.clone();
      #if HD
      scaleX=(double)IM_WIDTH_HD/localFrame.cols;
      scaleY=(double)IM_HEIGHT_HD/localFrame.rows;
      #else
      scaleX=(double)IM_WIDTH_VGA/localFrame.cols;
      scaleY=(double)IM_HEIGHT_VGA/localFrame.rows;
      #endif
    if (scaleX != 1. || scaleY != 1.)
        resize(localFrame, localFrame, Size(), scaleX, scaleY, INTER_AREA);
    imshow("frame", localFrame);

    //sleep(1);
    //imshow("Grid", Grid);
    getCoordinate(coordinate, line);
    paintGrid(Grid, coordinate, width, height);
    imshow("Grid", Grid);
    //key='s';
    Grid =Mat::zeros(ROWS_GRID,COLS_GRID, CV_8UC3);
    string bufferTemp;
    bool flagNextImage=false;
    while(!flagNextImage)
    {
    key=waitKey(10);
    if(key=='c') 
    {
      cout<<"Capturar"<<endl;
      bool flag=true;
      while(flag)//guarda los datos del imu
      {
	fd1=open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	//fd1=open("/dev/ttyACM1", O_RDWR | O_NOCTTY| FNDELAY); 
        /* wait for the Arduino to reboot */
        usleep(500000);
        bufferTemp.erase();
        int N=80;
        struct termios options;
        char buffer[N];
        buffer[0]=0;
        bool flag2=false;
        unsigned char checksum1=0, checksum2=0;
          if (rd < 0)
            fputs("read failed!\n", stderr);
          else
          {
	    fcntl(fd1, F_SETFL,0);// Sets the read() function to return NOW and not wait for data to enter buffer if there isn't anything there
		//Configure port for 8N1 transmission
	/*	tcgetattr(fd1, &options);					//Gets the current options for the port
		cfsetispeed(&options, B9600);				//Sets the Input Baud Rate
		cfsetospeed(&options, B9600);				//Sets the Output Baud Rate
		options.c_cflag |= (CLOCAL | CREAD);		//? all these set options for 8N1 serial operations
		options.c_cflag &= ~PARENB;					//? 
		options.c_cflag &= ~CSTOPB;					//?
		options.c_cflag &= ~CSIZE;					//?
		options.c_cflag |= CS8;						//?

		tcsetattr(fd1, TCSANOW, &options);			//Set the new options for the port "NOW"*/
	    rd=read(fd1,buffer,sizeof(buffer));
            //cout<<"bytes: "<<(char)buffer[0]<<endl<<(char)buffer[1]<<endl<<(char)buffer[2]<<endl<<(char)buffer[3]<<endl<<(char)buffer[4]<<endl<<endl;
            //printf("Bytes sent are %s\n",buffer);
            //cout<<"hola"<<endl;
            printf("bytes: %d\n", rd);
            //printf("Bytes are: \n%s\n",buffer);

            for(int i=0; i<N; i++)
            {
              if(buffer[i]=='*')//La cadena llegó completa solo falta verificar checksum
              {
                //bufferTemp[cont]=buffer[i];
                bufferTemp.push_back(buffer[i]);
                checksum2=buffer[i+1];
                checksum1^=buffer[i];
                cout<<"cadena válida"<<endl<<"tamaño 2: "<<bufferTemp.length()<<endl<<"cadena: "<<bufferTemp<<endl<<
                "checksum1: "<<checksum1<<", checksum2: "<<checksum2<<endl;
                if(checksum1==checksum2)
                {
                  flag=false;
                  cout<<"Checksum correcto"<<endl;
                  bufferTemp.erase(bufferTemp.length()-1,1);
                  bufferTemp.erase(0, 1);
		  stringstream numString;
		  numString<<contFrames;
		  snprintf(fileNameTemp, 99,"%s%s%06d%s",currentPath,fileName,contFrames, ext);
                  cout<<"Nombre: "<<fileNameTemp<<endl;
		  imwrite(fileNameTemp, localFrame);

                  out<<bufferTemp<<", "<<fileNameTemp<<", "<<coordinate.x<<", "<<coordinate.y<<endl;
                  cout<<"Guardado!!"<<endl;
                  //out
                  contFrames++;
                  flagNextImage=true;
                }
                else
                  cout<<"checksum incorrecto!"<<endl<<"se tomará otra muestra"<<endl;

                break;
              }
              if(buffer[i]=='$')
              {
                //bufferTemp[cont]=buffer[i];
                bufferTemp.push_back(buffer[i]);
                //cout<<"dato válido en "<<i<<endl;
                flag2=true;//Primer caracter
                checksum1^=buffer[i];
              }
              else if(flag2)
              {
                //bufferTemp[cont]=buffer[i];
                bufferTemp.push_back(buffer[i]);
                checksum1^=buffer[i];
              }
              if(i==(N-1))
                cout<<"cadena inválida"<<endl;
            }
            //cout<<"Primer byte: "<<buffer[0]<<", segundo: "<<buffer[1]<<", tercero: "<<buffer[2]<<endl;
            /*while(true)//Llegó algo, hay que comprobar que el paquete esté correcto
            {

              char key2='x';
              key2=waitKey(10);
              if(key2=='g')
              {
                flag=false;
		stringstream numString;
		numString<<contFrames;
		snprintf(fileNameTemp, 99,"%s%s%06d%s",currentPath,fileName,contFrames, ext);
                cout<<"Nombre: "<<fileNameTemp<<endl;
		imwrite(fileNameTemp, frame);
                out<<buffer<<", "<<fileNameTemp<<endl;
                cout<<"Guardado!!"<<endl;
                //out
                contFrames++;
                break;//guarda
              }
              if(key2=='n') break;
            }*/           
          }
         cout<<"cerrando puerto "<<endl;
	close(fd1);
         cout<<"puerto cerrado"<<endl;
        //sleep(1);
      }

    }//While(flag)

      if(key=='q') 
        break;
    //waitKey();
    }//for(;;)
  }//while(getline)
  out.close();
  //cout<<"Esperando a thread 1..."<<endl;
  flagExit=true;
  //pthread_join(thread1, NULL);
  return 0;
}


void paintGrid(Mat &Grid, Point2f coordinate, int width, int height)
{
  
  Grid =Mat::zeros(ROWS_GRID,COLS_GRID, CV_8UC3);
  int xGrid, yGrid;
  //xGrid=width*coordinate.x;
  //yGrid = height*coordinate.y;
  //Se cambia el marco de referencia y se pasa a pixeles
  xGrid=(coordinate.x+W/2.0)/dimXPix;
  yGrid = -(coordinate.y-CY)/dimYPix;
  cout<<"xGrid: "<<xGrid<<", yGrid: "<<yGrid<<endl;
  Mat roi=Grid(Rect(xGrid, yGrid, width, height));
  //roi.setTo(255);
  //circle(Grid, /*Point(20, 20)*/Point(xGrid+CIRCLE_RAD, yGrid+CIRCLE_RAD), CIRCLE_RAD, Scalar(180, 100, 140) ,10);
  circle(Grid, Point(xGrid+CIRCLE_RAD, yGrid+CIRCLE_RAD), CIRCLE_RAD, Scalar(rand()%256, rand()%256, rand()%256) ,10);
 // cout<<"coordenada "<<coordinate<<endl;

}

void getCoordinate(Point2f& coordinates, string line)
{
  int pos=0, pos_ant=0;
  int SxField=4, SyField=5;
  string SxTemp, SyTemp;
  for(int i=0; i<6; i++)
  {
    pos=line.find(',', pos_ant);
    if(i==SxField)
      SxTemp=line.substr(pos_ant, pos-pos_ant).c_str();
    if(i==SyField)
      SyTemp=line.substr(pos_ant, pos-pos_ant).c_str();
    pos_ant=pos+1;
  }
  coordinates.x=atof(SxTemp.c_str());
  coordinates.y=atof(SyTemp.c_str());
  //coordinates.x=-W/2;
  //coordinates.y=CY;
  cout<<"Sx = "<<coordinates.x<<", Sy = "<<coordinates.y<<endl;
}

void *getFrame(void *ptr)
{
  cout<<"Iniciando hilo de la captura"<<endl;
  
    cvNamedWindow("nuevo",WINDOW_NORMAL);//WINDOW_AUTOSIZE );
   /* VideoCapture capture;
    
    double scaleX, scaleY;
    if (!capture.open(1))
    {
      cout<<"no pudo abrir la camara, se usara la webcam integrada"<<endl; //si no se puede acceder a la cámara USB usa la webcam de la laptop
      capture.open(0);
    }
    if(!capture.isOpened())
    {
      cout<<"No abre"<<endl;
      //return -1;
    }
        cout<<"Se abrió la cámara"<<endl;
        
    #if HD
    capture.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    #else
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    #endif*/


  for(;;)
  {
      pthread_mutex_lock( &mutex1 );      
      capture>>frame;
      flip(frame, frame, 1);
      imshow("nuevo", frame);
      pthread_mutex_unlock( &mutex1 );
      
      /*#if HD
      scaleX=(double)IM_WIDTH_HD/frame.cols;
      scaleY=(double)IM_HEIGHT_HD/frame.rows;
      #else
      scaleX=(double)IM_WIDTH_VGA/frame.cols;
      scaleY=(double)IM_HEIGHT_VGA/frame.rows;
      #endif
    if (scaleX != 1. || scaleY != 1.)
        resize(frame, frame, Size(), scaleX, scaleY, INTER_AREA);
      */
    //imshow("frame", frame);


    waitKey(10);
  }
}
/*void *getCoordinate(void *ptr)
{
  char *message=(char*)ptr;
  cout<<message<<endl;
  for(;;)
  {
    if(flagExit) break;

    cin>>coordinate.x;
    cin>>coordinate.y;
    //cout<<"Coordenada introducida: "<<coordinate<<endl;
    flagChange=true;
  }
}*/


