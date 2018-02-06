#include <stdlib.h>
//#include <string.h>
#include <math.h>
#include "conio.h"
#include "Tserial.h"
#include <iostream>
#include <string>
using namespace std;

// Screen surface


// Screen pitch
#define PITCH (gScreen->pitch / 4)
// Screen width
#define WIDTH 600//480
// Screen height
#define HEIGHT 400//320
// Definition of PI
#define PI 3.1415926535897932384626433832795f

// Physics iterations per second
#define PHYSICSFPS 100
#define BUFFER_LENGTH 9
#define SIZE	1
#define NUM_BIAS	10

// Last iteration's tick value
int gLastTick;
char DataBufferChar[BUFFER_LENGTH];


void parse_data(int xyz_rate[7], char frm[47], unsigned short len)
{	
	unsigned short i=0, j=0, seg=0, digit=0;
	char sstr[5];
	
	while(i<len)
	{
		while(frm[i]!=0x0d)
		{
			sstr[j++]=frm[i++];
		}
		sstr[j]=0;
		++i;	j=0;
		xyz_rate[seg]=atoi(sstr);
		++seg;
	}
}

void grate_to_eulerrate(float eangle[3], float omega[3], float erate[3])
{
  // eangle[3]: current Euler angles at which the gyro's rate of turn (omega[3]) is read
  // erate[3]: Euler rate at the current Euler angles
 
  erate[0]=omega[0]*cos(eangle[1]) + omega[2]*sin(eangle[1]);
  erate[2]=(-omega[0]*sin(eangle[1]) + omega[2]*cos(eangle[1]))/cos(eangle[0]);
  erate[1]=omega[1] - erate[2]*sin(eangle[0]);
}


// Entry point
int main(int argc, char *argv[])
{
	float euler[3], omega[3], erate[3], r2d, d2r;
    int i=0, len, frm_num=0, j, k;
    Tserial *com;
	char temp=0x01;
	int index = 0;
	char str[47];
	int xyz[7], xyz_bias[7];
	char sc[2]={0x0d, 0x0a};
	FILE *fp;
  
	com = new Tserial();
    if (com!=0)
    {
        com->connect("COM4", 115200, spNONE);
	}

	fp=fopen("gread.dat","w");

//	Send command to trigger continuous data frames
	com->sendArray(sc, 2);
	while(com->getChar() != 0x0c);

//compute bias drift
	for(k=0;k<7;k++)
		xyz_bias[k] = 0;

	printf("Computing bias drift...");
	while(frm_num<NUM_BIAS)
	{
		temp = com->getChar();
		if(temp != 0x0c)
			str[i++] = temp;
		else
		{
//			printf("%s\n", str);	//fprintf(fp,"%s\n",str);
//			printf("frame: %d", frm_num+1);
			frm_num++;
			len = i-1;
			parse_data(xyz, str, len);
			for(k=1;k<7;k++)
				xyz_bias[k] += xyz[k];			
			str[i] = 0;
			i = 0;
		}
	}
	for(k=1;k<7;k++)
		xyz_bias[k]/=NUM_BIAS;

	printf("bias computation done.\n");
	fprintf(fp, "bias: ");
	for(k=1;k<7;k++)
		fprintf(fp,"%d ", xyz_bias[k]);
	fprintf(fp,"\n");

	for(k=0;k<3;k++)
		euler[k]=0.0;
	frm_num=0;
	r2d=180.0/3.1415927;		d2r=3.1415927/180.0;
	while(1)
	{
		temp = com->getChar();
		if(temp != 0x0c)
			str[i++] = temp;
		else
		{
			printf("%s\n", str);
			frm_num++;
			len=i-1;
			parse_data(xyz, str, len);
			for(j=0;j<7;j++)
				printf("%d ", xyz[j]);
			printf("\n");

			for(k=1;k<7;k++)
				xyz[k] -= xyz_bias[k];

			for(k=0;k<3;k++)
				omega[k] = xyz[k+1]*(80.0/1092.0)*d2r;

			grate_to_eulerrate(euler, omega, erate);

			for(k=0;k<3;k++)
				euler[k] += erate[k]*0.01;
			
			printf("pitch:%f roll:%f yaw:%f\n",euler[0]*r2d, euler[1]*r2d, euler[2]*r2d);
			str[i]=0;
			fprintf(fp,"%s bytes=%d\n",str, i+1);
			i=0;
		}

		if(_kbhit())
		{
			fclose(fp);	
			com->sendChar(0x1b);
			return 0;
		}
	}
}
