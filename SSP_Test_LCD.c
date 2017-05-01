/*
=================================================================================
 Name        : SSP_Test_lcd.c
 Author      : Tianran Che, SATYAM SHETH
 Version     :
 Copyright   : $(copyright)
 Description : this program allow us using the 1.8" Color TFT LCD via ssp ports,
 	 	 	   able to display screensaver of rectangle rotation and tree on the
 	 	 	   screen
=================================================================================
*/
#include <cr_section_macros.h>
#include <NXP/crp.h>
#include "longhorn_sunset.h"

// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

#include "LPC17xx.h"                        /* LPC17xx definitions */
#include "ssp.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "font.h"
#include "extint.h"


/* Be careful with the port number and location number, because
some of the location may not exist in that port. */
#define PORT_NUM            0
#define LOCATION_NUM        0
#define TREE_NUM           2

#define OFFSETX 64
#define OFFSETY 80
#define OFFSETZ 30
#define CUBESIZE 40
#define ARRAY_SIZE 15
#define NUM_LINES 19

double x1prime,y1prime,z1prime,x2prime,y2prime,z2prime,x3prime,y3prime,z3prime,x4prime,y4prime,z4prime,x5prime,y5prime,z5prime,x6prime,y6prime,z6prime,x7prime,y7prime,z7prime,x8prime,y8prime,z8prime = 0;

double alpha=0;
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

uint8_t src_addr[SSP_BUFSIZE];
uint8_t dest_addr[SSP_BUFSIZE];
int colstart = 0;
int rowstart = 0;

/*****************************************************************************
** Function name:       LCD_TEST
**
** Descriptions:        2D and 3D graphics engine
**
** parameters:            None
** Returned value:        None
**
*****************************************************************************/

//LCD
#define ST7735_TFTWIDTH  127
#define ST7735_TFTHEIGHT 159
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define swap(x, y) { x = x + y; y = x - y; x = x - y; }
#define PI 3.14

//Colors
#define BLACK  0x000000
#define WHITE  0xFFFFFF
#define GREEN  0x00FF00
#define RED    0xFF0000
#define BLUE   0x0000FF
#define YELLOW 0xFFFF00
#define CYAN   0x00FFFF
#define MAGENTA 0xFF00FF
#define PURPLE 0x8000FF
#define ORANGE 0xFF8000
#define OLIVE 0x808000
#define LTBLUE 0x9090FF
#define BROWN 0x330000

//Axes
int _height = ST7735_TFTHEIGHT;
int _width = ST7735_TFTWIDTH;
int cursor_x = 0, cursor_y = 0;

#define NUM_PATTERN_ITERATIONS 13
#define R_angle 330
uint32_t colour_array[12] = {BLACK, GREEN, RED, BLUE, YELLOW, MAGENTA, PURPLE, ORANGE, OLIVE, LTBLUE};
int colour_choice;
int count;
float lambda;
time_t t;
double s_para=0;

struct world{
	int x;
	int y;
}s0,st1,st2,point1,point2,sos;

struct types{
	int xe,ye,ze;
}saxis;

struct matrix{
	double x1,y1,z1,x2,y2,z3,x4,y4,z4,x5,y5,z5,x6,y6,z6,x7,y7,z7,x8,y8,z8;
};
// Single point light source
struct lightSource {
	int x, y, z;
};
static struct lightSource Ps = {100, -50, 2800};

// Normal vector
struct NVector {

	float x, y, z;
};

// 3D co-ordinates
struct shadowPoint{
	float x, y, z;
};

void square(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
	int16_t y2, int16_t x3, int16_t y3, int16_t count, uint32_t colour) {
	printf("value of xo = %d\n",x0);
	// Exit condition
	if(count == 0) {
		return;
	}

	drawLine(x0, y0, x1, y1, colour);
	drawLine(x1, y1, x2, y2, colour);
	drawLine(x2, y2, x3, y3, colour);
	drawLine(x3, y3, x0, y0, colour);

	int q0 = x0 + 0.8 * (x1 - x0);
	int q1 = y0 + 0.8 * (y1 - y0);

	int q2 = x1 + 0.8 * (x2 - x1);
	int q3 = y1 + 0.8 * (y2 - y1);

	int q4 = x2 + 0.8 * (x3 - x2);
	int q5 = y2 + 0.8 * (y3 - y2);

	int q6 = x3 + 0.8 * (x0 - x3);
	int q7 = y3 + 0.8 * (y0 - y3);

	lcddelay(100);
//printf("value of q0 = %d\n",q0);
//printf("value of q1 = %d\n",q1);
	square(q0, q1, q2, q3, q4, q5, q6, q7, count - 1, colour);
}

void spiwrite(uint8_t c)
{
    int portnum = 0;
    src_addr[0] = c;
    SSP_SSELToggle( portnum, 0 );
    SSPSend( portnum, (uint8_t *)src_addr, 1 );
    SSP_SSELToggle( portnum, 1 );
}

void writecommand(uint8_t c) {
    LPC_GPIO0->FIOCLR |= (0x1<<21);
    spiwrite(c);
}

void writedata(uint8_t c) {

    LPC_GPIO0->FIOSET |= (0x1<<21);
    spiwrite(c);
}

void writeword(uint16_t c) {

    uint8_t d;
    d = c >> 8;
    writedata(d);
    d = c & 0xFF;
    writedata(d);
}

void write888(uint32_t color, uint32_t repeat) {
    uint8_t red, green, blue;
    int i;
    red = (color >> 16);
    green = (color >> 8) & 0xFF;
    blue = color & 0xFF;
    for (i = 0; i< repeat; i++) {
        writedata(red);
        writedata(green);
        writedata(blue);
    }
}
// function to set adress window between the given coordinates
void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
                    uint16_t y1) {

      writecommand(ST7735_CASET);
      writeword(x0);
      writeword(x1);
      writecommand(ST7735_RASET);
      writeword(y0);
      writeword(y1);

}
// function to draw pixel on the screen
void drawPixel(int16_t x, int16_t y, uint32_t color) {
    if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

    setAddrWindow(x,y,x+1,y+1);
    writecommand(ST7735_RAMWR);
    write888(color, 1);
}
//Function to perform LCD delay in milliseconds
void lcddelay(int ms)
{
	int count = 24000;
	int i;

	for ( i = count*ms; i--; i > 0);
}


//Initialize LCD
void lcd_init()
{
/*
 * portnum     = 0 ;
 * cs         = p0.16 / p0.6 ?
 * rs        = p0.21
 * rst        = p0.22
 */
    uint32_t portnum = 0;
    int i;
    printf("LCD initialized\n");
    /* Notice the hack, for portnum 0 p0.16 is used */
    if ( portnum == 0 )
      {
        LPC_GPIO0->FIODIR |= (0x1<<16);        /* SSP1, P0.16 defined as Outputs */
      }
      else
      {
        LPC_GPIO0->FIODIR |= (0x1<<6);        /* SSP0 P0.6 defined as Outputs */
      }
    /* Set rs(dc) and rst as outputs */
    LPC_GPIO0->FIODIR |= (0x1<<21);        /* rs/dc P0.21 defined as Outputs */
    LPC_GPIO0->FIODIR |= (0x1<<22);        /* rst P0.22 defined as Outputs */


    /* Reset sequence */
    LPC_GPIO0->FIOSET |= (0x1<<22);

 lcddelay(500);                        /*delay 500 ms */
    LPC_GPIO0->FIOCLR |= (0x1<<22);
 lcddelay(500);                        /* delay 500 ms */
    LPC_GPIO0->FIOSET |= (0x1<<22);
 lcddelay(500);                        /* delay 500 ms */

 for ( i = 0; i < SSP_BUFSIZE; i++ )    /* Init RD and WR buffer */
        {
            src_addr[i] = 0;
            dest_addr[i] = 0;
        }

     /* do we need Sw reset (cmd 0x01) ? */

     /* Sleep out */
     SSP_SSELToggle( portnum, 0 );
     src_addr[0] = 0x11;    /* Sleep out */
     SSPSend( portnum, (uint8_t *)src_addr, 1 );
     SSP_SSELToggle( portnum, 1 );

lcddelay(200);
    /* delay 200 ms */
    /* Disp on */
     SSP_SSELToggle( portnum, 0 );
     src_addr[0] = 0x29;    /* Disp On */
     SSPSend( portnum, (uint8_t *)src_addr, 1 );
     SSP_SSELToggle( portnum, 1 );
    /* delay 200 ms */
lcddelay(200);
}

void fillrect(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint32_t color)
{
	int16_t i;
	int16_t width, height;

	width = x1-x0+1;
	height = y1-y0+1;
	setAddrWindow(x0,y0,x1,y1);
	writecommand(ST7735_RAMWR);
	write888(color,width*height);
}
unsigned int randr(unsigned int min, unsigned int max){
	double scaled = (double)rand()/RAND_MAX;
	return (max - min +1)*scaled + min;
}
//-------------------------------------------------------


//---------------------------------------------------------------------------------------
//Draw line function
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,uint32_t color)
{
	int16_t slope = abs(y1 - y0) > abs(x1 - x0);
	if (slope)
	{
	swap(x0, y0);
	swap(x1, y1);
	}

	if (x0 > x1)
	{
	swap(x0, x1);
	swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1)
	{
	ystep = 1;
	} else
	{
	ystep = -1;
	}

	for (; x0<=x1; x0++)
	{
	if (slope)
	{
	drawPixel(y0, x0, color);
	} else
	{
	drawPixel(x0, y0, color);
	}
	err -= dy;
	if (err < 0)
	{
	y0 += ystep;
	err += dx;
	}
	}
}


void VLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	drawLine(x, y, x, y+h-1, color);
 }

 void HLine(int16_t x, int16_t y,  int16_t w, uint16_t color)
 {
	 drawLine(x, y, x+w-1, y, color);
 }

//------------------------------------------------------------------------------------------------------


 struct world worldCordinateSystem(int x, int y, int z)
 {
 	int xe=saxis.xe, ye=saxis.ye, ze=saxis.ze;
 //printf("value of saxis.xe = %d\n",saxis.xe); // =100
 	int d=130;
 	int x0=50,y0=95;            //x0=50 , y0=105
 	struct world s;

 	double theta,phi,rho;
 	double xprime,yprime,zprime;

 	rho = sqrt(pow(xe,2)+pow(ye,2)+pow(ze,2));
 	theta = acos(xe/(sqrt(pow(xe,2)+pow(ye,2))));
 	phi = acos(ze/(sqrt(pow(xe,2)+pow(ye,2)+pow(ze,2))));

 	double matrix3d[4][4]={
 							{-sin(theta),				cos(theta),					0,						0	},
 							{-cos(phi)*sin(theta),		-cos(phi)*sin(theta),		sin(phi),				0	},
 							{-sin(phi)*cos(theta),		-sin(phi)*cos(theta),		-cos(phi),				rho	},
 							{0,							0,							0,						1	}
 						};
 	//printf("value of matrix[3][0] =%lf\n",matrix3d[3][0]);
 	//printf("value of matrix[0][3] =%lf\n",matrix3d[0][3]);     [row][coloum]
 	xprime	=	((x*matrix3d[0][0])	+	(y*matrix3d[0][1])	+	(z*matrix3d[0][2])	+	(1*matrix3d[0][3]));
 	yprime	=	((x*matrix3d[1][0])	+	(y*matrix3d[1][1])	+	(z*matrix3d[1][2])	+	(1*matrix3d[1][3]));
 	zprime	=	((x*matrix3d[2][0])	+	(y*matrix3d[2][1])	+	(z*matrix3d[2][2])	+	(1*matrix3d[2][3]));

 	s.x=x0+(10+(xprime/zprime)*d);
 	s.y=y0-(30+(yprime/zprime)*d);

 	return s;
 }

 void Matrix(double x1, double y1,double z1,double x2, double y2,double z2,double x3, double y3,double z3,double x4, double y4,double z4,double x5, double y5,double z5,double x6, double y6,double z6,double x7, double y7,double z7,double x8, double y8,double z8)
{
	// double alpha=30;
  	double matrix3dr[4][4]={
				{1,					0,					0,						0	},
				{0,                 cos(alpha),		   -sin(alpha),			    0   },
				{0,					sin(alpha),		    cos(alpha),				0   },
				{0,					0,					0,						1	}
				};
  //	printf("\nvalue of alpha= %lf\n",cos(alpha));

  //	printf("\nvalue of cos alpha= %lf\n",matrix3dr[1][1]);
 // 	printf("\nvalue of sin alpha= %lf\n",matrix3dr[1][2]);

	x1prime	=	((x1*matrix3dr[0][0])	+(y1*matrix3dr[1][0])	+	(z1*matrix3dr[2][0])	+	(1*matrix3dr[3][0]));
	y1prime	=	((x1*matrix3dr[0][1])	+(y1*matrix3dr[1][1])	+	(z1*matrix3dr[2][1]) 	+	(1*matrix3dr[3][1]));
	z1prime	=	((x1*matrix3dr[0][2])	+(y1*matrix3dr[1][2])	+	(z1*matrix3dr[2][2])	+	(1*matrix3dr[3][2]));
    x2prime	=	((x2*matrix3dr[0][0])	+(y2*matrix3dr[1][0])	+	(z2*matrix3dr[2][0])	+	(1*matrix3dr[3][0]));
	y2prime	=	((x2*matrix3dr[0][1])	+(y2*matrix3dr[1][1])	+	(z2*matrix3dr[2][1])	+	(1*matrix3dr[3][1]));
	z2prime	=	((x2*matrix3dr[0][2])	+(y2*matrix3dr[1][2])	+	(z2*matrix3dr[2][2])	+	(1*matrix3dr[3][2]));
    x3prime	=	((x3*matrix3dr[0][0])	+(y3*matrix3dr[1][0])	+	(z3*matrix3dr[2][0])	+	(1*matrix3dr[3][0]));
	y3prime	=	((x3*matrix3dr[0][1])	+(y3*matrix3dr[1][1])	+	(z3*matrix3dr[2][1])	+	(1*matrix3dr[3][1]));
	z3prime	=	((x3*matrix3dr[0][2])	+(y3*matrix3dr[1][2])	+	(z3*matrix3dr[2][2])	+	(1*matrix3dr[3][2]));
    x4prime	=	((x4*matrix3dr[0][0])	+(y4*matrix3dr[1][0])	+	(z4*matrix3dr[2][0])	+	(1*matrix3dr[3][0]));
	y4prime	=	((x4*matrix3dr[0][1])	+(y4*matrix3dr[1][1])	+	(z4*matrix3dr[2][1])	+	(1*matrix3dr[3][1]));
	z4prime	=	((x4*matrix3dr[0][2])	+(y4*matrix3dr[1][2])	+	(z4*matrix3dr[2][2])	+	(1*matrix3dr[3][2]));
    x5prime	=	((x5*matrix3dr[0][0])	+(y5*matrix3dr[1][0])	+	(z5*matrix3dr[2][0])	+	(1*matrix3dr[3][0]));
	y5prime	=	((x5*matrix3dr[0][1])	+(y5*matrix3dr[1][1])	+	(z5*matrix3dr[2][1])	+	(1*matrix3dr[3][1]));
	z5prime	=	((x5*matrix3dr[0][2])	+(y5*matrix3dr[1][2])	+	(z5*matrix3dr[2][2])	+	(1*matrix3dr[3][2]));
    x6prime	=	((x6*matrix3dr[0][0])	+(y6*matrix3dr[1][0])	+	(z6*matrix3dr[2][0])	+	(1*matrix3dr[3][0]));
	y6prime	=	((x6*matrix3dr[0][1])	+(y6*matrix3dr[1][1])	+	(z6*matrix3dr[2][1])	+	(1*matrix3dr[3][1]));
	z6prime	=	((x6*matrix3dr[0][2])	+(y6*matrix3dr[1][2])	+	(z6*matrix3dr[2][2])	+	(1*matrix3dr[3][2]));
    x7prime	=	((x7*matrix3dr[0][0])	+(y7*matrix3dr[1][0])	+	(z7*matrix3dr[2][0])	+	(1*matrix3dr[3][0]));
	y7prime	=	((x7*matrix3dr[0][1])	+(y7*matrix3dr[1][1])	+	(z7*matrix3dr[2][1])	+	(1*matrix3dr[3][1]));
	z7prime	=	((x7*matrix3dr[0][2])	+(y7*matrix3dr[1][2])	+	(z7*matrix3dr[2][2])	+	(1*matrix3dr[3][2]));
    x8prime	=	((x8*matrix3dr[0][0])	+(y8*matrix3dr[1][0])	+	(z8*matrix3dr[2][0])	+	(1*matrix3dr[3][0]));
	y8prime	=	((x8*matrix3dr[0][1])	+(y8*matrix3dr[1][1])	+	(z8*matrix3dr[2][1])	+	(1*matrix3dr[3][1]));
	z8prime	=	((x8*matrix3dr[0][2])	+(y8*matrix3dr[1][2])	+	(z8*matrix3dr[2][2])	+	(1*matrix3dr[3][2]));
//return y8prime;

}

 void worldcoordinatAxis()
 {
 	struct world s1,s2,s3;

 	s1=worldCordinateSystem(175,0,0);
 		drawLine(s0.x,s0.y,s1.x,s1.y,BLUE);
 	s1=worldCordinateSystem(175,1,0);
 		drawLine(s0.x,s0.y,s1.x,s1.y,BLUE);
 	s1=worldCordinateSystem(175,0,1);
 		drawLine(s0.x,s0.y,s1.x,s1.y,BLUE);


 	s2=worldCordinateSystem(0,175,0);
 		drawLine(s0.x,s0.y,s2.x,s2.y,RED);
 	s2=worldCordinateSystem(1,175,0);
 		drawLine(s0.x,s0.y,s2.x,s2.y,RED);
 	s2=worldCordinateSystem(0,175,1);
 		drawLine(s0.x,s0.y,s2.x,s2.y,RED);


 	s3=worldCordinateSystem(0,0,175);
 		drawLine(s0.x,s0.y,s3.x,s3.y,GREEN);
 	s3=worldCordinateSystem(1,0,175);
 		drawLine(s0.x,s0.y,s3.x,s3.y,GREEN);
 	s3=worldCordinateSystem(0,1,175);
 		drawLine(s0.x,s0.y,s3.x,s3.y,GREEN);
 }

 void generatecuberot(int a, int b, int c)
 {
 	struct world s1,s2,s3,s4;
 	struct world s5,s6,s7,s8;

 	//uint16_t outline_color = BLACK;

 	s1=worldCordinateSystem(x1prime+a,y1prime+b,z1prime+c);
 	s2=worldCordinateSystem(x2prime+a,y2prime+b,z2prime+c);
 	s3=worldCordinateSystem(x3prime+a,y3prime+b,z3prime+c);
 	s4=worldCordinateSystem(x4prime+a,y4prime+b,z4prime+c);

 	drawLine(s1.x,s1.y,s2.x,s2.y,BLACK);
 	drawLine(s2.x,s2.y,s3.x,s3.y,BLACK);
 	drawLine(s3.x,s3.y,s4.x,s4.y,BLACK);
 	drawLine(s4.x,s4.y,s1.x,s1.y,BLACK);


 	s5=worldCordinateSystem(x5prime+a,y5prime+b,z5prime+c);
 	s6=worldCordinateSystem(x6prime+a,y6prime+b,z6prime+c);
 	s7=worldCordinateSystem(x7prime+a,y7prime+b,z7prime+c);
 	s8=worldCordinateSystem(x8prime+a,y8prime+b,z8prime+c);

 	drawLine(s2.x,s2.y,s6.x,s6.y,BLACK);
 	drawLine(s3.x,s3.y,s7.x,s7.y,BLACK);
 	drawLine(s4.x,s4.y,s8.x,s8.y,BLACK);
 	drawLine(s6.x,s6.y,s7.x,s7.y,BLACK);
 	drawLine(s7.x,s7.y,s8.x,s8.y,BLACK);
// 	printf("value of yprime= %lf\n",x1prime);
// 	printf("value of yprime= %lf\n",y1prime);
// 	printf("value of yprime= %lf\n",z1prime);
// 	printf("value of yprime= %lf\n",x2prime);
// 	printf("value of yprime= %lf\n",y2prime);
// 	printf("value of yprime= %lf\n",z2prime);
// 	printf("value of yprime= %lf\n",x3prime);
// 	printf("value of yprime= %lf\n",y3prime);
// 	printf("value of yprime= %lf\n",z3prime);
// 	printf("value of yprime= %lf\n",x4prime);
// 	printf("value of yprime= %lf\n",y4prime);
// 	printf("value of yprime= %lf\n",z4prime);
// 	printf("value of yprime= %lf\n",x5prime);
// 	printf("value of yprime= %lf\n",y5prime);
// 	printf("value of yprime= %lf\n",z5prime);
// 	printf("value of yprime= %lf\n",x6prime);
// 	printf("value of yprime= %lf\n",y6prime);
// 	printf("value of yprime= %lf\n",z6prime);
// 	printf("value of yprime= %lf\n",x7prime);
// 	printf("value of yprime= %lf\n",y7prime);
// 	printf("value of yprime= %lf\n",z7prime);
// 	printf("value of yprime= %lf\n",x8prime);
// 	printf("value of yprime= %lf\n",y8prime);
// 	printf("value of yprime= %lf\n",z8prime);

 }

 void generatecubedis(int size, int a, int b, int c)
   {
   	struct world s1,s2,s3,s4;
   	struct world s5,s6,s7,s8;

   	//uint16_t outline_color = BLACK;
   	s1=worldCordinateSystem(0+a,0+b,size+c);
   	 	s2=worldCordinateSystem(size+a,0+b,size+c);
   	 	s3=worldCordinateSystem(size+a,size+b,size+c);
   	 	s4=worldCordinateSystem(0+a,size+b,size+c);

   	drawLine(s1.x,s1.y,s2.x,s2.y,BLACK);
   	drawLine(s2.x,s2.y,s3.x,s3.y,BLACK);
   	drawLine(s3.x,s3.y,s4.x,s4.y,BLACK);
   	drawLine(s4.x,s4.y,s1.x,s1.y,BLACK);

    	s5=worldCordinateSystem(0+a,0+b,0+c);
   	 	s6=worldCordinateSystem(size+a,0+b,0+c);
   	 	s7=worldCordinateSystem(size+a,size+b,0+c);
   	 	s8=worldCordinateSystem(0+a,size+b,0+c);

   	drawLine(s2.x,s2.y,s6.x,s6.y,BLACK);
   	drawLine(s3.x,s3.y,s7.x,s7.y,BLACK);
   	drawLine(s4.x,s4.y,s8.x,s8.y,BLACK);
   	drawLine(s6.x,s6.y,s7.x,s7.y,BLACK);
   	drawLine(s7.x,s7.y,s8.x,s8.y,BLACK);
   	//drawLine(s5.x,s5.y,s6.x,s6.y,BLACK);
   	//drawLine(s5.x,s5.y,s8.x,s8.y,BLACK);
   	//drawLine(s5.x,s5.y,s1.x,s1.y,BLACK);

   }

 void initial(int size,int d,int b,int c,double para)
 {
 	struct world s1;

 	int i,j;
 	int map[size][size];

 	for(i=0;i<size;i++)
 	{
 		for(j=0;j<size;j++)
 		{
 	//		if(i>=5 && j>=10 && j<=size-10 && i<=15) //upper horizontal
 		//		map[i][j]=1;
 	//		else if(i>=15 && j>=10 && j<=20 && i<=54) //right vertical
 	//			map[i][j]=1;

  		 if(i>=(8*para) && j>=(20*para) && j<=(25*para) && i<=(27*para)) //left vertical
 		 				map[i][j]=1;
 			else if(i>=(25*para) && j>=(5*para) && j<=(size-(5*para)) && i<=(27*para)) //bottom horizontal
 							map[i][j]=1;

// 			else if(i>=55 && j>=30 && j<=60 && i<=64)
 //				map[i][j]=1;
 			//else if(i>=44 && j>=50 && j<=60 && i<=54)
 				//map[i][j]=1;
 			else
 				map[i][j]=0;
 		}
 	}
printf("Ok in initial\n");
 	for(i=0;i<size;i++)
 	{
 		for(j=0;j<size;j++)
 		{
 			if(map[i][j]==1)
 			{
 				s1=worldCordinateSystem(j+d,i+b,size+c);
 				drawPixel(s1.x,s1.y,0x000000);
 			}
 		}
 	}
 }

 void initialrot(int size,int d,int b,int c,double paras)
  {
  	struct world s1;

  	int i,j;
  	int map[size][size];

  	for(i=0;i<size;i++)
  	{
  		for(j=0;j<size;j++)
  		{
  			//		if(i>=5 && j>=10 && j<=size-10 && i<=15) //upper horizontal
  			 		//		map[i][j]=1;
  			 	//		else if(i>=15 && j>=10 && j<=20 && i<=54) //right vertical
  			 	//			map[i][j]=1;

  			  		 if(i>=(8*paras) && j>=(20*paras) && j<=(25*paras) && i<=(27*paras)) //left vertical
  			 		 				map[i][j]=1;
  			 			else if(i>=(25*paras) && j>=(5*paras) && j<=(size-(5*paras)) && i<=(27*paras)) //bottom horizontal
  			 							map[i][j]=1;

  			// 			else if(i>=55 && j>=30 && j<=60 && i<=64)
  			 //				map[i][j]=1;
  			 			//else if(i>=44 && j>=50 && j<=60 && i<=54)
  			 				//map[i][j]=1;
  			 			else
  			 				map[i][j]=0;
  		}
  	}
 printf("Ok in initial\n");
  	for(i=0;i<size;i++)
  	{
  		for(j=0;j<size;j++)
  		{
  			if(map[i][j]==1)
  			{
  				s1=worldCordinateSystem(j+d,(((i*cos(alpha)) + (size)*sin(alpha))+b),(((i*(-sin(alpha))) + (size*cos(alpha)))+c));
  				drawPixel(s1.x,s1.y,BLACK);
  			}
  		}
  	}
  }

 //i=((i*cos(alpha)) + (size)*sin(alpha)) size=((i*(-sin(alpha))) + (size*cos(alpha)))


 //----------------------------------------------------------------------------------------------------------------------------------------------
void GenerateBranch(float *x2, float *y2, float x1, float y1, int angle) {
	float xt, yt, xnext, ynext;
	float cs, ss, radi;
	//int a=60;
	radi = angle * (3.14159265359 / 180);
	//printf("Value of angle: %f\n",ra);
	//Doing the Translation -> 1 ---------------------------------------
	xt = *x2 - x1;
	yt = *y2 - y1;
	cs = cos(radi);
	ss = sin(radi);
	//xnext = xt * c + yt * s;
	//Doing the Rotation --> 2-----------------------------------
	xnext = xt * cs - yt * ss;  //(x2-x1) cos angle - (y2-y1) sin angle
	//ynext = xt * s - yt * c;
	ynext = xt * ss + yt * cs;  //(x2-x1) sin angle + (y2-y1) cos angle
	//Doing the translation --> 3 -------------------------------------------------------
	*x2 = xnext + x1;
	*y2 = ynext + y1;
	return;
}

void Generatetree(float x1, float y1, float x2, float y2, uint32_t color, int num) {
	//printf("inside tree value of x1: %f \n",x1);
	int x3, y3;
	if (num <= 0)
		return;
	drawLine(x1, y1, x2, y2, color);
	x3 = x1;
	y3 = y1;
	x1 = x2;
	y1 = y2;
	y2 = y2 + (y1 - y3) * .8;
	x2 = x2 + (x1 - x3) * .8;
	int d;
	d = randr(20,40);
	//printf("Value of angle: %d\n",d);
	GenerateBranch(&x2, &y2, x1, y1, d);
	drawLine(x1, y1, x2, y2, color);
	Generatetree(x1, y1, x2, y2, color, num - 1);

	GenerateBranch(&x2, &y2, x1, y1, R_angle);
	drawLine(x1, y1, x2, y2, color);
	Generatetree(x1, y1, x2, y2, color, num - 1);

	GenerateBranch(&x2, &y2, x1, y1, R_angle);
	drawLine(x1, y1, x2, y2, BLACK);
	Generatetree(x1, y1, x2, y2, color, num - 1);

}

void cubefill(int size,int d,int b,int c, double para)
{
	struct world s1,s2,s3,s4,s5,s6,s_temp;

	int i,j;
	int a[size][size];

	for(i=0;i<size;i++)
	{
		for(j=0;j<size;j++)
		{
  			    s1=worldCordinateSystem(j+d,i+b,size+c);	//top fill
			//	drawPixel(s1.x,s1.y,WHITE);
				drawPixel(s1.x,s1.y,GREEN);

				s1=worldCordinateSystem(i+d,size+b,j+c);	// right fill
			//	drawPixel(s1.x,s1.y,WHITE);
				drawPixel(s1.x,s1.y,MAGENTA);

				s1=worldCordinateSystem(size+d,j+b,i+c);	// left fill
			//	drawPixel(s1.x,s1.y,WHITE);
				drawPixel(s1.x,s1.y,YELLOW);
		}
	}
	generatecubedis(size,0+d,0+b,0+c);

	initial(size,d,b,c,para);
	//tree num =2, size =12 , j 0 to 30, i = 18 to 50

	int jk,j1,sx0,sx1,sx2,sx3,sy0,sy1,sy2,sy3,ja,ia,c1,si,sj,ss,c2;
	for (jk=0;jk<3;jk++){
	//	int ja,ia,c1;
		ja = randr(0*para,15*para);
		ia = randr(9*para,24*para);
		c1 = rand() % 10;
		j1=ja+ (5*para);
	s1=worldCordinateSystem(ia+d,size+b,ja+c);	// right fill
	s2=worldCordinateSystem(ia+d,size+b,j1+c);
	//drawLine(s1.x,s1.y,s2.x,s2.y,GREEN);
	Generatetree(s1.x, s1.y, s2.x , s2.y,GREEN, 2);


	printf("Now Squares\n");
	//int sx0,sx1,sx2,sx3,sy0,sy1,sy2,sy3,sk;



//	int si,sj,ss,c2;
	si = randr(0*para,15*para);
	sj = randr(0*para,15*para);
	ss = randr(10*para,15*para);
	c2 = rand() % 10;

	sx0 = si,sy0 = sj;
	sx1 = sx0 + ss, sy1 = sy0;
	sx2 = sx1, sy2 = sy1 + ss;
	sx3 = sx0, sy3 = sy2;

	s3=worldCordinateSystem(size+d,sx0+b,sy0+c);	// left fill
	s4=worldCordinateSystem(size+d,sx1+b,sy1+c);
	s5=worldCordinateSystem(size+d,sx2+b,sy2+c);
	s6=worldCordinateSystem(size+d,sx3+b,sy3+c);
	//drawLine(s3.x,s3.y,s4.x,s4.y,GREEN);
	//drawLine(s4.x,s4.y,s5.x,s5.y,GREEN);
	//drawLine(s5.x,s5.y,s6.x,s6.y,GREEN);
	//drawLine(s6.x,s6.y,s3.x,s3.y,GREEN);
	square(s3.x, s3.y, s4.x, s4.y, s5.x, s5.y, s6.x, s6.y, 4,colour_array[c2]);

	}

	printf("ok\n");
	 			//	drawPixel(s1.x,s1.y,0x404040);


	// Generating cube shadow

		struct NVector n;
		struct shadowPoint P0, P1, P2, P3, N1, N2, R0, R1, R2, R3, Fill_Sha, Fill_R;
		P0.x = size+d;
		P0.y = size+b;
		P0.z = size+c;
		P1.x = 0+d;
		P1.y = size+b;
		P1.z = size+c;
		P2.x = 0+d;
		P2.y = 0+b;
		P2.z = size+c;
		P3.x = size+d;
		P3.y = 0+b;
		P3.z = size+c;
		N1.x = P1.x - P0.x;
		N1.y = P1.y - P0.y;
		N1.z = P1.z - P0.z;
		N2.x = P3.x - P0.x;
		N2.y = P3.y - P0.y;
		N2.z = P3.z - P0.z;
		n.x = (N1.y * N2.z) - (N1.z * N2.y);	//X coordinate of the normal
		n.y = (N1.z * N2.x) - (N1.x * N2.z);	//Y coordinate of the normal
		n.z = (N1.x * N2.y) - (N1.y * N2.x);	//Z coordinate of the normal
		// Find lambda
		float lambdaP0, lambdaP1, lambdaP2, lambdaP3, temp_lambda;
		lambdaP0 = ((n.x * P0.x) + (n.y * P0.y) + (n.z * P0.z))/((n.x * (P0.x-Ps.x)) + (n.y * (P0.y-Ps.y)) + (n.z * (P0.z-Ps.z)));
		lambdaP1 = ((n.x * P1.x) + (n.y * P1.y) + (n.z * P1.z))/((n.x * (P1.x-Ps.x)) + (n.y * (P1.y-Ps.y)) + (n.z * (P1.z-Ps.z)));
		lambdaP2 = ((n.x * P2.x) + (n.y * P2.y) + (n.z * P2.z))/((n.x * (P2.x-Ps.x)) + (n.y * (P2.y-Ps.y)) + (n.z * (P2.z-Ps.z)));
		lambdaP3 = ((n.x * P3.x) + (n.y * P3.y) + (n.z * P3.z))/((n.x * (P3.x-Ps.x)) + (n.y * (P3.y-Ps.y)) + (n.z * (P3.z-Ps.z)));

		// R_i = P_i(xi, yi, zi) + lamda * [ P_s(xs, ys, zs) - P_i(xi, yi, zi) ]
		R0.x = P0.x + lambdaP0 * (Ps.x - P0.x);
		R0.y = P0.y + lambdaP0 * (Ps.y - P0.y);
		R0.z = P0.z + lambdaP0 * (Ps.z - P0.z);
		R1.x = P1.x + lambdaP1 * (Ps.x - P1.x);
		R1.y = P1.y + lambdaP1 * (Ps.y - P1.y);
		R1.z = P1.z + lambdaP1 * (Ps.z - P1.z);
		R2.x = P2.x + lambdaP2 * (Ps.x - P2.x);
		R2.y = P2.y + lambdaP2 * (Ps.y - P2.y);
		R2.z = P2.z + lambdaP2 * (Ps.z - P2.z);
		R3.x = P3.x + lambdaP3 * (Ps.x - P3.x);
		R3.y = P3.y + lambdaP3 * (Ps.y - P3.y);
		R3.z = P3.z + lambdaP3 * (Ps.z - P3.z);


		s1=worldCordinateSystem(R0.x, R0.y, R0.z); // P0
		s2=worldCordinateSystem(R1.x, R1.y, R1.z); // P1
		s3=worldCordinateSystem(R2.x, R2.y, R2.z); // P2
		s4=worldCordinateSystem(R3.x, R3.y, R3.z); // P3

		drawLine(s1.x, s1.y, s2.x, s2.y, BLACK);
		drawLine(s2.x, s2.y, s3.x, s3.y, BLACK);
		drawLine(s3.x, s3.y, s4.x, s4.y, BLACK);
		drawLine(s4.x, s4.y, s1.x, s1.y, BLACK);


		// Filling the shadow
		for (i=0; i<size; i++) {

			for (j=0; j<size; j++) {

				Fill_Sha.x = j+d;
				Fill_Sha.y = i+b;
				Fill_Sha.z = size+c;
				temp_lambda = ((n.x * Fill_Sha.x) + (n.y * Fill_Sha.y) + (n.z * Fill_Sha.z))/((n.x * (Fill_Sha.x-Ps.x)) + (n.y * (Fill_Sha.y-Ps.y)) + (n.z * (Fill_Sha.z-Ps.z)));
				Fill_R.x = Fill_Sha.x + temp_lambda * (Ps.x - Fill_Sha.x);
				Fill_R.y = Fill_Sha.y + temp_lambda * (Ps.y - Fill_Sha.y);
				Fill_R.z = Fill_Sha.z + temp_lambda * (Ps.z - Fill_Sha.z);
				s_temp = worldCordinateSystem(Fill_R.x, Fill_R.y, Fill_R.z);
				drawPixel(s_temp.x, s_temp.y, BROWN);
			}
		}

}

void cubefillrot(int size,int d,int b,int c,double paras)
{

	struct world s1,s2,s3,s4,s5,s6,s11,s22,s33,s44,s_temp,sss;

	int i,j;
	int a[size][size];

	for(i=0;i<size;i++)
	{
		for(j=0;j<size;j++)
		{
			s1=worldCordinateSystem(j+d,(((i*cos(alpha)) + (size)*sin(alpha))+b),(((i*(-sin(alpha))) + (size*cos(alpha)))+c));	//top fill
			//s1=worldCordinateSystem(((i*cos(alpha)) - (j)*sin(alpha)),1,((i*sin(alpha)) + (j*cos(alpha))));	//top fill

			//s1=worldCordinateSystem(j+d,i+b,size+c);	//top fill
			//	drawPixel(s1.x,s1.y,WHITE);
				drawPixel(s1.x,s1.y,CYAN);

				s1=worldCordinateSystem(i+d,(((size*cos(alpha)) + (j)*sin(alpha))+b),(((size*(-sin(alpha))) + (j*cos(alpha)))+c));	// right fill
			//	drawPixel(s1.x,s1.y,WHITE);
				drawPixel(s1.x,s1.y,MAGENTA);

				s1=worldCordinateSystem(size+d,(((j*cos(alpha)) + (i)*sin(alpha))+b),(((j*(-sin(alpha))) + (i*cos(alpha)))+c));	// left fill
			//	drawPixel(s1.x,s1.y,WHITE);
				drawPixel(s1.x,s1.y,YELLOW);
		}
	}

	generatecuberot(d,b,c);
	initialrot(size,d,b,c,paras);


	//tree num =2, size =12 , j 0 to 30, i = 18 to 50

	int jk,sx0,sx1,sx2,sx3,sy0,sy1,sy2,sy3,c1,si,sj,ss,c2;
	double ja,ia,j1;
	for (jk=0;jk<3;jk++){
	//	int ja,ia,c1;
		ja = randr(0*paras,15*paras);
		ia = randr(9*paras,24*paras);
		c1 = rand() % 10;
		j1=ja + (6*paras);
//		((size*cos(alpha)) + (ja)*sin(alpha)),((size*(-sin(alpha))) + (ja*cos(alpha)))
	s1=worldCordinateSystem(ia+d,(((size*cos(alpha)) + (ja)*sin(alpha))+b),(((size*(-sin(alpha))) + (ja*cos(alpha)))+c));	// right fill
	s2=worldCordinateSystem(ia+d,(((size*cos(alpha)) + (j1)*sin(alpha))+b),(((size*(-sin(alpha))) + (j1*cos(alpha)))+c));
	//drawLine(s1.x,s1.y,s2.x,s2.y,GREEN);
	Generatetree(s1.x, s1.y, s2.x , s2.y,GREEN, 2);

	printf("Now Squares\n");

	//int sx0,sx1,sx2,sx3,sy0,sy1,sy2,sy3,sk;


//	int si,sj,ss,c2;
	si = randr(0*paras,15*paras);
	sj = randr(0*paras,15*paras);
	ss = randr(10*paras,15*paras);
	c2 = rand() % 10;

	sx0 = si,sy0 = sj;
	sx1 = sx0 + ss, sy1 = sy0;
	sx2 = sx1, sy2 = sy1 + ss;
	sx3 = sx0, sy3 = sy2;
	//(size,((sx0*cos(alpha)) + (sy0)*sin(alpha)),((sx0*(-sin(alpha))) + (sy0*cos(alpha))));
	s3=worldCordinateSystem(size+d,(((sx0*cos(alpha)) + (sy0)*sin(alpha))+b),(((sx0*(-sin(alpha))) + (sy0*cos(alpha)))+c));	// left fill
	s4=worldCordinateSystem(size+d,(((sx1*cos(alpha)) + (sy1)*sin(alpha))+b),(((sx1*(-sin(alpha))) + (sy1*cos(alpha)))+c));
	s5=worldCordinateSystem(size+d,(((sx2*cos(alpha)) + (sy2)*sin(alpha))+b),(((sx2*(-sin(alpha))) + (sy2*cos(alpha)))+c));
	s6=worldCordinateSystem(size+d,(((sx3*cos(alpha)) + (sy3)*sin(alpha))+b),(((sx3*(-sin(alpha))) + (sy3*cos(alpha)))+c));
	//drawLine(s3.x,s3.y,s4.x,s4.y,GREEN);
	//drawLine(s4.x,s4.y,s5.x,s5.y,GREEN);
	//drawLine(s5.x,s5.y,s6.x,s6.y,GREEN);
	//drawLine(s6.x,s6.y,s3.x,s3.y,GREEN);
	square(s3.x, s3.y, s4.x, s4.y, s5.x, s5.y, s6.x, s6.y, 4,RED);

	}

	printf("ok\n");
	 			//	drawPixel(s1.x,s1.y,0x404040);

	// Generating cube shadow of tilted cube

			struct NVector n;
			struct shadowPoint P0, P1, P2, P3, N1, N2, R0, R1, R2, R3, Fill_Sha, Fill_R;
			P0.x = size+d;
			P0.y = size+b;
			P0.z = size+c;
			P1.x = 0+d;
			P1.y = size+b;
			P1.z = size+c;
			P2.x = 0+d;
			P2.y = 0+b;
			P2.z = size+c;
			P3.x = size+d;
			P3.y = 0+b;
			P3.z = size+c;
			N1.x = P1.x - P0.x;
			N1.y = P1.y - P0.y;
			N1.z = P1.z - P0.z;
			N2.x = P3.x - P0.x;
			N2.y = P3.y - P0.y;
			N2.z = P3.z - P0.z;
			n.x = (N1.y * N2.z) - (N1.z * N2.y);	//X coordinate of the normal
			n.y = (N1.z * N2.x) - (N1.x * N2.z);	//Y coordinate of the normal
			n.z = (N1.x * N2.y) - (N1.y * N2.x);	//Z coordinate of the normal

			float lambdaP0, lambdaP1, lambdaP2, lambdaP3, temp_lambda;
			lambdaP0 = ((n.x * P0.x) + (n.y * P0.y) + (n.z * P0.z))/((n.x * (P0.x-Ps.x)) + (n.y * (P0.y-Ps.y)) + (n.z * (P0.z-Ps.z)));
			lambdaP1 = ((n.x * P1.x) + (n.y * P1.y) + (n.z * P1.z))/((n.x * (P1.x-Ps.x)) + (n.y * (P1.y-Ps.y)) + (n.z * (P1.z-Ps.z)));
			lambdaP2 = ((n.x * P2.x) + (n.y * P2.y) + (n.z * P2.z))/((n.x * (P2.x-Ps.x)) + (n.y * (P2.y-Ps.y)) + (n.z * (P2.z-Ps.z)));
			lambdaP3 = ((n.x * P3.x) + (n.y * P3.y) + (n.z * P3.z))/((n.x * (P3.x-Ps.x)) + (n.y * (P3.y-Ps.y)) + (n.z * (P3.z-Ps.z)));

			// R_i = P_i(xi, yi, zi) + lamda * [ P_s(xs, ys, zs) - P_i(xi, yi, zi) ]
			R0.x = P0.x + lambdaP0 * (Ps.x - P0.x);
			R0.y = P0.y + lambdaP0 * (Ps.y - P0.y);
			R0.z = P0.z + lambdaP0 * (Ps.z - P0.z);
			R1.x = P1.x + lambdaP1 * (Ps.x - P1.x);
			R1.y = P1.y + lambdaP1 * (Ps.y - P1.y);
			R1.z = P1.z + lambdaP1 * (Ps.z - P1.z);
			R2.x = P2.x + lambdaP2 * (Ps.x - P2.x);
			R2.y = P2.y + lambdaP2 * (Ps.y - P2.y);
			R2.z = P2.z + lambdaP2 * (Ps.z - P2.z);
			R3.x = P3.x + lambdaP3 * (Ps.x - P3.x);
			R3.y = P3.y + lambdaP3 * (Ps.y - P3.y);
			R3.z = P3.z + lambdaP3 * (Ps.z - P3.z);


			sss=worldCordinateSystem(Ps.x, Ps.y, Ps.z);
			s1=worldCordinateSystem(R0.x, R0.y, R0.z); // P0
			s2=worldCordinateSystem(R1.x, R1.y, R1.z); // P1
			s3=worldCordinateSystem(R2.x, R2.y, R2.z); // P2
			s4=worldCordinateSystem(R3.x, R3.y, R3.z); // P3

//			drawLine(Ps.x, Ps.y, s2.x, s2.y, BLACK);
//			drawLine(Ps.x, Ps.y, s3.x, s3.y, BLACK);
//			drawLine(Ps.x, Ps.y, s4.x, s4.y, BLACK);
//			drawLine(Ps.x, Ps.y, s1.x, s1.y, BLACK);


//			drawLine(s1.x, s1.y, s2.x, s2.y, BLACK);
//			drawLine(s2.x, s2.y, s3.x, s3.y, BLACK);
//			drawLine(s3.x, s3.y, s4.x, s4.y, BLACK);
//			drawLine(s4.x, s4.y, s1.x, s1.y, BLACK);


			// Filling the shadow
			for (i=0; i<size; i++) {

				for (j=0; j<size; j++) {

					Fill_Sha.x = j+d;
					Fill_Sha.y = (((i*cos(alpha)) + (size)*sin(alpha))+b);   //rotated y = x cos(A) + j sin(A)
					Fill_Sha.z = (((i*(-sin(alpha))) + (size*cos(alpha)))+c); // rotated Z = x (-sin(A)) + j cos(A)
					temp_lambda = ((n.x * Fill_Sha.x) + (n.y * Fill_Sha.y) + (n.z * Fill_Sha.z))/((n.x * (Fill_Sha.x-Ps.x)) + (n.y * (Fill_Sha.y-Ps.y)) + (n.z * (Fill_Sha.z-Ps.z)));
					Fill_R.x = Fill_Sha.x + temp_lambda * (Ps.x - Fill_Sha.x);
					Fill_R.y = Fill_Sha.y + temp_lambda * (Ps.y - Fill_Sha.y);
					Fill_R.z = Fill_Sha.z + temp_lambda * (Ps.z - Fill_Sha.z);
					s_temp = worldCordinateSystem(Fill_R.x, Fill_R.y, Fill_R.z);
					drawPixel(s_temp.x, s_temp.y, BROWN);
				}
			}
}




/******************************************************************************
**   Main Function  main()
******************************************************************************/
int main (void)
{
  //EINTInit();
  uint32_t i, portnum = PORT_NUM;
  portnum = 0 ; /* For LCD use 1 */
  /* SystemClockUpdate() updates the SystemFrequency variable */
//  SystemClockUpdate();
   if ( portnum == 0 )
    SSP0Init();            /* initialize SSP port */
  else if ( portnum == 1 )
    SSP1Init();
  for ( i = 0; i < SSP_BUFSIZE; i++ )
  {
    src_addr[i] = (uint8_t)i;
    dest_addr[i] = 0;
  }
  //initialize LCD
  lcd_init();
  fillrect(0, 0, ST7735_TFTWIDTH, ST7735_TFTHEIGHT, WHITE);
  //drawLine(20, 80, 40, 130, RED);
  //lcddelay(500);

  // Draw a line, get width and colour from user
  int opt;

  srand((unsigned) time(&t));
  int j;
  j = rand() % 10;

	  	fillrect(0, 0, _width, _height, ORANGE);
	  		//fillrect(0, 0, _width, _height*1/4, WHITE);
	  	//	fillrect(0, 0, _width, _height*2/4, WHITE);

	  	//	fillrect(0, 0, _width, _height*3/4, WHITE);
lcddelay(500);
char ch;
uint32_t cnt,v2=65,v1=65,t2=110,t1=150,t3,v3;
int k;
/*
printf("Press Any Key to discontinue\n");
scanf("%c", &ch2);


 if (scanf("%c",&ch2)){

	printf("into the scanf loop\n");
	fillrect(0, 0, _width, _height, BLUE);

}
*/

fillrect(0, 0, _width, _height, LTBLUE);
//lcddelay(1500);
saxis.xe=110;
saxis.ye=110;
saxis.ze=110;
//drawLine(50,70,50,20,BLACK);

double cube_size=30;

s0=worldCordinateSystem(0,0,0);
//printf("value of so.x =%d\n",s0.x);
//printf("value of so.y =%d\n",s0.y);

worldcoordinatAxis();

int co,lo;
for(co=2;co<120;co++)
	{
		for(lo=2;lo<120;lo++)
		{
  			    sos=worldCordinateSystem(co,lo,0);	//top fill
			//	drawPixel(sos.x,sos.y,WHITE);
				drawPixel(sos.x,sos.y,WHITE);
		}
	}


lcddelay(500);

//cube..................1
/*
double size_cube_1 = 22;
s_para= (size_cube_1 / cube_size);
printf("value of s_para = %lf\n",s_para);

generatecubedis(size_cube_1,0,55,30);
cubefill(size_cube_1,0,55,30,s_para);
*/
//
//cube 1 rotating
alpha = 25 * (3.14159265359 / 180);
double size_cube_1 = 22;
Matrix (0,0,size_cube_1,size_cube_1,0,size_cube_1,size_cube_1,size_cube_1,size_cube_1,0,size_cube_1,size_cube_1,0,0,0,size_cube_1,0,0,size_cube_1,size_cube_1,0,0,size_cube_1,0);

s_para= (size_cube_1 / cube_size);
generatecuberot(0,50,35);
cubefillrot(size_cube_1,0,50,35,s_para);


//cube..................2
double size_cube_2 = 25;
s_para= (size_cube_2 / cube_size);
generatecubedis(size_cube_2,50,0,20);
cubefill(size_cube_2,50,0,20,s_para);

//cube 2 rotating
/*
alpha = 35 * (3.14159265359 / 180);
double size_cube_2 = 25;
Matrix (0,0,size_cube_2,size_cube_2,0,size_cube_2,size_cube_2,size_cube_2,size_cube_2,0,size_cube_2,size_cube_2,0,0,0,size_cube_2,0,0,size_cube_2,size_cube_2,0,0,size_cube_2,0);

s_para= (size_cube_2 / cube_size);
generatecuberot(0,0,40);
cubefillrot(size_cube_2,0,0,40,s_para);
*/

//cube..................3
alpha = 15 * (3.14159265359 / 180);
double size_cube_3 = 30;
Matrix (0,0,size_cube_3,size_cube_3,0,size_cube_3,size_cube_3,size_cube_3,size_cube_3,0,size_cube_3,size_cube_3,0,0,0,size_cube_3,0,0,size_cube_3,size_cube_3,0,0,size_cube_3,0);

s_para= (size_cube_3 / cube_size);
generatecuberot(0,0,40);
cubefillrot(size_cube_3,0,0,40,s_para);
//double bj,ij,jj,kk;
printf("value of s_para = %lf\n",s_para);
struct world temps;
temps = worldCordinateSystem(Ps.x, Ps.y, Ps.z);
//printf("Ps(x, y): (%lf, %lf)\n", temps.x, temps.y);

//generatecubetilt(cube_size,0,20,0);
//double s;
//s=Matrix(doublex1, double y1,double z1,double x2, double y2,double z2,double x3, double y3,double z3,double x4, double y4,double z4,double x5, double y5,double z5,double x6, double y6,double z6,double x7, double y7,double z7,double x7, double y7,double z7);

  return 0;
}


/******************************************************************************
**                            End Of File
******************************************************************************/





