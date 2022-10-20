/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "fatfs_sd.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TX_TIMEOUT          ((uint32_t)100)
#define bool BYTE
#define TRUE  1
#define FALSE 0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
volatile uint16_t LCD_HEIGHT = ILI9341_SCREEN_HEIGHT;
volatile uint16_t LCD_WIDTH	 = ILI9341_SCREEN_WIDTH;

extern volatile uint16_t Timer1, Timer2;					/* 1ms Timer Counter */

static volatile DSTATUS Stat = STA_NOINIT;	/* Disk Status */
static uint8_t CardType;                    /* Type 0:MMC, 1:SDC, 2:Block addressing */
static uint8_t PowerFlag = 0;				/* Power flag */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_RNG_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*Send data (char) to LCD*/
void ILI9341_SPI_Send(unsigned char SPI_Data)
{
HAL_SPI_Transmit(&hspi1, &SPI_Data, 1, 1);
}

/* Send command (char) to LCD */
void ILI9341_Write_Command(uint8_t Command)
{
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
ILI9341_SPI_Send(Command);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/* Send Data (char) to LCD */
void ILI9341_Write_Data(uint8_t Data)
{
HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
ILI9341_SPI_Send(Data);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/* Set Address - Location block - to draw into */
void ILI9341_SetWindow(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2)
{
	LCD_WR_REG(0x2A);
	LCD_WR_DATA(X1>>8);
	LCD_WR_DATA(0xFF &X1);
	LCD_WR_DATA(X2>>8);
	LCD_WR_DATA(0xFF &X2);

    LCD_WR_REG(0x2B);
    LCD_WR_DATA(Y1>>8);
    LCD_WR_DATA(0xFF &Y1);
    LCD_WR_DATA(Y2>>8);
    LCD_WR_DATA(0xFF &Y2);

    LCD_WR_REG(0x2C);
}

/*HARDWARE RESET*/
void ILI9341_Reset(void)
{
HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
HAL_Delay(200);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
HAL_Delay(200);
HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
}

/*See rotation of the screen - changes x0 and y0*/
void ILI9341_Set_Rotation(uint8_t Rotation)
{

uint8_t screen_rotation = Rotation;

ILI9341_Write_Command(0x36);
HAL_Delay(1);

switch(screen_rotation)
	{
		case SCREEN_VERTICAL_1:
			ILI9341_Write_Data(0x40|0x08);
			LCD_WIDTH = 240;
			LCD_HEIGHT = 320;
			break;
		case SCREEN_HORIZONTAL_1:
			ILI9341_Write_Data(0x20|0x08);
			LCD_WIDTH  = 320;
			LCD_HEIGHT = 240;
			break;
		case SCREEN_VERTICAL_2:
			ILI9341_Write_Data(0x80|0x08);
			LCD_WIDTH  = 240;
			LCD_HEIGHT = 320;
			break;
		case SCREEN_HORIZONTAL_2:
			ILI9341_Write_Data(0x40|0x80|0x20|0x08);
			LCD_WIDTH  = 320;
			LCD_HEIGHT = 240;
			break;
		default:
			//EXIT IF SCREEN ROTATION NOT VALID!
			break;
	}
}

/*Enable LCD display*/
void ILI9341_Enable(void)
{
HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
}

typedef enum {
	ROTATE_0,
	ROTATE_90,
	ROTATE_180,
	ROTATE_270
} LCD_Horizontal_t;

static void RESET_L(void)
{
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
}

static void RESET_H(void)
{
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
}

static void CS_L(void)
{
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}

static void CS_H(void)
{
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

static void DC_L(void)
{
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
}

static void DC_H(void)
{
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
}

static void LED_H(void)
{
	HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_SET);
}



void ILI9341_SoftReset(void)
{
	uint8_t cmd;
	cmd = 0x01; //Software reset
	DC_L();
	if (HAL_SPI_Transmit(&hspi1, &cmd, 1, 1000) != HAL_OK) {
		Error_Handler();
	}
}

void LCD_WR_REG(uint8_t data)
{
	CS_L();
	DC_L();
	if (HAL_SPI_Transmit(&hspi1, &data, 1, 1000) != HAL_OK) {
		Error_Handler();
	}
}

void LCD_WR_DATA(uint8_t data)
{
	DC_H();
	CS_L();
	if (HAL_SPI_Transmit(&hspi1, &data, 1, 1000) != HAL_OK) {
		Error_Handler();
	}
	CS_H();
}


static void LCD_direction(LCD_Horizontal_t direction)
{
	switch (direction) {
	case ROTATE_0:
		LCD_WR_REG(0x36);
		LCD_WR_DATA(0x48);
		break;
	case ROTATE_90:
		LCD_WR_REG(0x36);
		LCD_WR_DATA(0x28);
		break;
	case ROTATE_180:
		LCD_WR_REG(0x36);
		LCD_WR_DATA(0x88);
		break;
	case ROTATE_270:
		LCD_WR_REG(0x36);
		LCD_WR_DATA(0xE8);
		break;
	}
}

/*Initialize LCD display*/
void ILI9341_Init(void)
{
	ILI9341_Reset();
	ILI9341_SoftReset();

	/* Power Control A */
	LCD_WR_REG(0xCB);
	LCD_WR_DATA(0x39);
	LCD_WR_DATA(0x2C);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x34);
	LCD_WR_DATA(0x02);
	/* Power Control B */
	LCD_WR_REG(0xCF);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xC1);
	LCD_WR_DATA(0x30);
	/* Driver timing control A */
	LCD_WR_REG(0xE8);
	LCD_WR_DATA(0x85);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x78);
	/* Driver timing control B */
	LCD_WR_REG(0xEA);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	/* Power on Sequence control */
	LCD_WR_REG(0xED);
	LCD_WR_DATA(0x64);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0x12);
	LCD_WR_DATA(0x81);
	/* Pump ratio control */
	LCD_WR_REG(0xF7);
	LCD_WR_DATA(0x20);
	/* Power Control 1 */
	LCD_WR_REG(0xC0);
	LCD_WR_DATA(0x10);
	/* Power Control 2 */
	LCD_WR_REG(0xC1);
	LCD_WR_DATA(0x10);
	/* VCOM Control 1 */
	LCD_WR_REG(0xC5);
	LCD_WR_DATA(0x3E);
	LCD_WR_DATA(0x28);
	/* VCOM Control 2 */
	LCD_WR_REG(0xC7);
	LCD_WR_DATA(0x86);
	/* VCOM Control 2 */
	LCD_WR_REG(0x36);
	LCD_WR_DATA(0x48);
	/* Pixel Format Set */
	LCD_WR_REG(0x3A);
	LCD_WR_DATA(0x55);    //16bit
	LCD_WR_REG(0xB1);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x18);
#if 0
	// Little Endian for TouchGFX
	LCD_WR_REG(0xF6);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x20); // Little Endian
#endif
	/* Display Function Control */
	LCD_WR_REG(0xB6);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x82);
	LCD_WR_DATA(0x27);
	/* 3GAMMA FUNCTION DISABLE */
	LCD_WR_REG(0xF2);
	LCD_WR_DATA(0x00);
	/* GAMMA CURVE SELECTED */
	LCD_WR_REG(0x26); //Gamma set
	LCD_WR_DATA(0x01); 	//Gamma Curve (G2.2)
	//Positive Gamma  Correction
	LCD_WR_REG(0xE0);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x31);
	LCD_WR_DATA(0x2B);
	LCD_WR_DATA(0x0C);
	LCD_WR_DATA(0x0E);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x4E);
	LCD_WR_DATA(0xF1);
	LCD_WR_DATA(0x37);
	LCD_WR_DATA(0x07);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0x0E);
	LCD_WR_DATA(0x09);
	LCD_WR_DATA(0x00);
	//Negative Gamma  Correction
	LCD_WR_REG(0xE1);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x0E);
	LCD_WR_DATA(0x14);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0x11);
	LCD_WR_DATA(0x07);
	LCD_WR_DATA(0x31);
	LCD_WR_DATA(0xC1);
	LCD_WR_DATA(0x48);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x0C);
	LCD_WR_DATA(0x31);
	LCD_WR_DATA(0x36);
	LCD_WR_DATA(0x0F);
	//EXIT SLEEP
	LCD_WR_REG(0x11);

	HAL_Delay(120);

	//TURN ON DISPLAY
	LCD_WR_REG(0x29);
	LCD_WR_DATA(0x2C);

	LCD_direction(ROTATE_270);

}

//INTERNAL FUNCTION OF LIBRARY, USAGE NOT RECOMENDED, USE Draw_Pixel INSTEAD
/*Sends single pixel colour information to LCD*/
void ILI9341_Draw_Colour(uint16_t Colour)
{
//SENDS COLOUR
unsigned char TempBuffer[2] = {Colour>>8, Colour};
HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
HAL_SPI_Transmit(&hspi1, TempBuffer, 2, 1);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

//INTERNAL FUNCTION OF LIBRARY
/*Sends block colour information to LCD*/
void ILI9341_Draw_Colour_Burst(uint16_t Colour, uint32_t Size)
{
//SENDS COLOUR
uint32_t Buffer_Size = 0;
if((Size*2) < BURST_MAX_SIZE)
{
	Buffer_Size = Size;
}
else
{
	Buffer_Size = BURST_MAX_SIZE;
}

HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

unsigned char chifted = 	Colour>>8;;
unsigned char burst_buffer[Buffer_Size];
for(uint32_t j = 0; j < Buffer_Size; j+=2)
	{
		burst_buffer[j] = 	chifted;
		burst_buffer[j+1] = Colour;
	}

uint32_t Sending_Size = Size*2;
uint32_t Sending_in_Block = Sending_Size/Buffer_Size;
uint32_t Remainder_from_block = Sending_Size%Buffer_Size;

if(Sending_in_Block != 0)
{
	for(uint32_t j = 0; j < (Sending_in_Block); j++)
		{
		HAL_SPI_Transmit(&hspi1, (unsigned char *)burst_buffer, Buffer_Size, 10);
		}
}

//REMAINDER!
HAL_SPI_Transmit(&hspi1, (unsigned char *)burst_buffer, Remainder_from_block, 10);

HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

//FILL THE ENTIRE SCREEN WITH SELECTED COLOUR (either #define-d ones or custom 16bit)
/*Sets address (entire screen) and Sends Height*Width ammount of colour information to LCD*/
void ILI9341_Fill_Screen(uint16_t Colour)
{
ILI9341_SetWindow(0,0,LCD_WIDTH,LCD_HEIGHT);
ILI9341_Draw_Colour_Burst(Colour, LCD_WIDTH*LCD_HEIGHT);
}

//DRAW PIXEL AT XY POSITION WITH SELECTED COLOUR
//
//Location is dependant on screen orientation. x0 and y0 locations change with orientations.
//Using pixels to draw big simple structures is not recommended as it is really slow
//Try using either rectangles or lines if possible
//
void ILI9341_Draw_Pixel(uint16_t X,uint16_t Y,uint16_t Colour)
{
if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) return;	//OUT OF BOUNDS!

//ADDRESS
HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
ILI9341_SPI_Send(0x2A);
HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

//XDATA
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
unsigned char Temp_Buffer[4] = {X>>8,X, (X+1)>>8, (X+1)};
HAL_SPI_Transmit(&hspi1, Temp_Buffer, 4, 1);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

//ADDRESS
HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
ILI9341_SPI_Send(0x2B);
HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

//YDATA
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
unsigned char Temp_Buffer1[4] = {Y>>8,Y, (Y+1)>>8, (Y+1)};
HAL_SPI_Transmit(&hspi1, Temp_Buffer1, 4, 1);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

//ADDRESS
HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
ILI9341_SPI_Send(0x2C);
HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

//COLOUR
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
unsigned char Temp_Buffer2[2] = {Colour>>8, Colour};
HAL_SPI_Transmit(&hspi1, Temp_Buffer2, 2, 1);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

}


//DRAW LINE FROM X,Y LOCATION to X+Width,Y LOCATION
void ILI9341_Draw_Horizontal_Line(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Colour)
{
if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) return;
if((X+Width-1)>=LCD_WIDTH)
	{
		Width=LCD_WIDTH-X;
	}
ILI9341_SetWindow(X, Y, X+Width-1, Y);
ILI9341_Draw_Colour_Burst(Colour, Width);
}

//DRAW LINE FROM X,Y LOCATION to X,Y+Height LOCATION
void ILI9341_Draw_Vertical_Line(uint16_t X, uint16_t Y, uint16_t Height, uint16_t Colour)
{
if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) return;
if((Y+Height-1)>=LCD_HEIGHT)
	{
		Height=LCD_HEIGHT-Y;
	}
ILI9341_SetWindow(X, Y, X, Y+Height-1);
ILI9341_Draw_Colour_Burst(Colour, Height);
}



void ILI9341_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                            uint16_t color) {
	 int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	  if (steep) {
	 //   int16_t x0, y0;
	   // swap_int16_t(x1, y1);

	    int16_t newx0 = y0;
	    int16_t newx1 = y1;
	    int16_t newy0 = x0;
	    int16_t newy1 = x1;

	    x0 = newx0;
	    y0 = newy0;
	    x1 = newx1;
		y1 = newy1;
	  }

	  if (x0 > x1) {
	//    _swap_int16_t(x0, x1);
	 //   _swap_int16_t(y0, y1);

	    int16_t newx0 = x1;
	    int16_t newx1 = x0;
	    int16_t newy0 = y1;
	    int16_t newy1 = y0;

	    x0 = newx0;
		x1 = newx1;
		y0 = newy0;
		y1 = newy1;

	  }

	  int16_t dx, dy;
	  dx = x1 - x0;
	  dy = abs(y1 - y0);

	  int16_t err = dx / 2;
	  int16_t ystep;

	  if (y0 < y1) {
	    ystep = 1;
	  } else {
	    ystep = -1;
	  }

	  for (; x0 <= x1; x0++) {
	    if (steep) {
	    	ILI9341_Draw_Pixel(y0, x0, color);
	    } else {
	    	ILI9341_Draw_Pixel(x0, y0, color);
	    }
	    err -= dy;
	    if (err < 0) {
	      y0 += ystep;
	      err += dx;
	    }
	  }
}


void ILI9341_Draw_Hollow_Circle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Colour)
{
	int x = Radius-1;
    int y = 0;
    int dx = 1;
    int dy = 1;
    int err = dx - (Radius << 1);

    while (x >= y)
    {
        ILI9341_Draw_Pixel(X + x, Y + y, Colour);
        ILI9341_Draw_Pixel(X + y, Y + x, Colour);
        ILI9341_Draw_Pixel(X - y, Y + x, Colour);
        ILI9341_Draw_Pixel(X - x, Y + y, Colour);
        ILI9341_Draw_Pixel(X - x, Y - y, Colour);
        ILI9341_Draw_Pixel(X - y, Y - x, Colour);
        ILI9341_Draw_Pixel(X + y, Y - x, Colour);
        ILI9341_Draw_Pixel(X + x, Y - y, Colour);

        if (err <= 0)
        {
            y++;
            err += dy;
            dy += 2;
        }
        if (err > 0)
        {
            x--;
            dx += 2;
            err += (-Radius << 1) + dx;
        }
    }
}

/*Draw filled circle at X,Y location with specified radius and colour. X and Y represent circles center */
void ILI9341_Draw_Filled_Circle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Colour)
{

		int x = Radius;
    int y = 0;
    int xChange = 1 - (Radius << 1);
    int yChange = 0;
    int radiusError = 0;

    while (x >= y)
    {
        for (int i = X - x; i <= X + x; i++)
        {
            ILI9341_Draw_Pixel(i, Y + y,Colour);
            ILI9341_Draw_Pixel(i, Y - y,Colour);
        }
        for (int i = X - y; i <= X + y; i++)
        {
            ILI9341_Draw_Pixel(i, Y + x,Colour);
            ILI9341_Draw_Pixel(i, Y - x,Colour);
        }

        y++;
        radiusError += yChange;
        yChange += 2;
        if (((radiusError << 1) + xChange) > 0)
        {
            x--;
            radiusError += xChange;
            xChange += 2;
        }
    }
		//Really slow implementation, will require future overhaul
		//TODO:	https://stackoverflow.com/questions/1201200/fast-algorithm-for-drawing-filled-circles
}


void ILI9341_Draw_Rectangle(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint16_t Colour)
{
if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) return;
if((X+Width-1)>=LCD_WIDTH)
	{
		Width=LCD_WIDTH-X;
	}
if((Y+Height-1)>=LCD_HEIGHT)
	{
		Height=LCD_HEIGHT-Y;
	}
ILI9341_SetWindow(X, Y, X+Width-1, Y+Height-1);
ILI9341_Draw_Colour_Burst(Colour, Height*Width);
}




void ILI9341_Draw_Hollow_Rectangle_Coord(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Colour)
{
	uint16_t 	X_length = 0;
	uint16_t 	Y_length = 0;
	uint8_t		Negative_X = 0;
	uint8_t 	Negative_Y = 0;
	float 		Calc_Negative = 0;

	Calc_Negative = X1 - X0;
	if(Calc_Negative < 0) Negative_X = 1;
	Calc_Negative = 0;

	Calc_Negative = Y1 - Y0;
	if(Calc_Negative < 0) Negative_Y = 1;


	//DRAW HORIZONTAL!
	if(!Negative_X)
	{
		X_length = X1 - X0;
	}
	else
	{
		X_length = X0 - X1;
	}
	ILI9341_Draw_Horizontal_Line(X0, Y0, X_length, Colour);
	ILI9341_Draw_Horizontal_Line(X0, Y1, X_length, Colour);



	//DRAW VERTICAL!
	if(!Negative_Y)
	{
		Y_length = Y1 - Y0;
	}
	else
	{
		Y_length = Y0 - Y1;
	}
	ILI9341_Draw_Vertical_Line(X0, Y0, Y_length, Colour);
	ILI9341_Draw_Vertical_Line(X1, Y0, Y_length, Colour);

	if((X_length > 0)||(Y_length > 0))
	{
		ILI9341_Draw_Pixel(X1, Y1, Colour);
	}

}


static void ConvHL(uint8_t *s, int32_t l)
{
	uint8_t v;
	while (l > 0) {
		v = *(s+1);
		*(s+1) = *s;
		*s = v;
		s += 2;
		l -= 2;
	}
}

/*Draw a filled rectangle between positions X0,Y0 and X1,Y1 with specified colour*/
void ILI9341_Draw_Filled_Rectangle_Coord(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Colour)
{
	uint16_t 	X_length = 0;
	uint16_t 	Y_length = 0;
	uint8_t		Negative_X = 0;
	uint8_t 	Negative_Y = 0;
	int32_t 	Calc_Negative = 0;

	uint16_t X0_true = 0;
	uint16_t Y0_true = 0;

	Calc_Negative = X1 - X0;
	if(Calc_Negative < 0) Negative_X = 1;
	Calc_Negative = 0;

	Calc_Negative = Y1 - Y0;
	if(Calc_Negative < 0) Negative_Y = 1;


	//DRAW HORIZONTAL!
	if(!Negative_X)
	{
		X_length = X1 - X0;
		X0_true = X0;
	}
	else
	{
		X_length = X0 - X1;
		X0_true = X1;
	}

	//DRAW VERTICAL!
	if(!Negative_Y)
	{
		Y_length = Y1 - Y0;
		Y0_true = Y0;
	}
	else
	{
		Y_length = Y0 - Y1;
		Y0_true = Y1;
	}

	ILI9341_Draw_Rectangle(X0_true, Y0_true, X_length, Y_length, Colour);
}



void ILI9341_Draw_Triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                                int16_t x2, int16_t y2, uint16_t color) {
	ILI9341_drawLine(x0, y0, x1, y1, color);
	ILI9341_drawLine(x1, y1, x2, y2, color);
	ILI9341_drawLine(x2, y2, x0, y0, color);


	 int16_t a, b, y, last;

	  // Sort coordinates by Y order (y2 >= y1 >= y0)
	  if (y0 > y1) {


	        int16_t newx0 = x1;
	   	    int16_t newx1 = x0;
	   	    int16_t newy0 = y1;
	   	    int16_t newy1 = y0;

	   	    x0 = newx0;
	   	    y0 = newy0;
	   	    x1 = newx1;
	   		y1 = newy1;


	  }
	  if (y1 > y2) {


	    int16_t newx1 = x2;
	    int16_t newx2 = x1;
	    int16_t newy1 = y2;
	    int16_t newy2 = y1;

	    	   	    x1 = newx1;
	    	   	    y1 = newy1;
	    	   	    x2 = newx2;
	    	   		y2 = newy2;
	  }
	  if (y0 > y1) {
		  int16_t newx0 = x1;
		 	   	    int16_t newx1 = x0;
		 	   	    int16_t newy0 = y1;
		 	   	    int16_t newy1 = y0;

		 	   	    x0 = newx0;
		 	   	    y0 = newy0;
		 	   	    x1 = newx1;
		 	   		y1 = newy1;
	  }


	  if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
	    a = b = x0;
	    if (x1 < a)
	      a = x1;
	    else if (x1 > b)
	      b = x1;
	    if (x2 < a)
	      a = x2;
	    else if (x2 > b)
	      b = x2;
	    ILI9341_Draw_Horizontal_Line(a, y0, b - a + 1, CYAN);
	   // endWrite();
	    return;
	  }

	  int16_t dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0,
	          dx12 = x2 - x1, dy12 = y2 - y1;
	  int32_t sa = 0, sb = 0;

	  // For upper part of triangle, find scanline crossings for segments
	  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
	  // is included here (and second loop will be skipped, avoiding a /0
	  // error there), otherwise scanline y1 is skipped here and handled
	  // in the second loop...which also avoids a /0 error here if y0=y1
	  // (flat-topped triangle).
	  if (y1 == y2)
	    last = y1; // Include y1 scanline
	  else
	    last = y1 - 1; // Skip it

	  for (y = y0; y <= last; y++) {
	    a = x0 + sa / dy01;
	    b = x0 + sb / dy02;
	    sa += dx01;
	    sb += dx02;
	    /* longhand:
	    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
	    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
	    */
	    if (a > b)
	    {
	    int16_t newA = b;
	    int16_t newB = a;

	    a = newA;
	    b = newB;
	    }
	    ILI9341_Draw_Horizontal_Line(a, y, b - a + 1, CYAN);
	  }

	  // For lower part of triangle, find scanline crossings for segments
	  // 0-2 and 1-2.  This loop is skipped if y1=y2.
	  sa = (int32_t)dx12 * (y - y1);
	  sb = (int32_t)dx02 * (y - y0);
	  for (; y <= y2; y++) {
	    a = x1 + sa / dy12;
	    b = x0 + sb / dy02;
	    sa += dx12;
	    sb += dx02;
	    /* longhand:
	    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
	    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
	    */
	    if (a > b)
	    {
	    	    int16_t newA = b;
	    	    int16_t newB = a;

	    	    a = newA;
	    	    b = newB;
	    	    }
	    ILI9341_Draw_Horizontal_Line(a, y, b - a + 1, CYAN);
	  }
	//  endWrite();

}


/*Draws a character (fonts imported from fonts.h) at X,Y location with specified font colour, size and Background colour*/
/*See fonts.h implementation of font on what is required for changing to a different font when switching fonts libraries*/
void ILI9341_Draw_Char(char Character, uint8_t X, uint8_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
		uint8_t 	function_char;
    uint8_t 	i,j;

		function_char = Character;

    if (function_char < ' ') {
        Character = 0;
    } else {
        function_char -= 32;
		}

		char temp[CHAR_WIDTH];
		for(uint8_t k = 0; k<CHAR_WIDTH; k++)
		{
		temp[k] = font[function_char][k];
		}

    // Draw pixels
		ILI9341_Draw_Rectangle(X, Y, CHAR_WIDTH*Size, CHAR_HEIGHT*Size, Background_Colour);
    for (j=0; j<CHAR_WIDTH; j++) {
        for (i=0; i<CHAR_HEIGHT; i++) {
            if (temp[j] & (1<<i)) {
							if(Size == 1)
							{
              ILI9341_Draw_Pixel(X+j, Y+i, Colour);
							}
							else
							{
							ILI9341_Draw_Rectangle(X+(j*Size), Y+(i*Size), Size, Size, Colour);
							}
            }
        }
    }
}

/*Draws an array of characters (fonts imported from fonts.h) at X,Y location with specified font colour, size and Background colour*/
/*See fonts.h implementation of font on what is required for changing to a different font when switching fonts libraries*/
void ILI9341_Draw_Text(const char* Text, uint8_t X, uint8_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
    while (*Text) {
        ILI9341_Draw_Char(*Text++, X, Y, Colour, Size, Background_Colour);
        X += CHAR_WIDTH*Size;
    }
}


void ILI9341_Draw_Image(const char* Image_Array, uint8_t Orientation)
{


		ILI9341_Set_Rotation(SCREEN_VERTICAL_2);
		ILI9341_SetWindow(0,0,ILI9341_SCREEN_HEIGHT,ILI9341_SCREEN_WIDTH);

		HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

		unsigned char Temp_small_buffer[BURST_MAX_SIZE];
		uint32_t counter = 0;
		for(uint32_t i = 0; i < ILI9341_SCREEN_WIDTH*ILI9341_SCREEN_HEIGHT*2/BURST_MAX_SIZE; i++)
		{
				for(uint32_t k = 0; k< BURST_MAX_SIZE; k++)
				{
					Temp_small_buffer[k]	= Image_Array[counter+k];
				}
				HAL_SPI_Transmit(&hspi1, (unsigned char*)Temp_small_buffer, BURST_MAX_SIZE, 10);
				counter += BURST_MAX_SIZE;
		}
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}




void Test_Lights_and_Buttons() {

	   int n = 0;
	   int8_t switch2 ;

	   switch2 =  HAL_GPIO_ReadPin (SWITCH_2_GPIO_Port, SWITCH_2_Pin);
	   n = switch2;

	   if (n != 0)

	   {
	   HAL_GPIO_WritePin(LED_CHG_G_GPIO_Port, LED_CHG_G_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(LED_CHG_O_GPIO_Port, LED_CHG_O_Pin, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, GPIO_PIN_RESET);
	   }


	   if (n == 0)

	   {
	   HAL_GPIO_WritePin(LED_CHG_G_GPIO_Port, LED_CHG_G_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(LED_CHG_O_GPIO_Port, LED_CHG_O_Pin, GPIO_PIN_SET);
	   HAL_GPIO_WritePin(LED_PWR_GPIO_Port, LED_PWR_Pin, GPIO_PIN_SET);
	   }
}


void Test_Touch() {

	  int n = 0;
	  int n2 = 0;
	  int8_t switch1 ;
	  int8_t switch2 ;
	  volatile uint16_t leftX = 50;
	  volatile uint16_t rightX = 190;
	  volatile uint16_t topY = 250;
	  volatile uint16_t bottomY = 300;

	  switch1 =  HAL_GPIO_ReadPin (T_IRQ_GPIO_Port, T_IRQ_Pin);
	  n = switch1;

	  if (n != 0)

	       {

		  HAL_Delay(50);
		  switch2 =  HAL_GPIO_ReadPin (T_IRQ_GPIO_Port, T_IRQ_Pin);
		  n2 = switch2;
		  if (n2 >> 0)
		       {
			   ILI9341_Draw_Filled_Rectangle_Coord(leftX,topY,rightX,bottomY,BLUE);
		       }
		   }



}

void Test_Buzzer() {

				  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
				  HAL_Delay(10);
				  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
				  HAL_Delay(10);

}


void Test_Screen_Visual() {

	ILI9341_Draw_Image((const char*)pro_X, SCREEN_VERTICAL_2);

}



FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count
/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

#define BUFFER_SIZE 75243
char buffer[BUFFER_SIZE];  // to store strings..

int i=0;

int bufsize (char *buf)
{
	int i=0;
	while (*buf++ != '\0') i++;
	return i;
}

void clear_buffer (void)
{
	for (int i=0; i<BUFFER_SIZE; i++) buffer[i] = '\0';
}


/***************************************
 * SPI functions
 **************************************/

/* slave select */
static void SELECT(void)
{
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
}

/* slave deselect */
static void DESELECT(void)
{
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}

/* SPI transmit a byte */
static void SPI_TxByte(uint8_t data)
{
	while(!__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_TXE));
	HAL_SPI_Transmit(&hspi3, &data, 1, SPI_TIMEOUT);
}

/* SPI transmit buffer */
static void SPI_TxBuffer(uint8_t *buffer, uint16_t len)
{
	while(!__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_TXE));
	HAL_SPI_Transmit(&hspi3, buffer, len, SPI_TIMEOUT);
}

/* SPI receive a byte */
static uint8_t SPI_RxByte(void)
{
	uint8_t dummy, data;
	dummy = 0xFF;

	while(!__HAL_SPI_GET_FLAG(&hspi3, SPI_FLAG_TXE));
	HAL_SPI_TransmitReceive(&hspi3, &dummy, &data, 1, SPI_TIMEOUT);

	return data;
}

/* SPI receive a byte via pointer */
static void SPI_RxBytePtr(uint8_t *buff)
{
	*buff = SPI_RxByte();
}

/***************************************
 * SD functions
 **************************************/

/* wait SD ready */
static uint8_t SD_ReadyWait(void)
{
	uint8_t res;

	/* timeout 500ms */
	Timer2 = 500;

	/* if SD goes ready, receives 0xFF */
	do {
		res = SPI_RxByte();
	} while ((res != 0xFF) && Timer2);

	return res;
}

/* power on */
static void SD_PowerOn(void)
{
	uint8_t args[6];
	uint32_t cnt = 0x1FFF;

	/* transmit bytes to wake up */
	DESELECT();
	for(int i = 0; i < 10; i++)
	{
		SPI_TxByte(0xFF);
	}

	/* slave select */
	SELECT();

	/* make idle state */
	args[0] = CMD0;		/* CMD0:GO_IDLE_STATE */
	args[1] = 0;
	args[2] = 0;
	args[3] = 0;
	args[4] = 0;
	args[5] = 0x95;		/* CRC */

	SPI_TxBuffer(args, sizeof(args));

	/* wait response */
	while ((SPI_RxByte() != 0x01) && cnt)
	{
		cnt--;
	}

	DESELECT();
	SPI_TxByte(0XFF);

	PowerFlag = 1;
}

/* power off */
static void SD_PowerOff(void)
{
	PowerFlag = 0;
}

/* check power flag */
static uint8_t SD_CheckPower(void)
{
	return PowerFlag;
}

/* receive data block */
static bool SD_RxDataBlock(BYTE *buff, UINT len)
{
	uint8_t token;

	/* timeout 200ms */
	Timer1 = 200;

	/* loop until receive a response or timeout */
	do {
		token = SPI_RxByte();
	} while((token == 0xFF) && Timer1);

	/* invalid response */
	if(token != 0xFE) return FALSE;

	/* receive data */
	do {
		SPI_RxBytePtr(buff++);
	} while(len--);

	/* discard CRC */
	SPI_RxByte();
	SPI_RxByte();

	return TRUE;
}

/* transmit data block */
#if _USE_WRITE == 1
static bool SD_TxDataBlock(const uint8_t *buff, BYTE token)
{
	uint8_t resp;
	uint8_t i = 0;

	/* wait SD ready */
	if (SD_ReadyWait() != 0xFF) return FALSE;

	/* transmit token */
	SPI_TxByte(token);

	/* if it's not STOP token, transmit data */
	if (token != 0xFD)
	{
		SPI_TxBuffer((uint8_t*)buff, 512);

		/* discard CRC */
		SPI_RxByte();
		SPI_RxByte();

		/* receive response */
		while (i <= 64)
		{
			resp = SPI_RxByte();

			/* transmit 0x05 accepted */
			if ((resp & 0x1F) == 0x05) break;
			i++;
		}

		/* recv buffer clear */
		while (SPI_RxByte() == 0);
	}

	/* transmit 0x05 accepted */
	if ((resp & 0x1F) == 0x05) return TRUE;

	return FALSE;
}
#endif /* _USE_WRITE */

/* transmit command */
static BYTE SD_SendCmd(BYTE cmd, uint32_t arg)
{
	uint8_t crc, res;

	/* wait SD ready */
	if (SD_ReadyWait() != 0xFF) return 0xFF;

	/* transmit command */
	SPI_TxByte(cmd); 					/* Command */
	SPI_TxByte((uint8_t)(arg >> 24)); 	/* Argument[31..24] */
	SPI_TxByte((uint8_t)(arg >> 16)); 	/* Argument[23..16] */
	SPI_TxByte((uint8_t)(arg >> 8)); 	/* Argument[15..8] */
	SPI_TxByte((uint8_t)arg); 			/* Argument[7..0] */

	/* prepare CRC */
	if(cmd == CMD0) crc = 0x95;	/* CRC for CMD0(0) */
	else if(cmd == CMD8) crc = 0x87;	/* CRC for CMD8(0x1AA) */
	else crc = 1;

	/* transmit CRC */
	SPI_TxByte(crc);

	/* Skip a stuff byte when STOP_TRANSMISSION */
	if (cmd == CMD12) SPI_RxByte();

	/* receive response */
	uint8_t n = 10;
	do {
		res = SPI_RxByte();
	} while ((res & 0x80) && --n);

	return res;
}

/***************************************
 * user_diskio.c functions
 **************************************/

/* initialize SD */
DSTATUS SD_disk_initialize(BYTE drv)
{
	uint8_t n, type, ocr[4];

	/* single drive, drv should be 0 */
	if(drv) return STA_NOINIT;

	/* no disk */
	if(Stat & STA_NODISK) return Stat;

	/* power on */
	SD_PowerOn();

	/* slave select */
	SELECT();

	/* check disk type */
	type = 0;
	//HAL_Delay(2000);
	/* send GO_IDLE_STATE command */
	if (SD_SendCmd(CMD0, 0) == 1)
	{
		/* timeout 1 sec */
		Timer1 = 1000;
		//HAL_Delay(2000);

		/* SDC V2+ accept CMD8 command, http://elm-chan.org/docs/mmc/mmc_e.html */
		if (SD_SendCmd(CMD8, 0x1AA) == 1)
		{
			/* operation condition register */
			for (n = 0; n < 4; n++)
			{
				ocr[n] = SPI_RxByte();
			}
			/* voltage range 2.7-3.6V */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA)
			{
				//HAL_Delay(2000);
				/* ACMD41 with HCS bit */
				do {
					if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 1UL << 30) == 0) break;
				} while (Timer1);

				/* READ_OCR */
				if (Timer1 && SD_SendCmd(CMD58, 0) == 0)
				{
					/* Check CCS bit */
					for (n = 0; n < 4; n++)
					{
						ocr[n] = SPI_RxByte();
					}

					/* SDv2 (HC or SC) */
					type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
				}
			}
		}
		else
		{
			/* SDC V1 or MMC */
			type = (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) <= 1) ? CT_SD1 : CT_MMC;

			do
			{
				if (type == CT_SD1)
				{
					if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) == 0) break; /* ACMD41 */
				}
				else
				{
					if (SD_SendCmd(CMD1, 0) == 0) break; /* CMD1 */
				}

			} while (Timer1);

			/* SET_BLOCKLEN */
			if (!Timer1 || SD_SendCmd(CMD16, 512) != 0) type = 0;
		}
	}
	CardType = type;

	/* Idle */
	DESELECT();
	SPI_RxByte();

	/* Clear STA_NOINIT */
	if (type)
	{
		Stat &= ~STA_NOINIT;
	}
	else
	{
		/* Initialization failed */
		SD_PowerOff();
	}

	return Stat;
}

/* return disk status */
DSTATUS SD_disk_status(BYTE drv)
{
	if (drv) return STA_NOINIT;
	return Stat;
}

/* read sector */
DRESULT SD_disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count)
{
	/* pdrv should be 0 */
	if (pdrv || !count) return RES_PARERR;

	/* no disk */
	if (Stat & STA_NOINIT) return RES_NOTRDY;

	/* convert to byte address */
	if (!(CardType & CT_SD2)) sector *= 512;

	SELECT();

	if (count == 1)
	{
		//HAL_Delay(8000);
		/* READ_SINGLE_BLOCK */
		if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, 512)) count = 0;
	}
	else
	{
		/* READ_MULTIPLE_BLOCK */
		if (SD_SendCmd(CMD18, sector) == 0)
		{
			do {
				if (!SD_RxDataBlock(buff, 512)) break;
				buff += 512;
			} while (--count);

			/* STOP_TRANSMISSION */
			SD_SendCmd(CMD12, 0);
		}
	}
	/* Idle */
	DESELECT();
	SPI_RxByte();

	return count ? RES_ERROR : RES_OK;
}

/* write sector */
#if _USE_WRITE == 1
DRESULT SD_disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count)
{
	/* pdrv should be 0 */
	if (pdrv || !count) return RES_PARERR;

	/* no disk */
	if (Stat & STA_NOINIT) return RES_NOTRDY;

	/* write protection */
	if (Stat & STA_PROTECT) return RES_WRPRT;

	/* convert to byte address */
	if (!(CardType & CT_SD2)) sector *= 512;

	SELECT();

	if (count == 1)
	{
		/* WRITE_BLOCK */
		if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlock(buff, 0xFE))
			count = 0;
	}
	else
	{
		/* WRITE_MULTIPLE_BLOCK */
		if (CardType & CT_SD1)
		{
			SD_SendCmd(CMD55, 0);
			SD_SendCmd(CMD23, count); /* ACMD23 */
		}

		if (SD_SendCmd(CMD25, sector) == 0)
		{
			do {
				if(!SD_TxDataBlock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);

			/* STOP_TRAN token */
			if(!SD_TxDataBlock(0, 0xFD))
			{
				count = 1;
			}
		}
	}

	/* Idle */
	DESELECT();
	SPI_RxByte();

	return count ? RES_ERROR : RES_OK;
}
#endif /* _USE_WRITE */

/* ioctl */
DRESULT SD_disk_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
	DRESULT res;
	uint8_t n, csd[16], *ptr = buff;
	WORD csize;

	/* pdrv should be 0 */
	if (drv) return RES_PARERR;
	res = RES_ERROR;

	if (ctrl == CTRL_POWER)
	{
		switch (*ptr)
		{
		case 0:
			SD_PowerOff();		/* Power Off */
			res = RES_OK;
			break;
		case 1:
			SD_PowerOn();		/* Power On */
			res = RES_OK;
			break;
		case 2:
			*(ptr + 1) = SD_CheckPower();
			res = RES_OK;		/* Power Check */
			break;
		default:
			res = RES_PARERR;
		}
	}
	else
	{
		/* no disk */
		if (Stat & STA_NOINIT) return RES_NOTRDY;

		SELECT();

		switch (ctrl)
		{
		case GET_SECTOR_COUNT:
			/* SEND_CSD */
			if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16))
			{
				if ((csd[0] >> 6) == 1)
				{
					/* SDC V2 */
					csize = csd[9] + ((WORD) csd[8] << 8) + 1;
					*(DWORD*) buff = (DWORD) csize << 10;
				}
				else
				{
					/* MMC or SDC V1 */
					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
					csize = (csd[8] >> 6) + ((WORD) csd[7] << 2) + ((WORD) (csd[6] & 3) << 10) + 1;
					*(DWORD*) buff = (DWORD) csize << (n - 9);
				}
				res = RES_OK;
			}
			break;
		case GET_SECTOR_SIZE:
			*(WORD*) buff = 512;
			res = RES_OK;
			break;
		case CTRL_SYNC:
			if (SD_ReadyWait() == 0xFF) res = RES_OK;
			break;
		case MMC_GET_CSD:
			/* SEND_CSD */
			if (SD_SendCmd(CMD9, 0) == 0 && SD_RxDataBlock(ptr, 16)) res = RES_OK;
			break;
		case MMC_GET_CID:
			/* SEND_CID */
			if (SD_SendCmd(CMD10, 0) == 0 && SD_RxDataBlock(ptr, 16)) res = RES_OK;
			break;
		case MMC_GET_OCR:
			/* READ_OCR */
			if (SD_SendCmd(CMD58, 0) == 0)
			{
				for (n = 0; n < 4; n++)
				{
					*ptr++ = SPI_RxByte();
				}
				res = RES_OK;
			}
		default:
			res = RES_PARERR;
		}

		DESELECT();
		SPI_RxByte();
	}

	return res;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_RNG_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_SET);
     HAL_Delay(20);
     ILI9341_Init();//initial driver setup to drive ili9341

   /*  FATFS fs; 	//Fatfs handle
     FIL fil; 		//File handle
     FRESULT fres; //Result after operations
     HAL_Delay(500);
     //Open the file system
     fres = f_mount(&fs, "/", 1); //1=mount now
     if (fres == FR_OK) {
     ILI9341_Draw_Text(" FR_OK", 0, 0, BLACK,1, WHITE);
     }
      if (fres == FR_DISK_ERR) { ILI9341_Draw_Text(" FR_DISK_ERR", 0, 25, BLACK,1, WHITE);}
      if (fres == FR_INT_ERR) { ILI9341_Draw_Text(" FR_INT_ERR", 0, 50, BLACK,1, WHITE);}
      if (fres == FR_NOT_READY) { ILI9341_Draw_Text(" FR_NOT_READY", 0, 75, BLACK,1, WHITE);}
      if (fres == FR_NO_FILE) { ILI9341_Draw_Text(" FR_NO_FILE", 0, 100, BLACK,1, WHITE);}
      if (fres == FR_NO_PATH) { ILI9341_Draw_Text(" FR_NO_PATH", 0, 125, BLACK,1, WHITE);}
      if (fres == FR_INVALID_NAME) { ILI9341_Draw_Text(" FR_INVALID_NAME", 0, 150, BLACK,1, WHITE);}
      if (fres == FR_DENIED) { ILI9341_Draw_Text(" FR_DENIED", 0, 175, BLACK,1, WHITE);}
      if (fres == FR_EXIST) { ILI9341_Draw_Text(" FR_EXIST", 0, 200, BLACK,1, WHITE);}
      if (fres == FR_INVALID_OBJECT) { ILI9341_Draw_Text(" FR_INVALID_OBJECT", 0, 225, BLACK,1, WHITE);}

            if (fres == FR_WRITE_PROTECTED) { ILI9341_Draw_Text(" FR_WRITE_PROTECTED", 100, 0, BLACK,1, WHITE);}
            if (fres == FR_INVALID_DRIVE) { ILI9341_Draw_Text(" FR_INVALID_DRIVE", 100, 25, BLACK,1, WHITE);}
            if (fres == FR_NOT_ENABLED) { ILI9341_Draw_Text(" FR_NOT_ENABLED", 100, 50, BLACK,1, WHITE);}
            if (fres == FR_NO_FILESYSTEM) { ILI9341_Draw_Text(" FR_NO_SYSTEM", 100, 75, BLACK,1, WHITE);}
            if (fres == FR_MKFS_ABORTED) { ILI9341_Draw_Text(" MKFS_ABORTED", 100, 100, BLACK,1, WHITE);}
            if (fres == FR_TIMEOUT) { ILI9341_Draw_Text(" FR_TIMEOUT", 100, 125, BLACK,1, WHITE);}
            if (fres == FR_LOCKED) { ILI9341_Draw_Text(" FR_LOCKED", 108, 150, BLACK,1, WHITE);}
            if (fres == FR_NOT_ENOUGH_CORE) { ILI9341_Draw_Text(" NOT_ENOUGH_CORE", 100, 175, BLACK,1, WHITE);}

            if (fres == FR_TOO_MANY_OPEN_FILES) { ILI9341_Draw_Text(" TOO_MANY_OPEN_FILES", 100, 200, BLACK,1, WHITE);}
            if (fres == FR_INVALID_PARAMETER) { ILI9341_Draw_Text(" INVALID_PARAMETER", 100, 225, BLACK,1, WHITE);}





     fres = f_mkdir("data");





     		  DWORD free_clusters, free_sectors, total_sectors;

     		    FATFS* getFreeFs;

     	 fres = f_open(&fil, "test.txt", FA_READ);
     	 if (fres == FR_OK) {
     	     ILI9341_Draw_Text("NOT FR_OK", 0, 0, BLACK,1, WHITE);
     	     }
     	      if (fres == FR_DISK_ERR) { ILI9341_Draw_Text(" FR_DISK_ERR 2", 0, 25, BLACK,1, WHITE);}
     	      if (fres == FR_INT_ERR) { ILI9341_Draw_Text(" FR_INT_ERR 2", 0, 50, BLACK,1, WHITE);}
     	      if (fres == FR_NOT_READY) { ILI9341_Draw_Text(" FR_NOT_READY 2", 0, 75, BLACK,1, WHITE);}
     	      if (fres == FR_NO_FILE) { ILI9341_Draw_Text(" FR_NO_FILE 2", 0, 100, BLACK,1, WHITE);}
     	      if (fres == FR_NO_PATH) { ILI9341_Draw_Text(" FR_NO_PATH 2", 0, 125, BLACK,1, WHITE);}
     	      if (fres == FR_INVALID_NAME) { ILI9341_Draw_Text(" FR_INVALID_NAME 2", 0, 150, BLACK,1, WHITE);}
     	      if (fres == FR_DENIED) { ILI9341_Draw_Text(" FR_DENIED 2", 0, 175, BLACK,1, WHITE);}
     	      if (fres == FR_EXIST) { ILI9341_Draw_Text(" FR_EXIST 2", 0, 200, BLACK,1, WHITE);}
     	      if (fres == FR_INVALID_OBJECT) { ILI9341_Draw_Text(" FR_INVALID_OBJECT 2", 0, 225, BLACK,1, WHITE);}

     	            if (fres == FR_WRITE_PROTECTED) { ILI9341_Draw_Text(" FR_WRITE_PROTECTED 2", 100, 0, BLACK,1, WHITE);}
     	            if (fres == FR_INVALID_DRIVE) { ILI9341_Draw_Text(" FR_INVALID_DRIVE 2", 100, 25, BLACK,1, WHITE);}
     	            if (fres == FR_NOT_ENABLED) { ILI9341_Draw_Text(" FR_NOT_ENABLED 2", 100, 50, BLACK,1, WHITE);}
     	            if (fres == FR_NO_FILESYSTEM) { ILI9341_Draw_Text(" FR_NO_SYSTEM 2", 100, 75, BLACK,1, WHITE);}
     	            if (fres == FR_MKFS_ABORTED) { ILI9341_Draw_Text(" MKFS_ABORTED 2", 100, 100, BLACK,1, WHITE);}
     	            if (fres == FR_TIMEOUT) { ILI9341_Draw_Text(" FR_TIMEOUT 2", 100, 125, BLACK,1, WHITE);}
     	            if (fres == FR_LOCKED) { ILI9341_Draw_Text(" FR_LOCKED 2", 108, 150, BLACK,1, WHITE);}
     	            if (fres == FR_NOT_ENOUGH_CORE) { ILI9341_Draw_Text(" NOT_ENOUGH_CORE 2", 100, 175, BLACK,1, WHITE);}

     	            if (fres == FR_TOO_MANY_OPEN_FILES) { ILI9341_Draw_Text(" TOO_MANY_OPEN_FILES 2", 100, 200, BLACK,1, WHITE);}
     	            if (fres == FR_INVALID_PARAMETER) { ILI9341_Draw_Text(" INVALID_PARAMETER 2", 100, 225, BLACK,1, WHITE);}


     	//  else
       //  ILI9341_Draw_Text("File Opened", 40, 100, BLACK,1, WHITE);
     	// unsigned int switch3 ;

     //	 switch3 =  HAL_GPIO_ReadPin (SD_DETECT_GPIO_Port, SD_DETECT_Pin);
     	// ILI9341_Draw_Text(switch3, 80, 150, BLACK,1, WHITE);
*/

     ILI9341_Set_Rotation(SCREEN_VERTICAL_2);
    		  ILI9341_SetWindow(0,0,ILI9341_SCREEN_HEIGHT,ILI9341_SCREEN_WIDTH);
 //    HAL_Delay (500);
     volatile uint16_t leftX = 50;
    	  volatile uint16_t rightX = 190;
    	  volatile uint16_t topY = 250;
    	  volatile uint16_t bottomY = 300;

    	  ILI9341_Draw_Filled_Rectangle_Coord(leftX,topY,rightX,bottomY,BLUE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {




	  {
	    		  uint8_t buf[64] = { 0 };
	    		  buf[0] = 0x03;
	    		  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1, 0x70, buf, 1, 500);
	    		  if (ret == HAL_OK)
	    		  {
	    			 ret = HAL_I2C_Master_Receive(&hi2c1, 0x70, buf, 4, 500);
	    			 if (ret == HAL_OK)
	    			 {
	    				// Mask out all but the 4 LSbits of the MSB for X and Y registers (see data sheet).
	    				volatile uint16_t x = (((uint16_t)(buf[0] & 0x0F)) << 8) + buf[1];
	    				volatile uint16_t y = (((uint16_t)(buf[2] & 0x0F)) << 8) + buf[3];

	    				int n2 = 1;
	    			    int8_t switch1;
	    			    int n3 = 1;
	    			    int8_t switch3;
	    			    int n4 = 1;
	    			    int8_t switch4;
	    			    int n5 = 1;
	    			    int8_t switch5;
	    			    int n6 = 1;
	    			    int8_t switch6;

	    				switch1 =  HAL_GPIO_ReadPin (T_IRQ_GPIO_Port, T_IRQ_Pin);
	    			    n2 = switch1;
	    			    HAL_Delay(5);
	    				switch3 =  HAL_GPIO_ReadPin (T_IRQ_GPIO_Port, T_IRQ_Pin);
	    				n3 = switch3;
	    				HAL_Delay(5);
	    				switch4 =  HAL_GPIO_ReadPin (T_IRQ_GPIO_Port, T_IRQ_Pin);
	    				n4 = switch4;
	    				HAL_Delay(5);
	    				switch5 =  HAL_GPIO_ReadPin (T_IRQ_GPIO_Port, T_IRQ_Pin);
	    				n5 = switch5;
	    				HAL_Delay(5);
	    			    switch6 =  HAL_GPIO_ReadPin (T_IRQ_GPIO_Port, T_IRQ_Pin);
	    				n6 = switch6;


	    				if ((n2==0)||(n3==0)||(n4==0)||(n5==0)||(n6==0))
	    				{
	    				//  if ((x != 0) || (y != 0))
	    				//  {
	    				//	if((x >= leftX) && (x <= rightX))
	    				//	{
	    				//	  if((y >= topY) && (y <= bottomY))
	    				//	  {
	    					    ILI9341_Draw_Filled_Rectangle_Coord(leftX,topY,rightX,bottomY,DARKCYAN);
	    					//  }
	    					//}
	    				 // }
	    				}
	    			else if ((n2==1)&&(n3==1)&&(n4==1)) { ILI9341_Draw_Filled_Rectangle_Coord(leftX,topY,rightX,bottomY,BLUE);}
	    			//	 if (z == 0)
	    			//	{

	    				//	ILI9341_Draw_Filled_Rectangle_Coord(leftX,topY,rightX,bottomY,BLUE);

	    				//}


	    				char strX[16];
	    				char strY[16];
	    				//char strZ[16];
	    				//char strU[16];

	    				sprintf(strX, "%u", n2);
	    				sprintf(strY, "%u", n3);
	    				//sprintf(strZ, "%u", z);

	    				ILI9341_Draw_Text(strX, 0, 25, BLACK,1, WHITE);
	    				ILI9341_Draw_Text(strY, 0, 50, BLACK,1, WHITE);
	    			//	ILI9341_Draw_Text(strZ, 0, 75, BLACK,1, WHITE);

	    				strX[0] = '/0';
	    				strY[0] = '/0';
	    			//	strZ[0] = '/0';
	    			 }
	    		  }
	    	  }


	                  uint8_t bufz[64] = { 0 };
	 	    		  bufz[0] = 0x07;
	 	    		  HAL_StatusTypeDef retz = HAL_I2C_Master_Transmit(&hi2c1, 0x70, bufz, 1, 500);
	 	    		  if (retz == HAL_OK)
	 	    		  {
	 	    			 retz = HAL_I2C_Master_Receive(&hi2c1, 0x70, bufz, 1, 500);
	 	    			 if (retz == HAL_OK)
	 	    			 {

	 	    				volatile uint8_t z = (((uint8_t)(bufz[0] )));

	 	    				char strZ[8];
	 	    				sprintf(strZ, "%u", z);
	 	    				ILI9341_Draw_Text(strZ, 10, 75, BLACK,1, WHITE);

	 	    			 }
	 	    		  }

	//  ILI9341_Draw_Filled_Rectangle_Coord(leftX,topY,rightX,bottomY,DARKCYAN);



	 // ILI9341_Draw_Horizontal_Line(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Colour)
	 // ILI9341_Draw_Hollow_Rectangle_Coord(50,250,190,300,PURPLE);
	  //ILI9341_Draw_Hollow_Rectangle_Coord(49,249,190,300,RED);
//Test_Lights_and_Buttons();

//Test_SD_Card();

//Test_Buzzer();

//Test_Screen_Visual();









    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x204064FF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20404768;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 7;
  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, BLE_RESET_Pin|RST_Pin|DAC_LDAC_Pin|DAC_MZ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, DC_Pin|LED_CHG_O_Pin|LED_PWR_Pin|LED_CHG_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin|BACKLIGHT_Pin|BUZZER_Pin|SD_CS_Pin
                          |ADC_CS3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EXP_GPIO_1_Pin|HUM_RESET_Pin|EXP_GPIO_2_Pin|VAN_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_CS4_GPIO_Port, ADC_CS4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLE_IRQ_Pin */
  GPIO_InitStruct.Pin = BLE_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_RESET_Pin RST_Pin DAC_LDAC_Pin DAC_MZ_Pin */
  GPIO_InitStruct.Pin = BLE_RESET_Pin|RST_Pin|DAC_LDAC_Pin|DAC_MZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : T_IRQ_Pin */
  GPIO_InitStruct.Pin = T_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin LED_CHG_O_Pin LED_PWR_Pin LED_CHG_G_Pin */
  GPIO_InitStruct.Pin = DC_Pin|LED_CHG_O_Pin|LED_PWR_Pin|LED_CHG_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin BACKLIGHT_Pin BUZZER_Pin ADC_CS3_Pin */
  GPIO_InitStruct.Pin = CS_Pin|BACKLIGHT_Pin|BUZZER_Pin|ADC_CS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SWITCH_2_Pin */
  GPIO_InitStruct.Pin = SWITCH_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWITCH_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BATCHG_Pin SD_DETECT_Pin */
  GPIO_InitStruct.Pin = BATCHG_Pin|SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXP_GPIO_1_Pin HUM_RESET_Pin EXP_GPIO_2_Pin VAN_EN_Pin */
  GPIO_InitStruct.Pin = EXP_GPIO_1_Pin|HUM_RESET_Pin|EXP_GPIO_2_Pin|VAN_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_DRDY1_Pin */
  GPIO_InitStruct.Pin = ADC_DRDY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_DRDY1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC_DRDY3_Pin ADC_DRDY4_Pin HUM_ALERT_Pin */
  GPIO_InitStruct.Pin = ADC_DRDY3_Pin|ADC_DRDY4_Pin|HUM_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_CS4_Pin */
  GPIO_InitStruct.Pin = ADC_CS4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADC_CS4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DAC_READY_Pin */
  GPIO_InitStruct.Pin = DAC_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DAC_READY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
