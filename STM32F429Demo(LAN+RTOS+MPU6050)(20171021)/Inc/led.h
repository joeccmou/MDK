#ifndef __LED_H
#define	__LED_H

#include "stm32f4xx.h"


/* 定义控制IO的宏 */
#define LED_R_TOGGLE	   HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin)
#define LED_R_OFF		     HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET)
#define LED_R_ON			   HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET)

#define LED_G_TOGGLE		HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin)
#define LED_G_OFF		    HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET)
#define LED_G_ON		   	HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET)

#define LED_B_TOGGLE		HAL_GPIO_TogglePin(LED_B_GPIO_Port,LED_B_Pin)
#define LED_B_OFF		    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET)
#define LED_B_ON			  HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET)

#define LED_4_TOGGLE		HAL_GPIO_TogglePin(LED_4_GPIO_Port,LED_4_Pin)
#define LED_4_OFF		    HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin,GPIO_PIN_SET)
#define LED_4_ON			  HAL_GPIO_WritePin(LED_4_GPIO_Port,LED_4_Pin,GPIO_PIN_RESET)

/** 控制LED灯亮灭的宏，
	* LED低电平亮，设置ON=0，OFF=1
	* 若LED高电平亮，把宏设置成ON=1 ，OFF=0 即可
	*/
#define ON  1
#define OFF 0

/* 带参宏，可以像内联函数一样使用 */
#define LED_R(a)	if (a)	\
          {	LED_R_ON; }  \
					else		        \
					{ LED_R_OFF;}

#define LED_G(a)	if (a)	\
          { LED_G_ON; }   \
					else		\
					{ LED_G_OFF; }

#define LED_B(a)	if (a)	\
				  { LED_B_ON; }   \
					else		\
					{ LED_B_OFF; }
					
#define LED_4(a)	if (a)	\
				  { LED_4_ON; }   \
					else		\
					{ LED_4_OFF; }					
					

/* 基本混色，后面高级用法使用PWM可混出全彩颜色,且效果更好 */

//红
#define LED_RED  	{LED_R_ON;	LED_G_OFF;	LED_B_OFF;}

//绿
#define LED_GREEN		{LED_R_OFF; LED_G_ON; LED_B_OFF;}

//蓝
#define LED_BLUE		{LED_R_OFF;	LED_G_OFF; 	LED_B_ON;}

//黄(红+绿)					
#define LED_YELLOW	{LED_R_ON;	LED_G_ON;	LED_B_OFF;}

//紫(红+蓝)
#define LED_PURPLE	{LED_R_ON;	LED_G_OFF; LED_B_ON;}

//青(绿+蓝)
#define LED_CYAN  {LED_R_OFF; LED_G_ON; LED_B_ON;}
					
//白(红+绿+蓝)
#define LED_WHITE	 {LED_R_ON; LED_G_ON; LED_B_ON;}
					
//黑(全部关闭)
#define LED_RGBOFF	{LED_R_OFF; LED_G_OFF;	LED_B_OFF}
					

#endif /* __LED_H */
