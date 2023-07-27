#include "LEDs.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef hlpuart1;

void UVLEDTest(void)
{
	/* All off */
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red */
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Green */
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Blue */
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_RESET);

	/* Test White */
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red-Blue (Purple) */
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red-Green (Yellow) */
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red-White (Pink) */
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Blue-Green */
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Blue-White */
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Green-White */
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red-Blue-Green */
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red-Blue-White */
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red-Green-White */
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Blue-Green-White */
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red - Blue-Green-White */
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(UV_R_CTRL_GPIO_Port, UV_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_B_CTRL_GPIO_Port, UV_B_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_G_CTRL_GPIO_Port, UV_G_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(UV_W_CTRL_GPIO_Port, UV_W_CTRL_Pin, GPIO_PIN_RESET);

}

void HEPALEDTest(void)
{
	/* All off */
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red */
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Green */
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Blue */
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_RESET);

	/* Test White */
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red-Blue (Purple) */
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red-Green (Yellow) */
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red-White (Pink) */
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Blue-Green */
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Blue-White */
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Green-White */
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red-Blue-Green */
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red-Blue-White */
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red-Green-White */
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Blue-Green-White */
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_RESET);

	/* Test Red-Blue-Green-White */
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(HEPA_R_CTRL_GPIO_Port, HEPA_R_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_B_CTRL_GPIO_Port, HEPA_B_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_G_CTRL_GPIO_Port, HEPA_G_CTRL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HEPA_W_CTRL_GPIO_Port, HEPA_W_CTRL_Pin, GPIO_PIN_RESET);

}
