#include "voltage_monitors.h"
#include <stdio.h>
#include <string.h>

uint16_t adc_raw;
float adc_voltage;
float voltage;
char msg[100];

extern UART_HandleTypeDef hlpuart1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/* Get 24V ADC value */
void get_vin()
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	adc_raw = HAL_ADC_GetValue(&hadc1);

	HAL_UART_Transmit(&hlpuart1, "\n", 1, HAL_MAX_DELAY);
	sprintf(msg, "24V ADC Value = %hu\r\n", adc_raw);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	adc_voltage = adc_raw * 0.0008056640625000;
	sprintf(msg, "24V Scaled = %.4f\r\n", adc_voltage);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	voltage = adc_voltage/0.0625;
	sprintf(msg, "24V Voltage = %.4f\r\n", voltage);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_UART_Transmit(&hlpuart1, "\n", 1, HAL_MAX_DELAY);

	HAL_Delay(250);
	HAL_ADC_Stop(&hadc1);
}

void get_3V3()
{
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	adc_raw = HAL_ADC_GetValue(&hadc2);

	HAL_UART_Transmit(&hlpuart1, "\n", 1, HAL_MAX_DELAY);
	sprintf(msg, "3V3 ADC Value = %hu\r\n", adc_raw);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	adc_voltage = adc_raw * 0.0008056640625000;
	sprintf(msg, "3V3 Scaled = %.4f\r\n", adc_voltage);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	voltage = adc_voltage/0.452555;
	sprintf(msg, "3V3 Voltage = %.4f\r\n", voltage);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_UART_Transmit(&hlpuart1, "\n", 1, HAL_MAX_DELAY);

	HAL_Delay(250);
	HAL_ADC_Stop(&hadc2);
}
