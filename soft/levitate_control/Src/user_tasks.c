#include "user_tasks.h"
#include "task.h"
#include "math.h"
#include "stdint.h"
#include "cmsis_os.h"
#include "main.h"

#define PID_DESIRED_VAL 								2048
#define LEVITATE_CONTROL_PERIOD_MS 			25

static TaskHandle_t xHandle;
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

TickType_t levitateControlPeriod;

void initBSP(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

uint16_t readADC(ADC_HandleTypeDef* hadc, uint32_t Channel)
{
	uint16_t adc_result = 0;
	
	ADC_ChannelConfTypeDef adcConfig = {0};
	
	adcConfig.Channel = Channel;
  adcConfig.Rank = ADC_REGULAR_RANK_1;
  adcConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(hadc, &adcConfig) != HAL_OK)
  {
    Error_Handler();
  }
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 10);
	adc_result = HAL_ADC_GetValue(hadc);
	
	return adc_result;
}

uint16_t readHALL_1(void)
{
	uint16_t adc_result = 0;
	
	adc_result = readADC(&hadc1, ADC_CHANNEL_1);
	
	return adc_result;
}

uint16_t readHALL_2(void)
{
	uint16_t adc_result = 0;
	
	adc_result = readADC(&hadc1, ADC_CHANNEL_2);
	
	return adc_result;
}

uint16_t readJOYSTICK_X(void)
{
	uint16_t adc_result = 0;
	
	adc_result = readADC(&hadc2, ADC_CHANNEL_3);
	
	return adc_result;
}

uint16_t readJOYSTICK_Y(void)
{
	uint16_t adc_result = 0;
	
	adc_result = readADC(&hadc2, ADC_CHANNEL_4);
	
	return adc_result;
}

void setPWMHall_1_A(uint16_t val)
{
	TIM1->CCR1 = val;
}
	
void setPWMHall_1_B(uint16_t val)
{
	TIM1->CCR2 = val;
}
	
void setPWMHall_2_A(uint16_t val)
{
	TIM1->CCR3 = val;
}
	
void setPWMHall_2_B(uint16_t val)
{
	TIM1->CCR4 = val;
}
	
void setPWMHall_1(float val)
{
	if(val >= 0)
	{
		setPWMHall_1_A((uint16_t)floor(val));
		setPWMHall_1_B(0);
	}
	else
	{
		setPWMHall_1_A(0);
		setPWMHall_1_B((uint16_t)(floor(val*(-1))));
	}
}

void setPWMHall_2(float val)
{
	if(val >= 0)
	{
		setPWMHall_2_A((uint16_t)floor(val));
		setPWMHall_2_B(0);
	}
	else
	{
		setPWMHall_2_A(0);
		setPWMHall_2_B((uint16_t)(floor(val*(-1))));
	}
}

float saturateValue(float val, float satValAbs)
{
	float valSign;
	valSign = (float)((0 < val) - (val < 0));
	if(fabs(val) > satValAbs)
			return valSign * satValAbs;
	else
		return val;
}

float addDeadZone(float val, float deadZoneAbs)
{
	float valSign;
	valSign = (float)((0 < val) - (val < 0));
	if(fabs(val) < deadZoneAbs)
			return valSign * deadZoneAbs;
	else
		return val;
}

void initTasks(void)
{	
	initBSP();
	
	levitateControlPeriod = LEVITATE_CONTROL_PERIOD_MS / portTICK_PERIOD_MS;
	
	xTaskCreate(
              vLevitateControlTask,       /* Function that implements the task. */
							"LevitateControl",          /* Text name for the task. */
							128,      /* Stack size in words, not bytes. */
							( void * ) 1,    /* Parameter passed into the task. */
							3,/* Priority at which the task is created. */
							&xHandle );   
	
	
}

void vLevitateControlTask(void * pvParameters)
{
	int i = 0;
  uint16_t hall1Value;
	uint16_t hall2Value;
	
	float coefPHall_1 = 1;
	float coefIHall_1 = 0;
	float coefDHall_1 = 0;
	
	float coefPHall_2 = 1;
	float coefIHall_2 = 0;
	float coefDHall_2 = 0;
	
	float integralSummHall1 = 0;
	float integralSummHall2 = 0;
	float errorValueHall1Prev = 0;
	float errorValueHall2Prev = 0;
	float errorValueHall1;
	float errorValueHall2;
	float resultHall1;
	float resultHall2;
	
	
	while(1)
	{
		// PID processing HALL 1
		hall1Value = readHALL_1();
		errorValueHall1 = PID_DESIRED_VAL - hall1Value;
		resultHall1 = integralSummHall1 * coefIHall_1 + coefDHall_1 * (errorValueHall1Prev - errorValueHall1)/((float)LEVITATE_CONTROL_PERIOD_MS / 1000.0) + errorValueHall1 * coefPHall_1;
		resultHall1 = saturateValue(resultHall1, 1024);
		integralSummHall1 = integralSummHall1 + errorValueHall1;
		integralSummHall1 = saturateValue(integralSummHall1, 4096);
		errorValueHall1Prev = errorValueHall1;

		setPWMHall_1(1024*sin(i*3.14/180.0));		
		i++;	
		
		// PID processing HALL 2
		hall2Value = readHALL_2();
		errorValueHall2 = PID_DESIRED_VAL - hall2Value;
		resultHall2 = integralSummHall2 * coefIHall_2 + coefDHall_2 * (errorValueHall2Prev - errorValueHall2)/((float)LEVITATE_CONTROL_PERIOD_MS / 1000.0) + errorValueHall2 * coefPHall_2;
		resultHall2 = saturateValue(resultHall2, 1024);
		integralSummHall2 = integralSummHall2 + errorValueHall2;
		integralSummHall2 = saturateValue(integralSummHall2, 4096);
		errorValueHall2Prev = errorValueHall2;
		
		setPWMHall_2(1024*cos(i*3.14/180.0));	
		
		vTaskDelay( levitateControlPeriod );
	}
	
}

void vMenuHandlerTask(void * pvParameters)
{
	
}
