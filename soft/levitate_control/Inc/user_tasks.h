#include "FreeRTOS.h"
#include "stdint.h"
#include "main.h"

void initBSP(void);
uint16_t readADC(ADC_HandleTypeDef* hadc, uint32_t Channel);
uint16_t readHALL_1(void);
uint16_t readHALL_2(void);
uint16_t readJOYSTICK_X(void);
uint16_t readJOYSTICK_Y(void);
void setPWMValHall_1(uint16_t val);
void setPWMValHall_2(uint16_t val);
void setPWMHall_1(float val);
void setPWMHall_2(float val);
float saturateValue(float val, float satValAbs);
float addDeadZone(float val, float deadZoneAbs);
void initTasks(void);
void vLevitateControlTask(void * pvParameters);
void vMenuHandlerTask(void * pvParameters);
