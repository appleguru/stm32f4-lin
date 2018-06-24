//--------------------------------------------------------------
// File     : stm32_ub_led.h
//--------------------------------------------------------------

//--------------------------------------------------------------
#ifndef __STM32F4_UB_LED_H
#define __STM32F4_UB_LED_H

//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"




//--------------------------------------------------------------
// List of all LEDs
// (no number twice and starting from 0)
//--------------------------------------------------------------
typedef enum 
{
  LED_GREEN = 0,  // LED4 on the STM32F4 discovery
  LED_ORANGE = 1, // LED3 on the STM32F4 discovery
  LED_RED = 2,    // LED5 on the STM32F4 discovery
  LED_BLUE = 3    // LED6 on the STM32F4 discovery
}LED_NAME_t;

#define  LED_ANZ   4 // number of LED_NAME_t


//--------------------------------------------------------------
// LED Status
//--------------------------------------------------------------
typedef enum {
  LED_OFF = 0,  // LED OFF
  LED_ON        // LED ON
}LED_STATUS_t;


//--------------------------------------------------------------
// LED Struct
//--------------------------------------------------------------
typedef struct {
  LED_NAME_t LED_NAME;    // Name
  GPIO_TypeDef* LED_PORT; // Port
  const uint16_t LED_PIN; // Pin
  const uint32_t LED_CLK; // Clock
  LED_STATUS_t LED_INIT;  // Init
}LED_t;


//--------------------------------------------------------------
// Global Functions
//--------------------------------------------------------------
void UB_Led_Init(void);
void UB_Led_Off(LED_NAME_t led_name);
void UB_Led_On(LED_NAME_t led_name);
void UB_Led_Toggle(LED_NAME_t led_name);
void UB_Led_Switch(LED_NAME_t led_name, LED_STATUS_t wert);



//--------------------------------------------------------------
#endif // __STM32F4_UB_LED_H
