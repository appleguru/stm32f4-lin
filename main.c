//--------------------------------------------------------------
// File     : main.c
// Date     : 11.04.2013
// Version  : 1.0
// Author   : UB
// EMail    : mc-4u(@)t-online.de
// Web      : www.mikrocontroller-4u.de
// CPU      : STM32F4
// IDE      : CooCox CoIDE 1.7.4
// GCC      : 4.7 2012q4
// Module   : CMSIS_BOOT, M4_CMSIS_CORE
// Function : Demo of the LIN Master Library
// Note     : These two files have to be at 8MHz
//              "cmsis_boot/stm32f4xx.h"
//              "cmsis_boot/system_stm32f4xx.c"
//--------------------------------------------------------------

#include "main.h"
#include "stm32_ub_lin_master.h"
#include "stm32_ub_led.h"

void pause(void)
{
  volatile uint32_t n;
  for(n=0;n<5000000;n++);
}

int main(void)
{
  LIN_FRAME_t myFrame;
  LIN_ERR_t check;
  SystemInit(); // enable quartz settings

  // init of the LEDs
  UB_Led_Init();

  // init the UART as LIN master
  UB_LIN_Master_Init();

  while(1)
  {
    // small pause
    pause();
    UB_Led_Toggle(LED_BLUE);

    //---------------------------------------
    // Send 2 bytes to the slave via LIN
    //---------------------------------------
    myFrame.frame_id=0x01;
    myFrame.data_len=2;
    myFrame.data[0]=0xA1;
    myFrame.data[1]=0xB2;
    UB_LIN_SendData(&myFrame);

    // small pause
    pause();
    UB_Led_Toggle(LED_BLUE);

    //---------------------------------------
    // Read 1 byte from the slave via LIN
    //---------------------------------------
    myFrame.frame_id=0x02;
    myFrame.data_len=1;
    check=UB_LIN_ReceiveData(&myFrame);
    // check data
    if((check==LIN_OK) && (myFrame.data[0]==0xF9)) {
      UB_Led_Toggle(LED_GREEN);
    }
    else {
      UB_Led_Toggle(LED_RED);
    }
  }
}

