//--------------------------------------------------------------
// File     : stm32_ub_lin_master.h
//--------------------------------------------------------------

//--------------------------------------------------------------
#ifndef __STM32F4_UB_LIN_MASTER_H
#define __STM32F4_UB_LIN_MASTER_H



//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "misc.h"



//--------------------------------------------------------------
// GPIO : UART-TX (PA2)
//--------------------------------------------------------------
#define  LIN_TX_CLOCK    RCC_AHB1Periph_GPIOA
#define  LIN_TX_PIN      GPIO_Pin_2
#define  LIN_TX_SOURCE   GPIO_PinSource2
#define  LIN_TX_PORT     GPIOA


//--------------------------------------------------------------
// GPIO : UART-RX (PA3)
//--------------------------------------------------------------
#define  LIN_RX_CLOCK    RCC_AHB1Periph_GPIOA
#define  LIN_RX_PIN      GPIO_Pin_3
#define  LIN_RX_SOURCE   GPIO_PinSource3
#define  LIN_RX_PORT     GPIOA


//--------------------------------------------------------------
// UART : (UART-2)
//--------------------------------------------------------------
#define  LIN_UART        USART2
#define  LIN_UART_CLOCK  RCC_APB1Periph_USART2
#define  LIN_UART_AF     GPIO_AF_USART2
#define  LIN_UART_IRQ    USART2_IRQn
#define  LIN_UART_ISR    USART2_IRQHandler


//--------------------------------------------------------------
// GPIO : NSLP pin from the transceiver (PA1)
//--------------------------------------------------------------
#define  LIN_NSLP_CLOCK    RCC_AHB1Periph_GPIOA
#define  LIN_NSLP_PIN      GPIO_Pin_1
#define  LIN_NSLP_PORT     GPIOA



//--------------------------------------------------------------
// baud rate (max 20 kBaud)
//
// @ 9600 Baud: Field Time (10bit) = about 1ms
//--------------------------------------------------------------
#define  LIN_UART_BAUD   9600



//--------------------------------------------------------------
// Global Defines
//--------------------------------------------------------------
#define  LIN_SYNC_DATA               0x55  // SyncField (do not change)
#define  LIN_MAX_DATA                   8  // max 8 databytes



//--------------------------------------------------------------
// Defines for the transceiver (do not change !!)
//--------------------------------------------------------------
#define  LIN_POWERON_DELAY          10000  // Pause at PowerOn     (ca. 10ms)
#define  LIN_AKTIV_DELAY               50  // Pause for Transceiver (ca. 50us)



//--------------------------------------------------------------
// breaks (be careful when changing !!)
//--------------------------------------------------------------
#define  LIN_INTER_FRAME_DELAY      10000  // Pause (Frame->Frame)   (ca. 10ms)
#define  LIN_FRAME_RESPONSE_DELAY    2000  // Pause (Header->Data)   (ca.  2ms)
#define  LIN_BREAKFIELD_DELAY        4000  // Pause (Breakfield)     (ca.  4ms)
#define  LIN_DATA_BYTE_DELAY         1000  // Pause (Data->Data)     (ca.  1ms)
#define  LIN_RX_TIMEOUT_CNT         30000  // Receive Timeout        (ca.  5ms)



//--------------------------------------------------------------
// LIN frame Struct
//--------------------------------------------------------------
typedef struct {
  uint8_t frame_id;              // unique ID number from the frame
  uint8_t data_len;              // number of data bytes
  uint8_t data[LIN_MAX_DATA];    // data fields  
}LIN_FRAME_t;



//--------------------------------------------------------------
// Master Mode
//--------------------------------------------------------------
typedef enum { 
  RECEIVE_DATA =0,  // receive data
  RECEIVE_CRC,      // receive CRC
  SEND_DATA         // send data
}LIN_MODE_t;



//--------------------------------------------------------------
// Master struct
//--------------------------------------------------------------
typedef struct {
  LIN_MODE_t mode;  // current Mode
  uint8_t data_ptr; // data pointer  
  uint8_t crc;      // checksum
}LIN_MASTER_t;



//--------------------------------------------------------------
// error messages
//--------------------------------------------------------------
typedef enum {
  LIN_OK  = 0,   // no error
  LIN_WRONG_LEN, // wrong number of data
  LIN_RX_EMPTY,  // no frame received
  LIN_WRONG_CRC  // Checksum wrong
}LIN_ERR_t;



//--------------------------------------------------------------
// Global Functions
//--------------------------------------------------------------
void UB_LIN_Master_Init(void);
LIN_ERR_t UB_LIN_SendData(LIN_FRAME_t *frame);
LIN_ERR_t UB_LIN_ReceiveData(LIN_FRAME_t *frame);




//--------------------------------------------------------------
#endif // __STM32F4_UB_LIN_MASTER_H
