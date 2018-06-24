//--------------------------------------------------------------
// File     : stm32_ub_lin_master.c
// Date     : 13.07.2014
// Version  : 1.1
// Autor    : UB
// EMail    : mc-4u(@)t-online.de
// Web      : www.mikrocontroller-4u.de
// CPU      : STM32F4
// IDE      : CooCox CoIDE 1.7.4
// GCC      : 4.7 2012q4
// Modules  : GPIO, UART, MISC
// Function : LIN interface (master)
//
// Notes: LIN Transceiver = MAX13020 / MAX13021 (or TJA1020)
//
// UART-TX (PA2) to  MAX13020 (TXD / Pin4)
// UART-RX (PA3) to  MAX13020 (RXD / Pin1)
// GPIO (PA1)    to  MAX13020 (NSLP / Pin2)
//--------------------------------------------------------------


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32_ub_lin_master.h"


//--------------------------------------------------------------
// Global variables
//--------------------------------------------------------------
LIN_MASTER_t LIN_MASTER;
LIN_FRAME_t LIN_FRAME;



//--------------------------------------------------------------
// internal functions
//--------------------------------------------------------------
void p_LIN_initGPIO(void);
void p_LIN_initUART(void);
void p_LIN_initNVIC(void);
void p_LIN_aktivateTransceiver(void);
uint8_t p_LIN_makeChecksum(LIN_FRAME_t *frame);
void p_LIN_wait_us(uint32_t n);



//--------------------------------------------------------------
// init of the LIN interface (in master mode)
//--------------------------------------------------------------
void UB_LIN_Master_Init(void)
{
  // init of all variables
  LIN_MASTER.mode=SEND_DATA;
  LIN_MASTER.data_ptr=0;  
  LIN_MASTER.crc=0;

  LIN_FRAME.frame_id=0;
  LIN_FRAME.data_len=0; 
  LIN_FRAME.data[0]=0;

  // init the GPIOs
  p_LIN_initGPIO();
  // init from the UART
  p_LIN_initUART();
  // init from NVIC
  p_LIN_initNVIC();

  // wait a moment until the transceiver is ready
  p_LIN_wait_us(LIN_POWERON_DELAY);

  // turn on the transceiver
  p_LIN_aktivateTransceiver();
}


// --------------------------------------------------------------
// sends data via LIN interface
// frame:
// frame_id = Unique ID [0x00 to 0xFF]
// data_len = number of data to be sent
// data [] = data to be sent
//
// return_value:
// LIN_OK = Frame has been sent
// LIN_WRONG_LEN = wrong number of data
// --------------------------------------------------------------
LIN_ERR_t UB_LIN_SendData(LIN_FRAME_t *frame)
{
  uint8_t checksum,n;

  // check the length
  if((frame->data_len<1) || (frame->data_len>LIN_MAX_DATA)) {
    return(LIN_WRONG_LEN);
  }   

  // calculate checksum
  checksum=p_LIN_makeChecksum(frame);

  // wait until the last byte has been sent
  while (USART_GetFlagStatus(LIN_UART, USART_FLAG_TXE) == RESET);

  //------------------------
  // Break-Field
  //------------------------
  USART_SendBreak(LIN_UART);
  // wait until BreakField has been sent 
  while (USART_GetFlagStatus(LIN_UART, USART_FLAG_TC) == RESET);

  // small pause
  p_LIN_wait_us(LIN_BREAKFIELD_DELAY);

  //------------------------
  // Sync-Field
  //------------------------
  USART_SendData(LIN_UART, LIN_SYNC_DATA);
  // wait until SyncField has been sent
  while (USART_GetFlagStatus(LIN_UART, USART_FLAG_TC) == RESET);

  // small pause
  p_LIN_wait_us(LIN_DATA_BYTE_DELAY); 

  //------------------------
  // ID-Field
  //------------------------
  USART_SendData(LIN_UART, frame->frame_id);
  // wait until IDField has been sent
  while (USART_GetFlagStatus(LIN_UART, USART_FLAG_TC) == RESET);

  // small pause
  p_LIN_wait_us(LIN_FRAME_RESPONSE_DELAY);

  //------------------------
  // Data-Field [1...n]
  //------------------------
  for(n=0;n<frame->data_len;n++) {
    USART_SendData(LIN_UART, frame->data[n]);
    // wait until DataField has been sent
    while (USART_GetFlagStatus(LIN_UART, USART_FLAG_TC) == RESET);

    // small Pause
    p_LIN_wait_us(LIN_DATA_BYTE_DELAY);
  }

  //------------------------
  // CRC-Field
  //------------------------
  USART_SendData(LIN_UART, checksum);
  // wait until CRCField has been sent
  while (USART_GetFlagStatus(LIN_UART, USART_FLAG_TC) == RESET);

  // small pause
  // so that the next frame is not sent too fast
  p_LIN_wait_us(LIN_INTER_FRAME_DELAY);

  return(LIN_OK);    
}


// --------------------------------------------------------------
// receives data via LIN interface
// frame:
// frame_id = Unique ID [0x00 to 0xFF]
// data_len = number of data to be received
// return:
// data [] = data received (if LIN_OK)
//
// return_value:
// LIN_OK = Frame was received
// LIN_WRONG_LEN = wrong number of data
// LIN_RX_EMPTY = no frame received
// LIN_WRONG_CRC = Checksum wrong
// --------------------------------------------------------------
LIN_ERR_t UB_LIN_ReceiveData(LIN_FRAME_t *frame)
{
  uint32_t rx_timeout;
  uint8_t checksum,n;

  // check the length
  if((frame->data_len<1) || (frame->data_len>LIN_MAX_DATA)) {
    return(LIN_WRONG_LEN);
  }

  // wait until the last byte has been sent
  while (USART_GetFlagStatus(LIN_UART, USART_FLAG_TXE) == RESET);

  //-------------------------------
  // Break-Field
  //-------------------------------
  USART_SendBreak(LIN_UART);
  // wait until BreakField has been sent 
  while (USART_GetFlagStatus(LIN_UART, USART_FLAG_TC) == RESET);

  // small pause
  p_LIN_wait_us(LIN_BREAKFIELD_DELAY);

  //-------------------------------
  // Sync-Field
  //-------------------------------
  USART_SendData(LIN_UART, LIN_SYNC_DATA);
  // wait until SyncField has been sent
  while (USART_GetFlagStatus(LIN_UART, USART_FLAG_TC) == RESET);

  // small pause
  p_LIN_wait_us(LIN_DATA_BYTE_DELAY); 

  //-------------------------------
  // ID-Field
  //-------------------------------
  USART_SendData(LIN_UART, frame->frame_id);
  // wait until IDField has been sent
  while (USART_GetFlagStatus(LIN_UART, USART_FLAG_TC) == RESET);

  // small pause
  p_LIN_wait_us(LIN_DATA_BYTE_DELAY);

  //-------------------------------
  // Prepare master
  //-------------------------------
  LIN_MASTER.mode=RECEIVE_DATA;
  LIN_MASTER.data_ptr=0;
  LIN_MASTER.crc=0;

  LIN_FRAME.data_len=frame->data_len;
  LIN_FRAME.data[0]=0; 

  //-------------------------------
  // wait until frame is received
  // or timeout
  //-------------------------------
  rx_timeout=0;
  n=0;
  do {   
    // timeout counter
    rx_timeout++;    
    if(rx_timeout>LIN_RX_TIMEOUT_CNT) {
      // leave the loop
      break;
    }
    // reset timeout, at data reception
    if(LIN_MASTER.data_ptr!=n) {
      n=LIN_MASTER.data_ptr;
      rx_timeout=0;
    }
  }while(LIN_MASTER.mode!=SEND_DATA);

  //-------------------------------
  // check if frame was received
  //-------------------------------
  if(LIN_MASTER.mode!=SEND_DATA) {
    // no frame received
    LIN_MASTER.mode=SEND_DATA;
    // small pause
    // so that the next frame is not sent too fast
    p_LIN_wait_us(LIN_INTER_FRAME_DELAY);
    return(LIN_RX_EMPTY);
  }  

  //-------------------------------
  // copy received data
  //-------------------------------
  for(n=0;n<frame->data_len;n++) {
    frame->data[n]=LIN_FRAME.data[n]; 
  }
  // calculate checksum
  checksum=p_LIN_makeChecksum(frame);

  //-------------------------------
  // check if crc ok
  //-------------------------------
  if(LIN_MASTER.crc!=checksum) {
    // checksum incorrect
    // small pause
    // so that the next frame is not sent too fast
    p_LIN_wait_us(LIN_INTER_FRAME_DELAY);
    return(LIN_WRONG_CRC);
  }
      
  //-------------------------------
  // data is ok
  //-------------------------------
  // small pause
  // so that the next frame is not sent too fast
  p_LIN_wait_us(LIN_INTER_FRAME_DELAY);

  return(LIN_OK);
}


//--------------------------------------------------------------
// internal function
// init the GPIOs
//--------------------------------------------------------------
void p_LIN_initGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //-----------------------
  // NSLP from the transceiver
  //-----------------------

  // Clock Enable
  RCC_AHB1PeriphClockCmd(LIN_NSLP_CLOCK, ENABLE);

  // Config as a digital output
  GPIO_InitStructure.GPIO_Pin = LIN_NSLP_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LIN_NSLP_PORT, &GPIO_InitStructure);

  // NSLP pin at Low level
  LIN_NSLP_PORT->BSRRH = LIN_NSLP_PIN;

  //-----------------------
  // RX + TX
  //-----------------------

  // Clock enable
  RCC_AHB1PeriphClockCmd(LIN_TX_CLOCK, ENABLE); // TX
  RCC_AHB1PeriphClockCmd(LIN_RX_CLOCK, ENABLE); // RX

  // GPIO PINs as an alternative function with PushPull
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  // TX
  GPIO_InitStructure.GPIO_Pin = LIN_TX_PIN;
  GPIO_Init(LIN_TX_PORT, &GPIO_InitStructure);
  // RX
  GPIO_InitStructure.GPIO_Pin = LIN_RX_PIN;
  GPIO_Init(LIN_RX_PORT, &GPIO_InitStructure);

  // connect alternative function with the IO pins
  GPIO_PinAFConfig(LIN_TX_PORT,LIN_TX_SOURCE,LIN_UART_AF);
  GPIO_PinAFConfig(LIN_RX_PORT,LIN_RX_SOURCE,LIN_UART_AF); 
}


//--------------------------------------------------------------
// internal function
// init from the UART
//--------------------------------------------------------------
void p_LIN_initUART(void)
{
  USART_InitTypeDef USART_InitStructure;

  // Clock enable
  if((LIN_UART==USART1) || (LIN_UART==USART6)) {
    RCC_APB2PeriphClockCmd(LIN_UART_CLOCK, ENABLE);
  }
  else {
    RCC_APB1PeriphClockCmd(LIN_UART_CLOCK, ENABLE);
  }

  // Oversampling not allowed with LIN (OVER8 = 0)

  // init with baud rate, 8 data bits, 1 stop bit, no parity, no RTS + CTS
  USART_InitStructure.USART_BaudRate = LIN_UART_BAUD;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(LIN_UART, &USART_InitStructure); 

  // UART enable
  USART_Cmd(LIN_UART, ENABLE); 

  // LIN enable
  USART_LINCmd(LIN_UART, ENABLE);
}


//--------------------------------------------------------------
// internal function
// init from NVIC
//--------------------------------------------------------------
void p_LIN_initNVIC(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  // RX-Interrupt enable
  USART_ITConfig(LIN_UART, USART_IT_RXNE, ENABLE);

  // enable Interrupt-Vector
  NVIC_InitStructure.NVIC_IRQChannel = LIN_UART_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


//--------------------------------------------------------------
// internal function
// LIN Transceiver (MAX13020)
// switch from "Sleep-Mode" to "Normal-Slope-Mode"
//--------------------------------------------------------------
void p_LIN_aktivateTransceiver(void)
{
  // NSLP pin at Low level
  LIN_NSLP_PORT->BSRRH = LIN_NSLP_PIN; // in "Sleep-Mode"
  // wait briefly (at least 10us)
  p_LIN_wait_us(LIN_AKTIV_DELAY);
  // NSLP pin to Hi level
  LIN_NSLP_PORT->BSRRL = LIN_NSLP_PIN; // in "Normal-Slope-Mode"
  // wait briefly (at least 10us)
  p_LIN_wait_us(LIN_AKTIV_DELAY);
}


// --------------------------------------------------------------
// internal function
// Calculate checksum over all data
// (classic-mode = inverted modulo256 sum)
//
// ret_value = checksum
//--------------------------------------------------------------
uint8_t p_LIN_makeChecksum(LIN_FRAME_t *frame)
{
  uint8_t ret_wert=0,n;
  uint16_t dummy;

  // calculate checksum  
  dummy=0;
  for(n=0;n<frame->data_len;n++) {
    dummy+=frame->data[n];
    if(dummy>0xFF) {
      dummy-=0xFF;
    } 
  }
  ret_wert=(uint8_t)(dummy);
  ret_wert^=0xFF;
  
  return(ret_wert);
}


// --------------------------------------------------------------
// internal function
// small break (without timer)
// 1 = about 1us
// 10 = about 10us
// 100 = about 100us
//--------------------------------------------------------------
void p_LIN_wait_us(uint32_t n)
{
  volatile uint32_t p,t;

  // small pause
  for(p=0;p<n;p++) {
    for(t=0;t<15;t++); // ca 1us
  }
}


// --------------------------------------------------------------
// internal function
// UART ISR
// is called when a LIN field has been received
// (all master TX-Fields are received as RX again)
//--------------------------------------------------------------
void LIN_UART_ISR(void) {
  uint16_t wert;

  if (USART_GetITStatus(LIN_UART, USART_IT_RXNE) == SET) { 
    // Read data
    wert=USART_ReceiveData(LIN_UART);

    // check which mode is currently active
    if(LIN_MASTER.mode==RECEIVE_DATA) {
      //---------------------------
      // DataField
      //---------------------------
      // Save data
      LIN_FRAME.data[LIN_MASTER.data_ptr]=(uint8_t)(wert);
      // switch pointers on
      LIN_MASTER.data_ptr++;
      // check if all data is received
      if(LIN_MASTER.data_ptr>=LIN_FRAME.data_len) {        
        LIN_MASTER.mode=RECEIVE_CRC;
      }
    }
    else if(LIN_MASTER.mode==RECEIVE_CRC) {
      //---------------------------
      // CRCField
      //---------------------------
      LIN_MASTER.crc=(uint8_t)(wert);
      LIN_MASTER.mode=SEND_DATA;
    }
  }
}

