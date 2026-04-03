/****************************************************************************
* Title                 :   
* Filename              :   blademotor.c
* Author                :   Nekraus
* Origin Date           :   17/08/2022
* Version               :   1.0.0

*****************************************************************************/
/** \file blademotor.c
*  \brief 
*
*/
/******************************************************************************
* Includes
*******************************************************************************/
#include <string.h>
#include <stdbool.h>

#include "stm32f_board_hal.h"

#include "main.h"
#include "board.h"

#include "blademotor.h" 

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define BLADEMOTOR_LENGTH_INIT_MSG 22
#define BLADEMOTOR_LENGTH_RQST_MSG 7
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/
typedef enum {
    BLADEMOTOR_INIT_1,
    BLADEMOTOR_INIT_2,
    BLADEMOTOR_RUN
}BLADEMOTOR_STATE_e;

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
UART_HandleTypeDef BLADEMOTOR_USART_Handler; // UART  Handle

DMA_HandleTypeDef hdma_uart_blade_rx;
DMA_HandleTypeDef hdma_uart_blade_tx;

static BLADEMOTOR_STATE_e blademotor_eState = BLADEMOTOR_INIT_1;

bool BLADEMOTOR_bActivated = false;
uint16_t BLADEMOTOR_u16RPM = 0;
uint16_t BLADEMOTOR_u16Power = 0;
uint32_t BLADEMOTOR_u32Error = 0;

static uint8_t blademotor_pu8ReceivedData[BLADEMOTOR_LENGTH_RECEIVED_MSG] = {0};
static uint8_t blademotor_pu8RqstMessage[BLADEMOTOR_LENGTH_RQST_MSG]  = {0x55, 0xaa, 0x03, 0x20, 0x80, 0x00, 0xA2};
static uint8_t blademotor_u8OnOff = 0;

const uint8_t blademotor_pcu8Preamble[5]  = {0x55,0xAA,0x0A,0x2,0xD0};
const uint8_t blademotor_pcu8InitMsg[BLADEMOTOR_LENGTH_INIT_MSG] =  { 0x55, 0xaa, 0x12, 0x20, 0x80, 0x00, 0xac, 0x0d, 0x00, 0x02, 0x32, 0x50, 0x1e, 0x04, 0x00, 0x15, 0x21, 0x05, 0x0a, 0x19, 0x3c, 0xaa };
/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
*  Public Functions
*******************************************************************************/

void blademotor_prepareMsg(void)
{    
    if (blademotor_u8OnOff)
    {
        blademotor_pu8RqstMessage[5] = 0x80; /* change speed Motor */
        blademotor_pu8RqstMessage[6] = 0x22; /* change CRC */
    }
    else
    {
        blademotor_pu8RqstMessage[5] = 0x00; /* change speed Motor */
        blademotor_pu8RqstMessage[6] = 0xa2; /* change CRC */
    }
}

/**
 * @brief Init the Blade Motor Serial Port (PAC5223)
 * @retval None
 */
void BLADEMOTOR_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PAC 5223 Reset Line (Blade Motor) */
    PAC5223RESET_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = PAC5223RESET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(PAC5223RESET_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(PAC5223RESET_GPIO_PORT, PAC5223RESET_PIN, 1);     /* take Blade PAC out of reset if HIGH */

    // enable port and usart clocks
    BLADEMOTOR_USART_GPIO_CLK_ENABLE();

	// Initiale USART3 for STM32f1 and USART6 for STM32f4
#if BOARD_YARDFORCE500_VARIANT_ORIG
	__HAL_RCC_USART3_CLK_ENABLE();
#elif BOARD_YARDFORCE500_VARIANT_B
	__HAL_RCC_USART6_CLK_ENABLE();
#endif
    
#if BOARD_YARDFORCE500_VARIANT_ORIG
    // RX
    GPIO_InitStruct.Pin = BLADEMOTOR_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(BLADEMOTOR_USART_RX_PORT, &GPIO_InitStruct);

    // TX
    GPIO_InitStruct.Pin = BLADEMOTOR_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(BLADEMOTOR_USART_TX_PORT, &GPIO_InitStruct);

    // Alternate Pin Set ?
    __HAL_AFIO_REMAP_USART2_ENABLE();
#elif BOARD_YARDFORCE500_VARIANT_B
    // RX TX
    GPIO_InitStruct.Pin = BLADEMOTOR_USART_TX_PIN | BLADEMOTOR_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(BLADEMOTOR_USART_TX_PORT, &GPIO_InitStruct);
#endif

    BLADEMOTOR_USART_Handler.Instance = BLADEMOTOR_USART_INSTANCE;
    BLADEMOTOR_USART_Handler.Init.BaudRate = 115200;               // Baud rate
    BLADEMOTOR_USART_Handler.Init.WordLength = UART_WORDLENGTH_8B; // The word is  8  Bit format
    BLADEMOTOR_USART_Handler.Init.StopBits = USART_STOPBITS_1;     // A stop bit
    BLADEMOTOR_USART_Handler.Init.Parity = UART_PARITY_NONE;       // No parity bit
    BLADEMOTOR_USART_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
    BLADEMOTOR_USART_Handler.Init.Mode = USART_MODE_TX_RX;         // Transceiver mode
    
    HAL_UART_Init(&BLADEMOTOR_USART_Handler); 

    DB_TRACE(" * Blade Motor UART initialized\r\n");

    /* UART4 DMA Init */
    /* UART4_RX Init */
#if BOARD_YARDFORCE500_VARIANT_ORIG
	hdma_uart_blade_rx.Instance = DMA1_Channel3;
#elif BOARD_YARDFORCE500_VARIANT_B
	hdma_uart_blade_rx.Instance = DMA2_Stream1;
	hdma_uart_blade_rx.Init.Channel = DMA_CHANNEL_5;
	hdma_uart_blade_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
#endif
	hdma_uart_blade_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_uart_blade_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_uart_blade_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_uart_blade_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_uart_blade_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_uart_blade_rx.Init.Mode = DMA_NORMAL;
	hdma_uart_blade_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_uart_blade_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&BLADEMOTOR_USART_Handler, hdmarx, hdma_uart_blade_rx);
    
    /* UART4 DMA Init */
    /* UART4_TX Init */
#if BOARD_YARDFORCE500_VARIANT_ORIG
	hdma_uart_blade_tx.Instance = DMA1_Channel2;
#elif BOARD_YARDFORCE500_VARIANT_B
	hdma_uart_blade_tx.Instance = DMA2_Stream6;
	hdma_uart_blade_tx.Init.Channel = DMA_CHANNEL_5;
	hdma_uart_blade_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
#endif
	hdma_uart_blade_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_uart_blade_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_uart_blade_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_uart_blade_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_uart_blade_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_uart_blade_tx.Init.Mode = DMA_NORMAL;
	hdma_uart_blade_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_uart_blade_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&BLADEMOTOR_USART_Handler, hdmatx, hdma_uart_blade_tx);
    
    // enable IRQ
#if BOARD_YARDFORCE500_VARIANT_ORIG
	IRQn_Type usart_irq = USART3_IRQn;
#elif BOARD_YARDFORCE500_VARIANT_B
	IRQn_Type usart_irq = USART6_IRQn;
#endif

    HAL_NVIC_SetPriority(usart_irq, 0, 0);
	HAL_NVIC_EnableIRQ(usart_irq);
    __HAL_UART_ENABLE_IT(&BLADEMOTOR_USART_Handler, UART_IT_TC);

    blademotor_eState = BLADEMOTOR_INIT_1;    
}

/// @brief handle drive motor messages
/// @param  
void  BLADEMOTOR_App(void){
    switch (blademotor_eState)
    {
    case BLADEMOTOR_INIT_1:

        HAL_UART_Transmit_DMA(&BLADEMOTOR_USART_Handler, (uint8_t*)blademotor_pcu8InitMsg, BLADEMOTOR_LENGTH_INIT_MSG);
        blademotor_eState = BLADEMOTOR_RUN;
        debug_printf(" * Blade Motor Controller initialized\r\n");     
        break;
    
    case BLADEMOTOR_RUN:

        /*error detected*/
        if(blademotor_pu8ReceivedData[6] != 0){
            blademotor_u8OnOff = 0;
            BLADEMOTOR_u32Error++;
        }
        blademotor_prepareMsg();
        /* prepare to receive the message before to launch the command */        
        HAL_UART_Receive_DMA(&BLADEMOTOR_USART_Handler, blademotor_pu8ReceivedData, BLADEMOTOR_LENGTH_RECEIVED_MSG);
                  
        HAL_UART_Transmit_DMA(&BLADEMOTOR_USART_Handler, (uint8_t*)blademotor_pu8RqstMessage, BLADEMOTOR_LENGTH_RQST_MSG);    
        break;
    
    default:
        break;
    }
}

/// @brief control blade motor (there is no speed control for this motor)
/// @param on_off 1 to turn on, 0 to turn off
void BLADEMOTOR_Set(uint8_t on_off, uint8_t direction)
{       
    blademotor_u8OnOff = on_off;
    if (on_off)
    {
        /*if (direction) {
            blademotor_pu8RqstMessage[5] = 0xC0;
            blademotor_pu8RqstMessage[6] = 0xE2;
        } else {*/
            blademotor_pu8RqstMessage[5] = 0x80; /* change speed Motor */
            blademotor_pu8RqstMessage[6] = 0x22; /* change CRC */
        //}
    }
    else
    {
        blademotor_pu8RqstMessage[5] = 0x00; /* change speed Motor */
        blademotor_pu8RqstMessage[6] = 0xa2; /* change CRC */
    }
}

/// @brief drive motor receive interrupt handler
/// @param  
void BLADEMOTOR_ReceiveIT(void)
{
    /* decode the frame */    
    if(memcmp(blademotor_pcu8Preamble, blademotor_pu8ReceivedData, 2) == 0){        
        uint8_t l_u8crc = crcCalc(blademotor_pu8ReceivedData, BLADEMOTOR_LENGTH_RECEIVED_MSG-1);

        if(blademotor_pu8ReceivedData[BLADEMOTOR_LENGTH_RECEIVED_MSG-1] == l_u8crc ){
            if((blademotor_pu8ReceivedData[5] & 0x80) == 0x80){
                BLADEMOTOR_bActivated = true;
            }
            else{
                BLADEMOTOR_bActivated = false;
            }
            BLADEMOTOR_u16RPM = blademotor_pu8ReceivedData[7] + (blademotor_pu8ReceivedData[8]<<8);
            BLADEMOTOR_u16Power = blademotor_pu8ReceivedData[9] + (blademotor_pu8ReceivedData[10]<<8) ;           
        }
  
    }
}

/******************************************************************************
*  Private Functions
*******************************************************************************/
