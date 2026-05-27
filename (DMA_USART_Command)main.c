/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — UART DMA Command Processor (Fixed)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 128   /* Increased for safety */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
DMA_HandleTypeDef hdma_memtomem_dma2_stream1;

uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t tx_buffer[TX_BUFFER_SIZE];

/* FIX 1: Store the actual command byte separately from command_index.
   The callbacks were zeroing command_index before main() could read it. */
volatile uint8_t command_ready = 0;
volatile uint8_t pending_command = 0;   /* Stores the actual command char */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void process_command(uint8_t cmd);
void UART_SendString(const char* str);

/* FIX 2: Shared helper — parse one byte from rx_buffer at index i.
   Called from both half and full callbacks to avoid code duplication.
   Uses a local static index so it persists across half/full boundaries. */
static void parse_rx_byte(uint8_t byte)
{
    /* Only accept printable digit commands and CR/LF as terminator */
    if (byte >= '0' && byte <= '9')
    {
        /* Save first received digit as the pending command */
        if (!command_ready)
        {
            pending_command = byte;
        }
    }
    else if (byte == '\r' || byte == '\n')
    {
        /* CR or LF = end of command — signal main loop */
        if (pending_command != 0)
        {
            command_ready = 1;
        }
    }
    /* Ignore null bytes (0x00) that fill the DMA buffer initially */
}

/* USER CODE BEGIN 0 */

/* FIX 3: Reliable blocking TX using polling instead of DMA busy-wait.
   DMA TX for short strings causes HAL state to lag; polling is simpler
   and safe for short response strings. Switch back to DMA TX only if
   you need non-blocking sends for large data. */
void UART_SendString(const char* str)
{
    uint16_t len = (uint16_t)strlen(str);
    if (len == 0) return;

    /* HAL_UART_Transmit is blocking — safe to call from main context */
    HAL_UART_Transmit(&huart2, (uint8_t*)str, len, 100 /* ms timeout */);
}

void process_command(uint8_t cmd)
{
    char response[64];

    switch (cmd)
    {
        case '0':
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            UART_SendString("\r\nLED turned OFF\r\n");
            break;

        case '1':
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            UART_SendString("\r\nLED turned ON\r\n");
            break;

        case '2':
        	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            UART_SendString("\r\nLED toggled\r\n");
            break;

        default:
            snprintf(response, sizeof(response),
                     "\r\nInvalid command '%c'! Use 0, 1, or 2\r\n", cmd);
            UART_SendString(response);
            break;
    }
}

/* FIX 4: Half-complete callback — process bytes 0..(N/2 - 1) only */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        for (int i = 0; i < RX_BUFFER_SIZE / 2; i++)
        {
            parse_rx_byte(rx_buffer[i]);
        }
    }
}

/* FIX 5: Full-complete callback — process bytes (N/2)..(N-1) only.
   DMA circular mode restarts automatically — no manual restart needed. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        for (int i = RX_BUFFER_SIZE / 2; i < RX_BUFFER_SIZE; i++)
        {
            parse_rx_byte(rx_buffer[i]);
        }
    }
}

/* FIX 6: Error callback — clear flags and restart DMA */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);   /* Overrun */
        __HAL_UART_CLEAR_NEFLAG(huart);    /* Noise */
        __HAL_UART_CLEAR_FEFLAG(huart);    /* Framing */
        __HAL_UART_CLEAR_PEFLAG(huart);    /* Parity */

        /* Abort any ongoing DMA transfer before restarting */
        HAL_UART_DMAStop(huart);
        HAL_UART_Receive_DMA(&huart2, rx_buffer, RX_BUFFER_SIZE);
    }
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_DMA_Init();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    /* Startup message */
    UART_SendString("\r\n=== UART Command Processor ===\r\n");
    UART_SendString("Commands: 0=OFF, 1=ON, 2=TOGGLE\r\n");
    UART_SendString("Enter command followed by ENTER: ");

    /* Start DMA circular reception */
    if (HAL_UART_Receive_DMA(&huart2, rx_buffer, RX_BUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable UART error interrupts */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);

    while (1)
    {
        /* FIX 7: Read command_ready and pending_command atomically.
           Disable IRQ briefly to avoid a race where the ISR updates
           pending_command between our check and our read. */
        if (command_ready)
        {
            __disable_irq();
            uint8_t cmd = pending_command;
            pending_command  = 0;
            command_ready    = 0;
            __enable_irq();

            process_command(cmd);
            UART_SendString("Enter command followed by ENTER: ");
        }

        /* FIX 8: Removed HAL_Delay(100) — it prevents timely command detection.
           If you need to do background work, keep it short or use a flag. */
    }
}

/* System Clock, UART, DMA, GPIO init — unchanged from your original ----------*/

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* mem-to-mem stream 0 */
    hdma_memtomem_dma2_stream0.Instance                 = DMA2_Stream0;
    hdma_memtomem_dma2_stream0.Init.Channel             = DMA_CHANNEL_0;
    hdma_memtomem_dma2_stream0.Init.Direction           = DMA_MEMORY_TO_MEMORY;
    hdma_memtomem_dma2_stream0.Init.PeriphInc           = DMA_PINC_ENABLE;
    hdma_memtomem_dma2_stream0.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_memtomem_dma2_stream0.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_memtomem_dma2_stream0.Init.Mode                = DMA_NORMAL;
    hdma_memtomem_dma2_stream0.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_memtomem_dma2_stream0.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_memtomem_dma2_stream0.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_memtomem_dma2_stream0.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_memtomem_dma2_stream0.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK) Error_Handler();

    /* mem-to-mem stream 1 */
    hdma_memtomem_dma2_stream1.Instance                 = DMA2_Stream1;
    hdma_memtomem_dma2_stream1.Init.Channel             = DMA_CHANNEL_0;
    hdma_memtomem_dma2_stream1.Init.Direction           = DMA_MEMORY_TO_MEMORY;
    hdma_memtomem_dma2_stream1.Init.PeriphInc           = DMA_PINC_ENABLE;
    hdma_memtomem_dma2_stream1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_memtomem_dma2_stream1.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_memtomem_dma2_stream1.Init.Mode                = DMA_NORMAL;
    hdma_memtomem_dma2_stream1.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_memtomem_dma2_stream1.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_memtomem_dma2_stream1.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_memtomem_dma2_stream1.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_memtomem_dma2_stream1.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_memtomem_dma2_stream1) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = LED_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
