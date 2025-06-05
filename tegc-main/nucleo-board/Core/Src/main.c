/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "pwm.h"
#include "tegc_config.h"
#include "temperature_controller.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

const char error_msg[] = "I am in error handler\r\n";

/***
 * Current temperatures of both TEG sides
 */
float temp_hot_side;
float temp_cold_side;

/***
 * Temperature controllers for the 2 sides of the TEG,
 * the hot side with the heating resistor,
 * the cold side using the fan
 */
temperature_controller_t tc_hot_side;
temperature_controller_t tc_cold_side;


/***
 * IMPORTANT!!! MAKE SURE TO SELECT THE APPROPRIATE VALUE IN THE MICROCURRENT MODULE.
 */
#define U_CURRENT_CONVERSION (1e-3 / 1e-6)  // 1mV / 1uA

/***
 * The buffer size is 2 because one value is for the open circuit voltage 
 * and the other one for the current measurement
 */
#define TEG_ADC_DMA_BUFFER_SIZE (2)
bool teg_adc_dma_complete = false;
uint32_t teg_adc_value[TEG_ADC_DMA_BUFFER_SIZE] = {0};
float teg_adc_voltage[TEG_ADC_DMA_BUFFER_SIZE] = {0};
float teg_open_circuit_voltage = 0.0f;
float teg_load_current = 0.0f;

/***
 * Coefficient for the weighted average, used to filter out spikes during ADC acquisitions.
 * The value should be between 0 and 1, the closer to 0 the more filtering is applied.
 * a value of 0.1 is equivalent fo doing a moving average of 10 samples. 0.001 is of 1000 samples.
 */
float teg_filter_coefficient = 0.001;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char uart_rx_buf[BUFSIZ] = {0};
char uart_line_buf[BUFSIZ] = {0};
uint32_t uart_rx_index = 0;
bool uart_line_received = false;
bool firstCsvLine = true;

bool cmd_compare(const char *cmd, const char *line) {
  return strncmp(cmd, line, strlen(cmd)) == 0;
}

/** Parse and processes UART commands */
void parse_uart_line(const char *line) {
  // sets the Hot Side surface temperature setpoint.
  // The PID controller will try to keep the temperature at this value
  if (cmd_compare(">setpoint", line)) {
    float setpoint;
    sscanf(line, ">setpoint %f", &setpoint);
    tc_set_temperature(&tc_hot_side, setpoint);
    return;
  }
  // Changes PID proportional constant to the received value.
  if (cmd_compare(">pid kp", line)) {
    sscanf(line, ">pid kp %f", &tc_hot_side.pid_controller.kp);
    return;
  }
  // Changes PID integral constant to the received value.
  if (cmd_compare(">pid ki", line)) {
    sscanf(line, ">pid ki %f", &tc_hot_side.pid_controller.ki);
    return;
  }
  // Changes PID derivative constant to the received value.
  if (cmd_compare(">pid kd", line)) {
    sscanf(line, ">pid kd %f", &tc_hot_side.pid_controller.kd);
    return;
  }
  // Changes PID constants to the received values.
  if (cmd_compare(">pid", line)) {
    sscanf(line, ">pid %f %f %f", &tc_hot_side.pid_controller.kp,
           &tc_hot_side.pid_controller.ki, &tc_hot_side.pid_controller.kd);
    return;
  }
  // Changes the duty cycle of the hot side heating element
  if (cmd_compare(">duty", line)) {
    float duty;
    sscanf(line, ">duty %f", &duty);
    tc_hot_side.commaded_duty_cycle = duty;
    return;
  }
  // Clears the terminal (prints 50 new lines)
  if (cmd_compare("clear", line)) {
    for (int i = 0; i < 50; i++) {
      print("\r\n");
    }
    return;
  }
  // Resets the microcontroller
  if (cmd_compare("reset", line)) {
    HAL_NVIC_SystemReset();
    return;
  }
  // Resend the CSV header
  if (cmd_compare("csv_header", line)) {
    firstCsvLine = true;
    return;
  }
  // Prints the current PID constants
  if (cmd_compare("pid", line)) {
    print("kp: %f, ki: %f, kd: %f\r\n", tc_hot_side.pid_controller.kp,
          tc_hot_side.pid_controller.ki, tc_hot_side.pid_controller.kd);
    return;
  }
  // Prints the current setpoint
  if (cmd_compare("setpoint", line)) {
    print("setpoint: %f\r\n", tc_hot_side.pid_controller.set_point);
    return;
  }

  // If the command is not recognized, print the help message
  print(
      "Unknown command: \"%s\"\r\n"
      ">temp <cold_side> <hot_side>\r\n"
      ">pid <kp> <ki> <kd>\r\n"
      ">duty <duty cycle>\r\n"
      "clear\r\n"
      "pid\r\n"
      "setpoint\r\n",
      line);
}


/***
 * This macro technique is called X-Macros (https://en.wikipedia.org/wiki/X_macro)
 * Basically it allows to define a list elements, each element is a call to a macro
 * yet to define. The trick is to define a macro (CSV_ROW) different each time, so that
 * in different parts of the code, the same list of elements can be used to generate
 * different code.
 * 
 * In this case, CSV_ROW has 3 arguments, the name of the column, the format specifier
 * and the value to be printed.
 */
#define CSV_COLUMNS()                                                       \
  CSV_ROW("time", "%" PRIu32, HAL_GetTick())                                \
  CSV_ROW("temp_hot_side", "%f", temp_hot_side)                             \
  CSV_ROW("temp_cold_side", "%f", temp_cold_side)                           \
  CSV_ROW("temp_delta", "%f", temp_hot_side - temp_cold_side)               \
  CSV_ROW("hot_side_set_point", "%f", tc_hot_side.pid_controller.set_point) \
  CSV_ROW("hot_side_pid_error", "%f", tc_hot_side.pid_controller.error)     \
  CSV_ROW("hot_side_duty_cycle", "%f", tc_hot_side.commaded_duty_cycle)     \
  CSV_ROW("teg_oc_voltage", "%f", teg_open_circuit_voltage)                 \
  CSV_ROW("teg_current", "%0.9f", teg_load_current)

void write_csv() {
  if (firstCsvLine) {

    firstCsvLine = false;
    print("csv_header:");
    // Define CSV_ROW such that it prints the name
#define CSV_ROW(name, format, value) print(name ",");
    // Call CSV_COLUMNS to expand the list of CSV_ROW calls
    CSV_COLUMNS();
    // Undefine CSV_ROW to avoid conflicts with other uses of the macro
#undef CSV_ROW
    print("\r\n");

  } else {

    print("csv_data:");
    // Define CSV_ROW such that it prints the value
#define CSV_ROW(name, format, value) print(format ",", value);
    // Call CSV_COLUMNS to expand the list of CSV_ROW calls
    CSV_COLUMNS();
    // Undefine CSV_ROW to avoid conflicts with other uses of the macro
#undef CSV_ROW
    print("\r\n");

  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  // Timer used to sample MAX6675 temperatures at specified frequency
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // Start listening for commands via UART
  setup_uart_dma();
  // Start adc acquisitions
  setup_adc_dma();

  /***
   * Initialization of the temperature controllers
   * The hot side controller has inverted duty cycle
   * due to the current control circuit, see schematics for more info
   */
  tc_init(&tc_hot_side, &htim2, TIM_CHANNEL_1, true);
  tc_init(&tc_cold_side, &htim3, TIM_CHANNEL_1, false);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int t = 0;
  int pid_t = 0;
  int adc_t = 0;
  write_csv();
  while (1) {

    // Every 100ms send in UART the CSV line
    if (HAL_GetTick() - t >= 100) {
      t = HAL_GetTick();
      write_csv();
    }
    // If DMA compleated and at least at 1kHz, start another conversion
    if (teg_adc_dma_complete && HAL_GetTick() - adc_t >= 1) {
      // the adc has completed the previous conversion and can start another one
      setup_adc_dma();
      adc_t = HAL_GetTick();
    }

    // Every 400ms update the PID controller
    if (HAL_GetTick() - pid_t >= 400) {
      pid_t = HAL_GetTick();
      tc_update(&tc_hot_side, temp_hot_side);
      if (tc_hot_side.pid_controller.integrator < 0) {
        tc_hot_side.pid_controller.integrator = 0;
      }

      // This controls the fan speed
      set_pwm_duty_cycle(&htim3, TIM_CHANNEL_1, 0.5);
    }

    // If a line is received via UART, parse it
    if (uart_line_received) {
      uart_line_received = false;
      parse_uart_line(uart_line_buf);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

/* USER CODE BEGIN 4 */

void setup_uart_dma() {
  if (uart_rx_index >= BUFSIZ) {
    uart_rx_index = 0;
  }
  // DMA is configured to receive one character at the time, that is then copied to the line buffer
  if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)(&uart_rx_buf[uart_rx_index]),
                           1) != HAL_OK) {
    Error_Handler();
  }
  uart_rx_index++;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
#define DELIMITER_LEN (2)
  const char *delimeters = "\r\n";
  if (huart->Instance == USART2) {
    for (int i = 0; i < DELIMITER_LEN; i++) {
      /***
       * if a line termination character is identified,
       * save the current line to be processed and start a new line
       */ 
      if (uart_rx_buf[uart_rx_index - 1] == delimeters[i]) {
        memset(uart_line_buf, 0, sizeof(uart_line_buf));
        uart_line_received = true;
        uart_rx_buf[uart_rx_index - 1] = '\0';
        memcpy(uart_line_buf, uart_rx_buf, uart_rx_index - 1);
        uart_rx_index = 0;
        break;
      }
    }
    // continue to monitor the UART
    setup_uart_dma();
  }
}

void setup_adc_dma() {
  // start the ADC conversion via DMA
  if (HAL_ADC_Start_DMA(&hadc1, teg_adc_value, TEG_ADC_DMA_BUFFER_SIZE) !=
      HAL_OK) {
    Error_Handler();
  }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    teg_adc_dma_complete = true;

    // For each value in the buffer, convert from bits to voltage (assuming 3V3 reference)
    // Apply moving average filter to the values
    for (size_t i = 0; i < TEG_ADC_DMA_BUFFER_SIZE; i++) {
      float sampled_voltage = teg_adc_value[i] * 3.3f / 4096.0f;
      teg_adc_voltage[i] = teg_adc_voltage[i] * (1 - teg_filter_coefficient) +
                           teg_filter_coefficient * sampled_voltage;
    }
    // The first value is the open circuit voltage, the second is the current
    teg_open_circuit_voltage = teg_adc_voltage[0];
    teg_load_current = teg_adc_voltage[1] / U_CURRENT_CONVERSION;
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /***
   * A timer is used to reading 
   * the max6675 temperatures at a desired frequency.
   * max6675 has a maximum allowed frequency, so make sure
   * to not exceed it.
   */
  static int sampleTimer = 0;
  static uint8_t sampleIndex = 0;
  if (htim->Instance == TIM3) {
    if (HAL_GetTick() - sampleTimer >= 100) {
      sampleTimer = HAL_GetTick();
      sampleIndex++;
      
      if (sampleIndex % 3 == 0){
        temp_hot_side =
            temp_hot_side * 0.5 +
            0.5 * max6675_read_temp(SPI_CS_1_GPIO_Port, SPI_CS_1_Pin);
      }
      if (sampleIndex % 3 == 1) {
        temp_cold_side = max6675_read_temp(SPI_CS_2_GPIO_Port, SPI_CS_2_Pin);
      }
    }
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  
  // __disable_irq(); // WARNING: irq is not disabled so HAL_Delay() works correctly
  char msg_buf[BUFSIZ] = {0};
  while (1) {
    snprintf((char *)msg_buf, sizeof(msg_buf), "Error_Handler: %" PRIu32 "\r\n",
             huart2.hdmarx->ErrorCode);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg_buf, sizeof(msg_buf), 100);
    HAL_UART_Transmit(&huart2, (uint8_t *)error_msg, sizeof(error_msg), 100);

    HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
   number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
   line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
