/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <cmath>
#include <cstring>
#include <tuple>
#include "Matrix.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config();
static void MX_GPIO_Init();
static void MX_USART2_UART_Init();
static void send_string(const uint8_t* s);
static void receive_string(const uint8_t* s);
/* USER CODE END 4 */


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const double minControlValue = -0.5;

const double maxControlValue = 0.5;

typedef enum {
    success,
    failure
} result;

void send_char(char c)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&c, 1, 1000);
}

int __io_putchar(int ch)
{
    if (ch == '\n')
        send_char('\r');
    send_char(ch);
    return ch;
}

result calculateOptimizationMatrices(const CMatrix& A, const CVector& B, const CVector& C, const CVector& xk, const double r, CMatrix& H, CMatrix& W) {
    CMatrix fi(3, 3), Rw(3, 3);
    CVector F(3, 1), Rs(3,1);

    fi[0][0] = (C * B)[0][0];
    fi[1][0] = (C * A * B)[0][0];
    fi[1][1] = fi[0][0];
    fi[2][0] = (C * A * A * B)[0][0];
    fi[2][1] = fi[1][0];
    fi[2][2] = fi[1][1];

    Rw[0][0] = 1; Rw[1][1] = 1; Rw[2][2] = 1;

    F[0][0] = (C * A * xk)[0][0];
    F[1][0] = (C * A * A * xk)[0][0];
    F[2][0] = (C * A * A * A * xk)[0][0];

    Rs[0][0] = r; Rs[1][0] = r; Rs[2][0] = r;

    H = fi.T() * fi + Rw;
    W = fi.T() * (F - Rs);

    return success;
}

result calculateProjectedGradientStep(const CMatrix& H, const CMatrix& F, const CVector& xk, CVector& v, const double step) {
    uint32_t rows, columns;
    v.GetSize(rows, columns);
    CVector gradient(rows, 1);
    gradient = H * v + F;
    v -= gradient * step;
    for (uint32_t i = 0; i < rows; i++) {
        if (v[i][0] < minControlValue) {
            v[i][0] = minControlValue;
        }
        else if (v[i][0] > maxControlValue) {
            v[i][0] = maxControlValue;
        }
    }
    return success;
}
double fastGradientMethod(const CMatrix& A, const CVector& B, const CVector& C, const CVector& xk) {
    const uint32_t predictionHorizon = 3;
    const double r = 4.0, eps = 0.01;
    double temp[predictionHorizon] = {0, 0, 0};
    uint32_t rowsMatrixA, columnsMatrixA;
    A.GetSize(rowsMatrixA, columnsMatrixA);
    CVector v(predictionHorizon, 1, temp);
    CMatrix H(predictionHorizon, predictionHorizon), W(predictionHorizon, 1);
    CMatrix J(1,1), J_prev(1,1);

    calculateOptimizationMatrices(A, B, C, xk, r, H, W);
    for (uint32_t i = 0; i < 100; i++) {
        calculateProjectedGradientStep(H, W, xk, v, 0.1);
        J_prev = J;
        J = v.T() * H * v / 2 + v.T() * W;
        if (std::fabs(J_prev[0][0] - J[0][0]) < eps) {
            break;
        }
    }
    return v[0][0];
}

/*int main() {double temp_A[16] = {1, 0, 1,2 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1};
    double temp_B[4] = {0, 0, 1, 1}, temp_C[4] = {1, 1, 0, 0};

    CMatrix A(4, 4, temp_A);
    CVector B(4, 1, temp_B), C(4, temp_C);
    fastGradientMethod(A, B, C);
    return 0;
}*/

/* USER CODE END 0 */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
/**
  * @brief  The application entry point.
  * @retval int
  */
int main()
{
  /* USER CODE BEGIN 1 */
    uint8_t buf[1024];
    char* pBuf = reinterpret_cast<char*>(buf);
    uint8_t buf_size = 0;
    sprintf(reinterpret_cast<char*>(buf), "Some values: %f\n%f\n", 2.5, 3.333);
    double v = 0;

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
    //to poczeka
  //  HAL_UART_Receive(&huart2, &buf_size, 1, 10000);
  /* Infinite loop */
    double temp_A[16] = {1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1};
    double temp_B[4] = {0, 0, 1, 0}, temp_C[4] = {1, 1, 0, 0};
    CMatrix A(4, 4, temp_A);
    CVector B(4, 1, temp_B), C(4, temp_C), xk(4, 1);
    char* end;
    uint16_t iter = 0;
    while (true) {
        HAL_UART_Receive(&huart2, &buf_size, 1, 100);
        HAL_UART_Receive(&huart2, const_cast<uint8_t*>(buf), static_cast<uint16_t>(buf_size), 100);
        pBuf = reinterpret_cast<char*>(buf); //to doda³em
        xk[0][0] = std::strtod(pBuf, &end);
        for (iter = 1; iter < 4; iter++) {
            pBuf = end;
            xk[iter][0] = std::strtod(pBuf, &end);
        }
        v = fastGradientMethod(A, B, C, xk);
        sprintf(reinterpret_cast<char*>(buf), "%f\n", v); //do przetestowania co jes w buforze
//      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        send_string(buf);
//        HAL_Delay(100);
    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

#pragma clang diagnostic pop

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config()
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks
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
  /** Initializes the CPU, AHB and APB busses clocks
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init()
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

void send_string(const uint8_t* s)
{
    HAL_UART_Transmit(&huart2, const_cast<uint8_t *>(s), strlen(reinterpret_cast<const char*>(s)), 1000);
}

void receive_string(const uint8_t* s)
{
//    do {
    HAL_UART_Receive(&huart2, const_cast<uint8_t*>(s), 89, 100);
//    } while(*(ptr - 1) != '\n');
//    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
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
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
