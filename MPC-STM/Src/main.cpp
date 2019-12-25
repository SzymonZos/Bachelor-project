#include "main.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <map>
#include <vector>
#include <regex>
#include "Matrix.h"

UART_HandleTypeDef huart2;

void SystemClock_Config();
static void MX_GPIO_Init();
static void MX_USART2_UART_Init();
static void send_string(const uint8_t* s);

double minControlValue = -0.5;
double maxControlValue = 0.5;

typedef enum {
    success,
    failure
} result;

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

double fastGradientMethod(const CMatrix& A, const CVector& B, const CVector& C, const CVector& xk, double r) {
    const uint32_t predictionHorizon = 3;
    double eps = 0.01;
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

void stringToDouble(const std::string& data_reference, std::vector<double>& data_to_fill) {
    const char* begin = nullptr;
    char* end = const_cast<char*>(data_reference.c_str());
    double value;
    do {
        begin = end;
        value = std::strtod(begin, &end);
        if (begin != end) {
            data_to_fill.push_back(value);
        }
    } while(begin != end);
}

template<typename __Map>
void print_map(const __Map& m)
{
    std::cout << "{";
    for(const auto& p : m) {
        std::cout << '\'' << p.first << "': [";
        for(const auto& v : p.second) {
            std::cout << v << ", ";
        }
        std::cout << "\b\b], ";
    }
    std::cout << "\b\b}\n";
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

int main() {
    uint8_t buf[1024];
    char* pBuf;
    uint8_t buf_size = 0;
    double v = 0;

    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    // 20s wait after STM reboot to read data sent from PC
    HAL_UART_Receive(&huart2, &buf_size, 1, 20000);
    HAL_UART_Receive(&huart2, const_cast<uint8_t*>(buf), buf_size, 100);
//    sprintf(reinterpret_cast<char*>(buf), "'A': [1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1], 'B': [0, 0, 1, 0], 'C': [1, 1, 0, 0], 'setPoint': [1], 'controlExtremeValues': [-5, 5]");

    std::string pythonString = reinterpret_cast<char*>(buf), valuesMatch;
    std::regex pattern(R"(([[:alpha:]]+)(': )(\[.+?\]))");
    std::smatch fullMatch;
    std::string::const_iterator iterator(pythonString.cbegin());
    std::map<std::string, std::vector<double>> dict;

    while(std::regex_search(iterator, pythonString.cend(), fullMatch, pattern)) {
        valuesMatch = fullMatch[3].str();
        std::replace(valuesMatch.begin(), valuesMatch.end(), ',', ' ');
        valuesMatch.pop_back(); // trim ]
        valuesMatch.erase(0, 1); // trim [
        if(fullMatch[1].str().find('A') != std::string::npos) {
            stringToDouble(valuesMatch, dict["A"]);
        }
        else if(fullMatch[1].str().find('B') != std::string::npos) {
            stringToDouble(valuesMatch, dict["B"]);
        }
        else if(fullMatch[1].str().find('C') != std::string::npos) {
            stringToDouble(valuesMatch, dict["C"]);
        }
        else if(fullMatch[1].str().find("set") != std::string::npos) {
            stringToDouble(valuesMatch, dict["set"]);
        }
        else if(fullMatch[1].str().find("control") != std::string::npos) {
            stringToDouble(valuesMatch, dict["control"]);
        }
        iterator = fullMatch.suffix().first;
    }

    size_t dimension = static_cast<size_t>(std::sqrt(dict["A"].size()));
    CMatrix A(dimension, dimension, dict["A"].data());
    CVector B(dict["B"].size(), 1, dict["B"].data());
    CVector C(dict["C"].size(), dict["C"].data());
    CVector xk(dict["C"].size(), 1);
    double w = dict["set"][0];
    minControlValue = dict["control"][0];
    maxControlValue = dict["control"][1];
    char* end;
    uint16_t iter = 0;
    sprintf(reinterpret_cast<char*>(buf), "buf size: %u, w: %f, min: %f, max: %f, A: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", buf_size, w, minControlValue, maxControlValue, A[0][0], A[0][1], A[0][2], A[0][3], A[1][0], A[1][1], A[1][2], A[1][3], A[2][0], A[2][1], A[2][2], A[2][3], A[3][0], A[3][1], A[3][2], A[3][3]);
    send_string(buf);

    while (true) {
        HAL_UART_Receive(&huart2, &buf_size, 1, 1000);
        HAL_UART_Receive(&huart2, const_cast<uint8_t*>(buf), buf_size, 100);
        pBuf = reinterpret_cast<char*>(buf);
        xk[0][0] = std::strtod(pBuf, &end);
        for (iter = 1; iter < 4; iter++) {
            pBuf = end;
            xk[iter][0] = std::strtod(pBuf, &end);
        }
        v = fastGradientMethod(A, B, C, xk, w);
        sprintf(reinterpret_cast<char*>(buf), "%f\n", v);
        send_string(buf);
    }
}

#pragma clang diagnostic pop

void SystemClock_Config() {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    // Initializes the CPU, AHB and APB busses clocks
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init() {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure GPIO pin Output Level
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    // Configure GPIO pin : B1_Pin
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    //Configure GPIO pin : LD2_Pin
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void){}

void send_string(const uint8_t* s) {
    HAL_UART_Transmit(&huart2, const_cast<uint8_t *>(s), strlen(reinterpret_cast<const char*>(s)), 1000);
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
