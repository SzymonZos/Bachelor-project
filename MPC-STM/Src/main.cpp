#include "HAL/main.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <map>
#include <vector>
#include <regex>
#include <random>
#include <tuple>
#include "utils/Matrix.hpp"
#include "HAL/Peripherals.hpp"

UART_HandleTypeDef huart2;

void SystemClock_Config();
static void MX_GPIO_Init();
static void MX_USART2_UART_Init();
static void send_string(const uint8_t* s);

// TODO: move this atrocity to class
double minControlValue = -0.5;
double maxControlValue = 0.5;
double w = 4;
uint8_t buf[1024];
uint16_t xk_size;
static bool isNewDataGoingToBeSend;
uint32_t prediction_horizon = 10;
uint32_t control_horizon = 4;
double L, mi, step, eigen_const;


double power_iteration(const CMatrix& matrix, uint32_t max_number_of_iterations) {
    double greatest_eigenvalue_current = 0, greatest_eigenvalue_previous;
    std::random_device rd;
    std::mt19937 gen(rd());
    CVector b_k(matrix.GetRows(), 1), b_k1(matrix.GetRows(), 1);
    for (uint32_t i = 0; i < matrix.GetRows(); i++) {
        b_k[i][0] = std::generate_canonical<double, 10>(gen);
    }
    for (uint32_t i = 0; i < max_number_of_iterations; i++) {
        b_k1 = matrix * b_k;
        b_k = b_k1 / std::sqrt((b_k1 * b_k1.T())[0][0]);
        greatest_eigenvalue_previous = greatest_eigenvalue_current;
        greatest_eigenvalue_current = ((matrix * b_k) * b_k.T())[0][0] / (b_k * b_k.T())[0][0];
        if (std::fabs(greatest_eigenvalue_current - greatest_eigenvalue_previous) < 0.01) {
            break;
        }
    }
    return std::fabs(greatest_eigenvalue_current);
}

std::tuple<CMatrix, CMatrix, CMatrix, CVector> calculateOptimizationMatrices(const CMatrix& A, const CVector& B, const CVector& C, const double r) {
    double R1 = 1;
    CMatrix fi(prediction_horizon, control_horizon), Rw(control_horizon, "eye");
    CVector product_matrix(C.GetColumns());
    Rw *= R1;
    product_matrix = C * (A ^ (prediction_horizon - control_horizon));
    for (uint32_t i = prediction_horizon; i != 0; i--) {
        for (uint32_t j = control_horizon; j != 0; j--) {
            if (i == prediction_horizon) {
                if (j < control_horizon) {
                    product_matrix *= A;
                }
                fi[i-1][j-1] = (product_matrix * B)[0][0];
            } else if (i < j && j < control_horizon) {
                fi[i-1][j-1] = fi[i][j];
            }
        }
    }

    CMatrix F(prediction_horizon, C.GetColumns());
    CVector Rs(prediction_horizon, 1, std::to_string(r));
    product_matrix();
    product_matrix = C * A;
    for (uint32_t i = 0; i < prediction_horizon; i++) {
        F.SetRow(i, product_matrix);
        product_matrix *= A;
    }
    return {fi, Rw, F, Rs};
}

void calculateProjectedGradientStep(CVector& v, CVector& w_v, const CMatrix& H, const CMatrix& W) {
    uint32_t rows, columns;
    v.GetSize(rows, columns);
    CVector gradient(rows, 1), v_old = v;
    gradient = H * v + W;
    w_v = v - gradient * step;
    for (uint32_t i = 0; i < rows; i++) {
        if (w_v[i][0] < minControlValue) {
            v[i][0] = minControlValue;
        }
        else if (w_v[i][0] > maxControlValue) {
            v[i][0] = maxControlValue;
        } else {
            v[i][0] = w_v[i][0];
        }
    }
    w_v = v + (v - v_old) * eigen_const;
}

double fastGradientMethod(const CMatrix& H, const CMatrix& fi, const CMatrix& F, const CVector& xk, const CVector& Rs, CMatrix& W) {
    CVector v(control_horizon, 1), w_v = v;
    CMatrix J(1,1), J_prev(1,1);
    const double eps = 0.01;

    for (uint32_t i = 0; i < 100; i++) {
        W = fi.T() * ((F * xk) - Rs);
        calculateProjectedGradientStep(v, w_v, H, W);
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

void receive_data(uint16_t timeout) {
    uint8_t buf_size = 0;
    HAL_UART_Receive(&huart2, &buf_size, 1, timeout);
    HAL_UART_Receive(&huart2, const_cast<uint8_t*>(buf), buf_size, 100);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == B1_Pin) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        isNewDataGoingToBeSend = true;
    }
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

int main() {
    char* pBuf, *end;
    uint8_t buf_size = 0;
    uint16_t iter = 0;
    double v = 0;
    std::string pythonString, valuesMatch;
    std::regex pattern(R"(([[:alpha:]]+)(': )(\[.+?\]))");
    std::smatch fullMatch;
    std::string::const_iterator iterator;
    std::map<std::string, std::vector<double>> dict;
    size_t dimension;
    CMatrix A(4, 4);
    CVector B(4, 1), C(4), xk(4, 1);
    CMatrix H(control_horizon, control_horizon), W(control_horizon, 1);
    CMatrix fi(prediction_horizon, control_horizon), Rw(control_horizon, "eye");
    CMatrix F(prediction_horizon, C.GetColumns());
    CVector Rs(prediction_horizon, 1, std::to_string(1));

    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL::Peripherals::GetInstance().Init();

    while (true) {
        if (isNewDataGoingToBeSend) {
            isNewDataGoingToBeSend = false;
            // 20s wait after pressing button to read data sent from PC
            HAL::Peripherals::GetInstance().receiveString(buf, 20000);
            if (*buf == '\'') {
                pythonString = reinterpret_cast<char*>(buf);
                iterator = pythonString.cbegin();
                while (std::regex_search(iterator, pythonString.cend(), fullMatch, pattern)) {
                    valuesMatch = fullMatch[3].str();
                    std::replace(valuesMatch.begin(), valuesMatch.end(), ',', ' ');
                    valuesMatch.pop_back(); // trim ]
                    valuesMatch.erase(0, 1); // trim [
                    if (fullMatch[1].str().find('A') != std::string::npos) {
                        stringToDouble(valuesMatch, dict["A"]);
                    } else if (fullMatch[1].str().find('B') != std::string::npos) {
                        stringToDouble(valuesMatch, dict["B"]);
                    } else if (fullMatch[1].str().find('C') != std::string::npos) {
                        stringToDouble(valuesMatch, dict["C"]);
                    } else if (fullMatch[1].str().find("set") != std::string::npos) {
                        stringToDouble(valuesMatch, dict["set"]);
                    } else if (fullMatch[1].str().find("control") != std::string::npos) {
                        stringToDouble(valuesMatch, dict["control"]);
                    } else if (fullMatch[1].str().find("horizon") != std::string::npos) {
                        stringToDouble(valuesMatch, dict["horizons"]);
                    }
                    iterator = fullMatch.suffix().first;
                }
                dimension = static_cast<size_t>(std::sqrt(dict["A"].size()));
                xk_size = dict["C"].size();
                A(dimension, dimension, dict["A"].data());
                B(dict["B"].size(), 1, dict["B"].data());
                C(1, xk_size, dict["C"].data());
                xk(xk_size, 1);
                w = dict["set"][0];
                minControlValue = dict["control"][0];
                maxControlValue = dict["control"][1];
                prediction_horizon = dict["horizons"][0];
                control_horizon = dict["horizons"][1];
                F(prediction_horizon, xk_size);
                fi(prediction_horizon, control_horizon);
                Rw(control_horizon, control_horizon, "eye");
                Rs(prediction_horizon, 1);
                H(control_horizon, control_horizon);
                W(control_horizon, 1);
                std::tie(fi, Rw, F, Rs) = calculateOptimizationMatrices(A, B, C, w);
                H = fi.T() * fi + Rw;
                HAL::Peripherals::GetInstance().sendString(buf, 100);
                L = power_iteration(H, 20), mi = power_iteration(H.Inverse(), 20);
                step = 1 / L, eigen_const = (std::sqrt(L) - std::sqrt(mi)) / (std::sqrt(L) + std::sqrt(mi));

                dict.clear();
//                sprintf(reinterpret_cast<char *>(buf), "%f %f %f\n", w, minControlValue, maxControlValue);
            }
        }
        else {
            HAL::Peripherals::GetInstance().receiveString(buf, 100);
            pBuf = reinterpret_cast<char *>(buf);
            xk[0][0] = std::strtod(pBuf, &end);
            for (iter = 1; iter < xk_size; iter++) {
                pBuf = end;
                xk[iter][0] = std::strtod(pBuf, &end);
            }
            v = fastGradientMethod(H, fi, F, xk, Rs, W);
            sprintf(reinterpret_cast<char *>(buf), "%f\n", v);
            HAL::Peripherals::GetInstance().sendString(buf, 100);
        }
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

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void){}

void send_string(const uint8_t* s) {
    HAL_UART_Transmit(&huart2, const_cast<uint8_t*>(s), strlen(reinterpret_cast<const char*>(s)), 1000);
}
