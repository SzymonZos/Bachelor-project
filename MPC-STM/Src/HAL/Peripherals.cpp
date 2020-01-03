#include "HAL/Peripherals.hpp"
#include <cstring>


HAL::Peripherals::Peripherals() : initGPIO{}, handleUART{}, oscillator{}, clock{} {
    handleUART.Instance = USART2;
    handleUART.Init.BaudRate = 115200;
    handleUART.Init.WordLength = UART_WORDLENGTH_8B;
    handleUART.Init.StopBits = UART_STOPBITS_1;
    handleUART.Init.Parity = UART_PARITY_NONE;
    handleUART.Init.Mode = UART_MODE_TX_RX;
    handleUART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    handleUART.Init.OverSampling = UART_OVERSAMPLING_16;
}


HAL::Peripherals& HAL::Peripherals::GetInstance() {
    static Peripherals instance;
    return instance;
}


UART_HandleTypeDef HAL::Peripherals::GetHandleUART() {
    return handleUART;
}


void HAL::Peripherals::Init() {
    HAL_Init();
    ConfigureSystemClock();
    InitializeGPIO();
}


void HAL::Peripherals::sendString(const uint8_t* string, uint16_t timeout) {
    HAL_UART_Transmit(&handleUART, const_cast<uint8_t*>(string), std::strlen(reinterpret_cast<const char*>(string)), timeout);
}


void HAL::Peripherals::receiveString(const uint8_t* string, uint16_t timeout) {
    uint8_t bufSize = 0;
    HAL_UART_Receive(&handleUART, &bufSize, 1, timeout);
    HAL_UART_Receive(&handleUART, const_cast<uint8_t*>(string), bufSize, 100);
}


void HAL::Peripherals::ConfigureSystemClock() {
    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    // Initializes the CPU, AHB and APB busses clocks
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    oscillator.PLL.PLLM = 16;
    oscillator.PLL.PLLN = 336;
    oscillator.PLL.PLLP = RCC_PLLP_DIV4;
    oscillator.PLL.PLLQ = 7;

    // Initializes the CPU, AHB and APB busses clocks
    clock.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV2;
    clock.APB2CLKDivider = RCC_HCLK_DIV1;
}

void HAL::Peripherals::InitializeGPIO() {
    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure GPIO pin Output Level
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    // Configure GPIO pin : B1_Pin
    initGPIO.Pin = B1_Pin;
    initGPIO.Mode = GPIO_MODE_IT_FALLING;
    initGPIO.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &initGPIO);

    // Configure GPIO pin : LD2_Pin
    initGPIO.Pin = LD2_Pin;
    initGPIO.Mode = GPIO_MODE_OUTPUT_PP;
    initGPIO.Pull = GPIO_NOPULL;
    initGPIO.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &initGPIO);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
