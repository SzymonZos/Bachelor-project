#ifndef MPC_STM_PERIPHERALS_HPP
#define MPC_STM_PERIPHERALS_HPP

#include "HAL/main.h"

namespace HAL {
    class Peripherals {
    public:
        Peripherals(const Peripherals&) = delete;
        Peripherals& operator=(const Peripherals&) = delete;
        Peripherals(Peripherals&&) = delete;
        Peripherals& operator=(Peripherals&&) = delete;

        static Peripherals& GetInstance();
        UART_HandleTypeDef GetHandleUART();
        void Init();
        void SendString(const uint8_t* string, uint16_t timeout);
        void ReceiveString(const uint8_t* string, uint16_t timeout);

    private:
        UART_HandleTypeDef handleUART;
        RCC_OscInitTypeDef oscillator;
        RCC_ClkInitTypeDef clock;
        GPIO_InitTypeDef initGPIO;

        Peripherals();
        void ConfigureSystemClock();
        void InitializeGPIO();
    };
}


#endif //MPC_STM_PERIPHERALS_HPP
