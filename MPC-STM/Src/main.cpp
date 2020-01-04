#include "HAL/main.h"
#include "HAL/Peripherals.hpp"
#include "MPC/Mpc.hpp"
#include "utils/DataParser.hpp"

int main() {
    uint8_t buf[1024] = {};
    double v = 0;
    Control::MPC mpc;
    Utils::DataParser dataParser;
    HAL::Peripherals::GetInstance().Init();

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (true) {
        if (Utils::DataParser::isNewDataGoingToBeSend) {
                Utils::DataParser::isNewDataGoingToBeSend = false;
            // 20s wait after pressing button to read data sent from PC
            HAL::Peripherals::GetInstance().ReceiveString(buf, 20000);
            if (*buf == '\'') {
                dataParser.ParseReceivedMsg(reinterpret_cast<char*>(buf));
                mpc.InitializeParameters(dataParser.GetStorage());
                dataParser.ClearStorage();
            }
        }
        else {
            HAL::Peripherals::GetInstance().ReceiveString(buf, 100);
            try {
                v = mpc.FastGradientMethod(reinterpret_cast<char*>(buf));
                sprintf(reinterpret_cast<char*>(buf), "%f\n", v);
            } catch (const std::exception& e) {
                sprintf(reinterpret_cast<char*>(buf), "%s\n", e.what());
            }
            HAL::Peripherals::GetInstance().SendString(buf, 100);
        }
    }
#pragma clang diagnostic pop
}