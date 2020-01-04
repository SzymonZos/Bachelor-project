#ifndef MPC_STM_MPC_HPP
#define MPC_STM_MPC_HPP

#include "Parameters.hpp"
#include <map>
#include <vector>
#include <string>

namespace Control {
    class MPC {
    public:
        MPC() = default;
        ~MPC() = default;
        double FastGradientMethod(const std::string& msg);
        void InitializeParameters(std::map<std::string, std::vector<double>> storage);

    private:
        constexpr static const double eps = 0.01;
        SystemMatrices sys;
        OptimizationMatrices opt;
        ControlValues controlValues{};
        Horizons horizons{};
        Eigenvalues eigenvalues{};

        void CalculateProjectedGradientStep(CVector& v, CVector& w);
        void CalculateOptimizationMatrices();
    };
}


#endif //MPC_STM_MPC_HPP
