#ifndef MPC_STM_MISC_HPP
#define MPC_STM_MISC_HPP

#include "utils/Matrix.hpp"
#include <vector>

namespace Utils {
    class Misc {
    public:
        static double PowerMethod(const CMatrix& matrix, uint32_t maxIterations);
        static void StringToDouble(const std::string& reference, std::vector<double>& dataToFill);
        static void StringToDouble(const std::string& reference, CVector& dataToFill);
    };
}

#endif //MPC_STM_MISC_HPP
