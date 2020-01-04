#include "utils/Misc.hpp"
#include <random>


double Utils::Misc::PowerMethod(const CMatrix& matrix, uint32_t maxIterations) {
    double greatest_eigenvalue_current = 0, greatest_eigenvalue_previous;
    std::random_device rd;
    std::mt19937 gen(rd());
    CVector b_k(matrix.GetRows(), 1), b_k1(matrix.GetRows(), 1);

    for (uint32_t i = 0; i < matrix.GetRows(); i++) {
        b_k[i][0] = std::generate_canonical<double, 10>(gen);
    }
    for (uint32_t i = 0; i < maxIterations; i++) {
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


void Utils::Misc::StringToDouble(const std::string& reference, std::vector<double>& dataToFill) {
    const char* begin = nullptr;
    char* end = const_cast<char*>(reference.c_str());
    double value;
    do {
        begin = end;
        value = std::strtod(begin, &end);
        if (begin != end) {
            dataToFill.push_back(value);
        }
    } while(begin != end);
}


void Utils::Misc::StringToDouble(const std::string& reference, CVector& dataToFill) {
    const char* begin = nullptr;
    char* end = const_cast<char*>(reference.c_str());
    double value;
    uint32_t i = 0;
    do {
        begin = end;
        value = std::strtod(begin, &end);
        if (begin != end) {
            dataToFill[i++][0] = value;
        }
    } while(begin != end && i < dataToFill.GetRows());
}
