#include <iostream>
#include <ctgmath>
#include <algorithm>
#include <map>
#include <vector>
#include <regex>
#include "Matrix.h"

const double minControlValue = -0.5;
const double maxControlValue = 0.5;

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

result fastGradientMethod(const CMatrix& A, const CVector& B, const CVector& C, double r) {
    const uint32_t predictionHorizon = 3;
    const double eps = 0.01;
    double temp[predictionHorizon] = {0, 0, 0};
    uint32_t rowsMatrixA, columnsMatrixA;
    A.GetSize(rowsMatrixA, columnsMatrixA);
    CVector xk(rowsMatrixA, 1), v(predictionHorizon, 1, temp);
    CMatrix H(predictionHorizon, predictionHorizon), W(predictionHorizon, 1);
    CMatrix J(1,1), J_prev(1,1);

    for (uint32_t j = 0; j < 100; j++) {
        calculateOptimizationMatrices(A, B, C, xk, r, H, W);
        for (uint32_t i = 0; i < 100; i++) {
            calculateProjectedGradientStep(H, W, xk, v, 0.1);
            J_prev = J;
            J = v.T() * H * v / 2 + v.T() * W;
            if (std::fabs(J_prev[0][0] - J[0][0]) < eps) {
//                std::cout << std::endl << "i = " << i << " J = " << J << "J_prev = " << J_prev;
                break;
            }
        }
        xk = A * xk + B * v[0][0];
        std::cout << "\nxk:\n[" << j << "] " << C * xk << "\nv:\n" << v;;
    }
    return success;
}

result stringToDouble(const char* c_string, double* c_array, unsigned length) {
    char* end;
    for (unsigned iter = 0; iter < length; iter++) {
        c_array[iter] = std::strtod(c_string, &end);
        if (c_string == end) {
            return failure;
        }
        c_string = end;
    }
    return success;
}

int main() {
    double temp_A[16], temp_B[4], temp_C[4], controlExtremeValues[2], w;
    char *dummy;
    char python[1024] = "'A': [1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1], 'B': [0, 0, 1, 0], 'C': [1, 1, 0, 0], 'setPoint': [10], 'controlExtremeValues': [-0.5, 0.5]";
    std::string pythonString = python, valuesMatch;
    std::regex namesPattern(R"(([[:alpha:]]+)(': )(\[.+?\]))");
    std::smatch namesMatch;
    std::string::const_iterator iterPtr(pythonString.cbegin());
    size_t currentSize;
    std::map<std::string, std::vector<double>> dict;

    while(std::regex_search(iterPtr, pythonString.cend(), namesMatch, namesPattern)) {
        valuesMatch = namesMatch[3].str();
        currentSize = std::count(valuesMatch.begin(), valuesMatch.end(), ',') + 1;
        std::replace(valuesMatch.begin(), valuesMatch.end(), ',', ' ');
        valuesMatch.pop_back(); // trim ]
        valuesMatch.erase(0, 1); // trim [
        std::cout << valuesMatch.length() << std::endl;
        if(namesMatch[1].str().find('A') != std::string::npos) {
            dict["A"] = currentSize;
            stringToDouble(valuesMatch.c_str(), temp_A, 16);
        }
        else if(namesMatch[1].str().find('B') != std::string::npos) {
            dict["A"] = currentSize;
            stringToDouble(valuesMatch.c_str(), temp_B, 4);
        }
        else if(namesMatch[1].str().find('C') != std::string::npos) {
            stringToDouble(valuesMatch.c_str(), temp_C, 4);
        }
        else if(namesMatch[1].str().find("set") != std::string::npos) {
            w = std::strtod(valuesMatch.c_str(), &dummy);
        }
        else if(namesMatch[1].str().find("control") != std::string::npos) {
            stringToDouble(valuesMatch.c_str(), controlExtremeValues, 2);
        }
        iterPtr = namesMatch.suffix().first;
    }

    CMatrix A(4, 4, temp_A);
    CVector B(4, 1, temp_B), C(4, temp_C);
//    fastGradientMethod(A, B, C, w);
    return 0;
}
