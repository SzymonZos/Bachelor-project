#include <iostream>
#include "Matrix.h"

const double minControlValue = -100.0;
const double maxControlValue = 100.0;

typedef enum {
    success,
    failure
} result;

result calculateOptimizationMatrices(const CMatrix& A, const CVector& B, CMatrix& H, CMatrix& F) {
    H = A;
    F = A;
    return success;
}

result calculateGradient(const CMatrix& H, const CMatrix& F, const CVector& xk, const CVector& v, CVector& gradient) {
    gradient = H * v + F * xk;
    return success;
}

result calculateProjectedGradientStep(const CMatrix& H, const CMatrix& F, const CVector& xk, CVector& v, const double step) {
    uint32_t rows, columns;
    v.GetSize(rows, columns);
    CVector gradient(rows, 1);
    calculateGradient(H, F, xk, v, gradient);
    v -= gradient * step;
    for (uint32_t i = 0; i < rows; i++) {
        if (v[i][0] < minControlValue) {
            v[i][0] = minControlValue;
        }
        else if (v[i][0] > maxControlValue) {
            v[i][0] = maxControlValue;
        }
    }
//    std::cout << "v:\n" << v << std::endl;
    return success;
}

result fastGradientMethod(const CMatrix& A, const CVector& B) {
    const uint32_t predictionHorizon = 3;
    double temp[predictionHorizon] = {maxControlValue, maxControlValue, maxControlValue};
    uint32_t rowsMatrixA, columnsMatrixA;
    A.GetSize(rowsMatrixA, columnsMatrixA);
    CVector xk(rowsMatrixA, 1), v(predictionHorizon, 1, temp);
    CMatrix H(rowsMatrixA, columnsMatrixA), F(predictionHorizon, predictionHorizon);

    calculateOptimizationMatrices(A, B, H, F);
    for (uint32_t j = 0; j < 10; j++) {
        for (uint32_t i = 0; i < 30; i++) {
            calculateProjectedGradientStep(H, F, xk, v, 0.1);
        }
        xk = A * xk + B * v[0][0];
        std::cout << "\nxk:\n" << xk << "\nv:\n" << v;;
    }
    return success;
}
int main() {
    double temp_A[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9}, temp_B[3] = {1, 0, 0};
    CMatrix A(3, 3, temp_A);
    CVector B(3, 1, temp_B);
    fastGradientMethod(A, B);
    std::cout << "Hejka";
    return 0;
}
