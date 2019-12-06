#include <iostream>
#include <ctgmath>
#include "Matrix.h"

const double minControlValue = -15;
const double maxControlValue = 15;

typedef enum {
    success,
    failure
} result;

result calculateOptimizationMatrices(const CMatrix& A, const CVector& B, const CVector& xk, const double w, CMatrix& H, CMatrix& F) {
//    CMatrix h11(1,1), F11(1, 1), F12(1,1), F13(1,1);
//    CVector a1(3);
//    for (uint32_t i = 0; i < 3; ++i) {
//        a1[0][i] = A[0][i];
//    }
//    h11 = a1 * B;
//    H[0][0] = 2 * (1 + B[0][0] * B[0][0] + h11[0][0] * h11[0][0]);
//    H[0][1] = 2 * h11[0][0] * B[0][0];
//    H[1][0] = H[0][1];
//    H[1][1] = 2 * (1 + B[0][0] * B[0][0]);
//    H[2][2] = 2;
//
//    F11 = a1 * xk;
//    F12 = a1 * A * xk;
//    F[0][0] = 2 * (F11[0][0] * B[0][0] - w * B[0][0] + F12[0][0] * h11[0][0] - w * h11[0][0]);
//    F[1][0] = 2 * (F12[0][0] * B[0][0] - w * B[0][0]);

    F = B;
    H = A;
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

result fastGradientMethod(const CMatrix& A, const CVector& B) {
    const uint32_t predictionHorizon = 3;
    const double w = 100.0, eps = 0.01;
    double temp[predictionHorizon] = {0, 0, 0};
    uint32_t rowsMatrixA, columnsMatrixA;
    A.GetSize(rowsMatrixA, columnsMatrixA);
    CVector xk(rowsMatrixA, 1), v(predictionHorizon, 1, temp);
    CMatrix H(rowsMatrixA, columnsMatrixA), F(predictionHorizon, 1);
    CMatrix J(1,1), J_prev(1,1);

    for (uint32_t j = 0; j < 1; j++) {
        calculateOptimizationMatrices(A, B, xk, w, H, F);
        for (uint32_t i = 0; i < 100; i++) {
            calculateProjectedGradientStep(H, F, xk, v, 0.2);
//            J_prev = J;
//            J = v.T() * H * v / 2 + v.T() * F * xk;
//            if (std::fabs(J_prev[0][0] - J[0][0]) < eps) {
//                std::cout << std::endl << "i = " << i << " J = " << J << "J_prev = " << J_prev;
//                break;
//            }
        }
        xk = A * xk + B * v[0][0];
        std::cout << "\nxk:\n[" << j << "] " << xk << "\nv:\n" << v;;
    }
    return success;
}
int main() {
   // double temp_A[9] = {-0.3833, -0.18, -0.1067, 0.25, 0, 0, 0, 0.0625, 0}, temp_B[3] = {0.5, 0, 0};
    double temp_A[9] = {5, -2, -1, -2, 4, 3, -1, 3, 5}, temp_B[3] = {2, -35, -47};

    CMatrix A(3, 3, temp_A);
    CVector B(3, 1, temp_B);
    fastGradientMethod(A, B);
//    std::cout << "Hejka";
    return 0;
}
