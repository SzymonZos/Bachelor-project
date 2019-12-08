#include <iostream>
#include <ctgmath>
#include "Matrix.h"

const double minControlValue = -15;
const double maxControlValue = 15;

typedef enum {
    success,
    failure
} result;

result calculateOptimizationMatrices(const CMatrix& A, const CVector& B, const CVector& C, const CVector& xk, const double r, CMatrix& H, CMatrix& W) {
    CMatrix fi(3, 3), Rw(3, 3);
    CVector F(3), Rs(3);

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
    std::cout << v << std::endl;
    return success;
}

result fastGradientMethod(const CMatrix& A, const CVector& B, const CVector& C) {
    const uint32_t predictionHorizon = 2;
    const double r = 100.0, eps = 0.1;
    double temp[predictionHorizon] = {0, 0};
    uint32_t rowsMatrixA, columnsMatrixA;
    A.GetSize(rowsMatrixA, columnsMatrixA);
    CVector xk(rowsMatrixA, 1), v(predictionHorizon, 1, temp);
    CMatrix H(predictionHorizon, predictionHorizon), W(predictionHorizon, 1);
    CMatrix J(1,1), J_prev(1,1);

    for (uint32_t j = 0; j < 100; j++) {
        calculateOptimizationMatrices(A, B, C, xk, r, H, W);
        for (uint32_t i = 0; i < 100; i++) {
            calculateProjectedGradientStep(H, W, xk, v, 0.2);
            J_prev = J;
            J = v.T() * H * v / 2 + v.T() * W;
            if (std::fabs(J_prev[0][0] - J[0][0]) < eps) {
                std::cout << std::endl << "i = " << i << " J = " << J << "J_prev = " << J_prev;
                break;
            }
        }
        xk = A * xk + B * v[0][0];
        std::cout << "\nxk:\n[" << j << "] " << xk << "\nv:\n" << v;;
    }
    return success;
}
int main() {
    double temp_A[16] = {1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1};
    double temp_B[4] = {0, 0, 1, 0}, temp_C[4] = {1, 1, 0, 0};

    CMatrix A(3, 3, temp_A);
    CVector B(3, 1, temp_B), C(4, temp_C);
    fastGradientMethod(A, B, C);
//    std::cout << "Hejka";
    return 0;
}
