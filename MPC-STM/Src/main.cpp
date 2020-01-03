#include "HAL/main.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <map>
#include <vector>
#include <regex>
#include <tuple>
#include "utils/Matrix.hpp"
#include "utils/Misc.hpp"
#include "HAL/Peripherals.hpp"

// TODO: move this atrocity to class
double minControlValue = -0.5;
double maxControlValue = 0.5;
double w = 4;
uint8_t buf[1024];
uint16_t xk_size;
bool isNewDataGoingToBeSend;
uint32_t prediction_horizon = 10;
uint32_t control_horizon = 4;
double L, mi, step, eigen_const;

std::tuple<CMatrix, CMatrix, CMatrix, CVector> calculateOptimizationMatrices(const CMatrix& A, const CVector& B, const CVector& C, const double r) {
    double R1 = 1;
    CMatrix fi(prediction_horizon, control_horizon), Rw(control_horizon, "eye");
    CVector product_matrix(C.GetColumns());
    Rw *= R1;
    product_matrix = C * (A ^ (prediction_horizon - control_horizon));
    for (uint32_t i = prediction_horizon; i != 0; i--) {
        for (uint32_t j = control_horizon; j != 0; j--) {
            if (i == prediction_horizon) {
                if (j < control_horizon) {
                    product_matrix *= A;
                }
                fi[i-1][j-1] = (product_matrix * B)[0][0];
            } else if (i < j && j < control_horizon) {
                fi[i-1][j-1] = fi[i][j];
            }
        }
    }

    CMatrix F(prediction_horizon, C.GetColumns());
    CVector Rs(prediction_horizon, 1, std::to_string(r));
    product_matrix();
    product_matrix = C * A;
    for (uint32_t i = 0; i < prediction_horizon; i++) {
        F.SetRow(i, product_matrix);
        product_matrix *= A;
    }
    return {fi, Rw, F, Rs};
}

void calculateProjectedGradientStep(CVector& v, CVector& w_v, const CMatrix& H, const CMatrix& W) {
    uint32_t rows, columns;
    v.GetSize(rows, columns);
    CVector gradient(rows, 1), v_old = v;
    gradient = H * v + W;
    w_v = v - gradient * step;
    for (uint32_t i = 0; i < rows; i++) {
        if (w_v[i][0] < minControlValue) {
            v[i][0] = minControlValue;
        }
        else if (w_v[i][0] > maxControlValue) {
            v[i][0] = maxControlValue;
        } else {
            v[i][0] = w_v[i][0];
        }
    }
    w_v = v + (v - v_old) * eigen_const;
}

double fastGradientMethod(const CMatrix& H, const CMatrix& fi, const CMatrix& F, const CVector& xk, const CVector& Rs, CMatrix& W) {
    CVector v(control_horizon, 1), w_v = v;
    CMatrix J(1,1), J_prev(1,1);
    const double eps = 0.01;

    for (uint32_t i = 0; i < 100; i++) {
        W = fi.T() * ((F * xk) - Rs);
        calculateProjectedGradientStep(v, w_v, H, W);
        J_prev = J;
        J = v.T() * H * v / 2 + v.T() * W;
        if (std::fabs(J_prev[0][0] - J[0][0]) < eps) {
            break;
        }
    }
    return v[0][0];
}


#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

int main() {
    double v = 0;
    std::string pythonString, valuesMatch;
    std::regex pattern(R"(([[:alpha:]]+)(': )(\[.+?\]))");
    std::smatch fullMatch;
    std::string::const_iterator iterator;
    std::map<std::string, std::vector<double>> dict;
    size_t dimension;
    CMatrix A(4, 4);
    CVector B(4, 1), C(4), xk(4, 1);
    CMatrix H(control_horizon, control_horizon), W(control_horizon, 1);
    CMatrix fi(prediction_horizon, control_horizon), Rw(control_horizon, "eye");
    CMatrix F(prediction_horizon, C.GetColumns());
    CVector Rs(prediction_horizon, 1, std::to_string(1));

    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL::Peripherals::GetInstance().Init();

    while (true) {
        if (isNewDataGoingToBeSend) {
            isNewDataGoingToBeSend = false;
            // 20s wait after pressing button to read data sent from PC
            HAL::Peripherals::GetInstance().ReceiveString(buf, 20000);
            if (*buf == '\'') {
                pythonString = reinterpret_cast<char*>(buf);
                iterator = pythonString.cbegin();
                while (std::regex_search(iterator, pythonString.cend(), fullMatch, pattern)) {
                    valuesMatch = fullMatch[3].str();
                    std::replace(valuesMatch.begin(), valuesMatch.end(), ',', ' ');
                    valuesMatch.pop_back(); // trim ]
                    valuesMatch.erase(0, 1); // trim [
                    if (fullMatch[1].str().find('A') != std::string::npos) {
                        Utils::Misc::StringToDouble(valuesMatch, dict["A"]);
                    } else if (fullMatch[1].str().find('B') != std::string::npos) {
                        Utils::Misc::StringToDouble(valuesMatch, dict["B"]);
                    } else if (fullMatch[1].str().find('C') != std::string::npos) {
                        Utils::Misc::StringToDouble(valuesMatch, dict["C"]);
                    } else if (fullMatch[1].str().find("set") != std::string::npos) {
                        Utils::Misc::StringToDouble(valuesMatch, dict["set"]);
                    } else if (fullMatch[1].str().find("control") != std::string::npos) {
                        Utils::Misc::StringToDouble(valuesMatch, dict["control"]);
                    } else if (fullMatch[1].str().find("horizon") != std::string::npos) {
                        Utils::Misc::StringToDouble(valuesMatch, dict["horizons"]);
                    }
                    iterator = fullMatch.suffix().first;
                }
                dimension = static_cast<size_t>(std::sqrt(dict["A"].size()));
                xk_size = dict["C"].size();
                A(dimension, dimension, dict["A"].data());
                B(dict["B"].size(), 1, dict["B"].data());
                C(1, xk_size, dict["C"].data());
                xk(xk_size, 1);
                w = dict["set"][0];
                minControlValue = dict["control"][0];
                maxControlValue = dict["control"][1];
                prediction_horizon = dict["horizons"][0];
                control_horizon = dict["horizons"][1];
                F(prediction_horizon, xk_size);
                fi(prediction_horizon, control_horizon);
                Rw(control_horizon, control_horizon, "eye");
                Rs(prediction_horizon, 1);
                H(control_horizon, control_horizon);
                W(control_horizon, 1);
                std::tie(fi, Rw, F, Rs) = calculateOptimizationMatrices(A, B, C, w);
                H = fi.T() * fi + Rw;
                HAL::Peripherals::GetInstance().SendString(buf, 100);
                L = Utils::Misc::PowerMethod(H, 20), mi = Utils::Misc::PowerMethod(H.Inverse(), 20);
                step = 1 / L, eigen_const = (std::sqrt(L) - std::sqrt(mi)) / (std::sqrt(L) + std::sqrt(mi));

                dict.clear();
//                sprintf(reinterpret_cast<char *>(buf), "%f %f %f\n", w, minControlValue, maxControlValue);
            }
        }
        else {
            HAL::Peripherals::GetInstance().ReceiveString(buf, 100);
            Utils::Misc::StringToDouble(reinterpret_cast<char *>(buf), xk);
            v = fastGradientMethod(H, fi, F, xk, Rs, W);
            sprintf(reinterpret_cast<char *>(buf), "%f\n", v);
            HAL::Peripherals::GetInstance().SendString(buf, 1000);
        }
    }
}

#pragma clang diagnostic pop