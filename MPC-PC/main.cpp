#include <iostream>
#include <ctgmath>
#include <algorithm>
#include <map>
#include <vector>
#include <regex>
#include "Matrix.h"
#include <tuple>
#include <random>

double minControlValue;
double maxControlValue;
const uint32_t prediction_horizon = 15;
const uint32_t control_horizon = 3;

double power_iteration(const CMatrix& H, uint32_t max_number_of_iterations) {
    double greatest_eigenvalue_current = 0, greatest_eigenvalue_previous;
    std::random_device rd;
    std::mt19937 gen(rd());
    CVector b_k(H.GetRows(), 1), b_k1(H.GetRows(), 1);
    for (uint32_t i = 0; i < H.GetRows(); i++) {
        b_k[i][0] = std::generate_canonical<double, 10>(gen);
    }
    for (uint32_t i = 0; i < max_number_of_iterations; i++) {
        b_k1 = H * b_k;
        b_k = b_k1 / std::sqrt((b_k1 * b_k1.T())[0][0]);
        greatest_eigenvalue_previous = greatest_eigenvalue_current;
        greatest_eigenvalue_current = ((H * b_k) * b_k.T())[0][0] / (b_k * b_k.T())[0][0];
        if (std::fabs(greatest_eigenvalue_current - greatest_eigenvalue_previous) < 0.01) {
            break;
        }
    }
    return std::fabs(greatest_eigenvalue_current);
}

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

void calculateProjectedGradientStep(const CMatrix& H, const CMatrix& W, const CVector& xk, CVector& v, CVector& w, const double step, const double eigen_const) {
    uint32_t rows, columns;
    v.GetSize(rows, columns);
    CVector gradient(rows, 1), v_old = v;
    gradient = H * v + W;
    w = v - gradient * step;
    for (uint32_t i = 0; i < rows; i++) {
        if (w[i][0] < minControlValue) {
            v[i][0] = minControlValue;
        }
        else if (w[i][0] > maxControlValue) {
            v[i][0] = maxControlValue;
        } else {
            v[i][0] = w[i][0];
        }
    }
    w = v + (v - v_old) * eigen_const;
}

void fastGradientMethod(const CMatrix& A, const CVector& B, const CVector& C, double r) {
    double temp[control_horizon] = {0, 0, 0};
    uint32_t rowsMatrixA, columnsMatrixA;
    A.GetSize(rowsMatrixA, columnsMatrixA);
    CVector xk(rowsMatrixA, 1), v(control_horizon, 1, temp), w = v;
    CMatrix H(control_horizon, control_horizon), W(control_horizon, 1);
    CMatrix J(1,1), J_prev(1,1);
    auto[fi, Rw, F, Rs] = calculateOptimizationMatrices(A, B, C, r);
    H = fi.T() * fi + Rw;
    std::cout << H;
    const double L = power_iteration(H, 20), mi = power_iteration(H.Inverse(), 20);
    const double eps = 0.01, step = 1 / L, eigen_const = (std::sqrt(L) - std::sqrt(mi)) / (std::sqrt(L) + std::sqrt(mi));

    for (uint32_t j = 0; j < 100; j++) {
        W = fi.T() * ((F * xk) - Rs);
        for (uint32_t i = 0; i < 100; i++) {
            calculateProjectedGradientStep(H, W, xk, v, w, step, eigen_const);
            J_prev = J;
            J = v.T() * H * v / 2 + v.T() * W;
            if (std::fabs(J_prev[0][0] - J[0][0]) < eps) {
                break;
            }
        }
        xk = A * xk + B * v[0][0];
        std::cout << "\ny:\n[" << j << "] " << C * xk << "\nv:\n" << v;;
    }
}

void stringToDouble(const std::string& data_reference, std::vector<double>& data_to_fill) {
    const char* begin = nullptr;
    char* end = const_cast<char*>(data_reference.c_str());
    double value;
    do {
        begin = end;
        value = std::strtod(begin, &end);
        if (begin != end) {
            data_to_fill.push_back(value);
        }
    } while(begin != end);
}

template<typename __Map>
void print_map(const __Map& m)
{
    std::cout << "{";
    for(const auto& p : m) {
        std::cout << '\'' << p.first << "': [";
        for(const auto& v : p.second) {
            std::cout << v << ", ";
        }
        std::cout << "\b\b], ";
    }
    std::cout << "\b\b}\n";
}

int main() {
    char python[1024] = "'A': [1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1], 'B': [0, 0, 1, 0], 'C': [1, 1, 0, 0], 'setPoint': [10], 'controlExtremeValues': [-10, 10], 'horizons': [15, 3]";
    std::string pythonString = python, valuesMatch;
    std::regex namesPattern(R"(([[:alpha:]]+)(': )(\[.+?\]))");
    std::smatch namesMatch;
    std::string::const_iterator iterPtr(pythonString.cbegin());
    std::map<std::string, std::vector<double>> dict;

    while(std::regex_search(iterPtr, pythonString.cend(), namesMatch, namesPattern)) {
        valuesMatch = namesMatch[3].str();
        std::replace(valuesMatch.begin(), valuesMatch.end(), ',', ' ');
        valuesMatch.pop_back(); // trim ]
        valuesMatch.erase(0, 1); // trim [
        if(namesMatch[1].str().find('A') != std::string::npos) {
            stringToDouble(valuesMatch, dict["A"]);
        }
        else if(namesMatch[1].str().find('B') != std::string::npos) {
            stringToDouble(valuesMatch, dict["B"]);
        }
        else if(namesMatch[1].str().find('C') != std::string::npos) {
            stringToDouble(valuesMatch, dict["C"]);
        }
        else if(namesMatch[1].str().find("set") != std::string::npos) {
            stringToDouble(valuesMatch, dict["set"]);
        }
        else if(namesMatch[1].str().find("control") != std::string::npos) {
            stringToDouble(valuesMatch, dict["control"]);
        } else if (namesMatch[1].str().find("horizon") != std::string::npos) {
            stringToDouble(valuesMatch, dict["horizons"]);
        }
        iterPtr = namesMatch.suffix().first;
    }

    print_map(dict);
    size_t dd = static_cast<size_t>(std::sqrt(dict["A"].size()));
    CMatrix A(dd, dd, dict["A"].data());
    CVector B(dict["B"].size(), 1, dict["B"].data()), C(4, dict["C"].data());
    CVector xk(dict["C"].size(), 1);
    double w = dict["set"][0];
    minControlValue = dict["control"][0];
    maxControlValue = dict["control"][1];
    fastGradientMethod(A, B, C, w);

    return 0;
}
