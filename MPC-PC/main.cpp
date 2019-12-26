#include <iostream>
#include <ctgmath>
#include <algorithm>
#include <map>
#include <vector>
#include <regex>
#include "Matrix.h"

double minControlValue;
double maxControlValue;

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
        char python[1024] = "'A': [1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1], 'B': [0, 0, 1, 0], 'C': [1, 1, 0, 0], 'setPoint': [10], 'controlExtremeValues': [-5, 5]";
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
        }
        iterPtr = namesMatch.suffix().first;
    }

    print_map(dict);
    size_t dd = static_cast<size_t>(std::sqrt(dict["A"].size()));
    CMatrix A(dd, dd, dict["A"].data());
    CVector B(dict["B"].size(), 1, dict["B"].data()), C(4, dict["C"].data());
    double w = dict["set"][0];
    minControlValue = dict["control"][0];
    maxControlValue = dict["control"][1];
//    fastGradientMethod(A, B, C, w);
    double temp1[4] = {9, 8, 7, 6};
    A(2, 2, temp1);
    std::cout << A;
    CMatrix temp2(2, 2, {1, 2, 3, 4});
    std::cout << temp2;
    temp2(3, 3, {6, 7, 8, 3, 5, 0, 2, 4, 5});
    std::cout << temp2();
    return 0;
}
