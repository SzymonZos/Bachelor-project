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
    uint32_t prediction_horizon = 15, control_horizon = 3;
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

//  H, W
//  return {fi.T() * fi + Rw, fi.T() * ((F * xk) - Rs)};
    return {fi, Rw, F, Rs};
}

void calculateProjectedGradientStep(const CMatrix& H, const CMatrix& W, const CVector& xk, CVector& v, const double step) {
    uint32_t rows, columns;
    v.GetSize(rows, columns);
    CVector gradient(rows, 1);
    gradient = H * v + W;
    v -= gradient * step;
    for (uint32_t i = 0; i < rows; i++) {
        if (v[i][0] < minControlValue) {
            v[i][0] = minControlValue;
        }
        else if (v[i][0] > maxControlValue) {
            v[i][0] = maxControlValue;
        }
    }
}

void fastGradientMethod(const CMatrix& A, const CVector& B, const CVector& C, double r) {
    const uint32_t predictionHorizon = 3;
    double temp[predictionHorizon] = {0, 0, 0};
    uint32_t rowsMatrixA, columnsMatrixA;
    A.GetSize(rowsMatrixA, columnsMatrixA);
    CVector xk(rowsMatrixA, 1), v(predictionHorizon, 1, temp);
    CMatrix H(predictionHorizon, predictionHorizon), W(predictionHorizon, 1);
    CMatrix J(1,1), J_prev(1,1);
    auto[fi, Rw, F, Rs] = calculateOptimizationMatrices(A, B, C, r);
    H = fi.T() * fi + Rw;
    const double eps = 0.01, step = 1 / (power_iteration(H, 20));

    for (uint32_t j = 0; j < 100; j++) {
        W = fi.T() * ((F * xk) - Rs);
        for (uint32_t i = 0; i < 100; i++) {
            calculateProjectedGradientStep(H, W, xk, v, step);
            J_prev = J;
            J = v.T() * H * v / 2 + v.T() * W;
            if (std::fabs(J_prev[0][0] - J[0][0]) < eps) {
                break;
            }
        }
        xk = A * xk + B * v[0][0];
        std::cout << "\ny:\n[" << j << "] " << C * xk << "\nv:\n" << v;;
    }
    std::cout << power_iteration(H, 4);
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


#define N 4

// Function to get cofactor of A[p][q] in temp[][]. n is current
// dimension of A[][]
void getCofactor(int A[N][N], int temp[N][N], int p, int q, int n) {
    int i = 0, j = 0;
    // Looping for each element of the matrix
    for (int row = 0; row < n; row++) {
        for (int col = 0; col < n; col++) {
            //  Copying into temporary matrix only those element
            //  which are not in given row and column
            if (row != p && col != q) {
                temp[i][j++] = A[row][col];
                // Row is filled, so increase row index and
                // reset col index
                if (j == n - 1) {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

/* Recursive function for finding determinant of matrix.
   n is current dimension of A[][]. */
int determinant(int A[N][N], int n) {
    int D = 0; // Initialize result
    //  Base case : if matrix contains single element
    if (n == 1) {
        return A[0][0];
    }
    int temp[N][N]; // To store cofactors
    int sign = 1;  // To store sign multiplier

    // Iterate for each element of first row
    for (int f = 0; f < n; f++) {
        // Getting Cofactor of A[0][f]
        getCofactor(A, temp, 0, f, n);
        D += sign * A[0][f] * determinant(temp, n - 1);

        // terms are to be added with alternate sign
        sign = -sign;
    }

    return D;
}

// Function to get adjoint of A[N][N] in adj[N][N].
void adjoint(int A[N][N], int adj[N][N])
{
    if (N == 1) {
        adj[0][0] = 1;
        return;
    }
    // temp is used to store cofactors of A[][]
    int sign = 1, temp[N][N];
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            // Get cofactor of A[i][j]
            getCofactor(A, temp, i, j, N);

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i+j) % 2 ==0 ) ? 1 : -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            adj[j][i] = (sign)*(determinant(temp, N-1));
        }
    }
}

// Function to calculate and store inverse, returns false if
// matrix is singular
bool inverse(int A[N][N], float inverse[N][N])
{
    // Find determinant of A[][]
    int det = determinant(A, N);
    if (det == 0) {
        std::cout << "Singular matrix, can't find its inverse";
        return false;
    }

    // Find adjoint
    int adj[N][N];
    adjoint(A, adj);

    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (int i=0; i<N; i++) {
        for (int j = 0; j < N; j++) {
            inverse[i][j] = adj[i][j] / float(det);
        }
    }

    return true;
}

template<typename T>
void display(T A[N][N]) {
    for (int i=0; i<N; i++) {
        for (int j=0; j<N; j++) {
            std::cout << A[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

int main() {
//    char python[1024] = "'A': [1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1], 'B': [0, 0, 1, 0], 'C': [1, 1, 0, 0], 'setPoint': [10], 'controlExtremeValues': [-5, 5]";
//    std::string pythonString = python, valuesMatch;
//    std::regex namesPattern(R"(([[:alpha:]]+)(': )(\[.+?\]))");
//    std::smatch namesMatch;
//    std::string::const_iterator iterPtr(pythonString.cbegin());
//    std::map<std::string, std::vector<double>> dict;
//
//    while(std::regex_search(iterPtr, pythonString.cend(), namesMatch, namesPattern)) {
//        valuesMatch = namesMatch[3].str();
//        std::replace(valuesMatch.begin(), valuesMatch.end(), ',', ' ');
//        valuesMatch.pop_back(); // trim ]
//        valuesMatch.erase(0, 1); // trim [
//        if(namesMatch[1].str().find('A') != std::string::npos) {
//            stringToDouble(valuesMatch, dict["A"]);
//        }
//        else if(namesMatch[1].str().find('B') != std::string::npos) {
//            stringToDouble(valuesMatch, dict["B"]);
//        }
//        else if(namesMatch[1].str().find('C') != std::string::npos) {
//            stringToDouble(valuesMatch, dict["C"]);
//        }
//        else if(namesMatch[1].str().find("set") != std::string::npos) {
//            stringToDouble(valuesMatch, dict["set"]);
//        }
//        else if(namesMatch[1].str().find("control") != std::string::npos) {
//            stringToDouble(valuesMatch, dict["control"]);
//        }
//        iterPtr = namesMatch.suffix().first;
//    }
//
//    print_map(dict);
//    size_t dd = static_cast<size_t>(std::sqrt(dict["A"].size()));
//    CMatrix A(dd, dd, dict["A"].data());
//    CVector B(dict["B"].size(), 1, dict["B"].data()), C(4, dict["C"].data());
//    CVector xk(dict["C"].size(), 1);
//    double w = dict["set"][0];
//    minControlValue = dict["control"][0];
//    maxControlValue = dict["control"][1];
//    fastGradientMethod(A, B, C, w);

    int A[N][N] = { {5, -2, 2, 7},
                    {1, 0, 0, 3},
                    {-3, 1, 5, 0},
                    {3, -1, -9, 4}};

    int adj[N][N];  // To store adjoint of A[][]

    float inv[N][N]; // To store inverse of A[][]

    cout << "Input matrix is :\n";
    display(A);

    cout << "\nThe Adjoint is :\n";
    adjoint(A, adj);
    display(adj);

    cout << "\nThe Inverse is :\n";
    if (inverse(A, inv))
        display(inv);

    return 0;
}
