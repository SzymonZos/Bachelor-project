#include "MPC/Mpc.hpp"
#include "utils/DataParser.hpp"
#include "utils/Misc.hpp"
#include <cmath>


double Control::MPC::FastGradientMethod(const std::string& msg) {
    CVector v(horizons.control, 1), w = v;
    CMatrix J(1,1), J_prev(1,1);
    Utils::Misc::StringToDouble(msg, sys.x);

    for (uint32_t i = 0; i < 100; i++) {
        opt.W = opt.fi.T() * ((opt.F * sys.x) - opt.Rs);
        CalculateProjectedGradientStep(v, w);
        J_prev = J;
        J = v.T() * opt.H * v / 2 + v.T() * opt.W;
        if (std::fabs(J_prev[0][0] - J[0][0]) < eps) {
            break;
        }
    }
    return v[0][0];
}


void Control::MPC::InitializeParameters(std::map<std::string, std::vector<double>> storage) {
    uint32_t dimension = static_cast<uint32_t>(storage["C"].size());

    sys.A(dimension, dimension, storage["A"].data());
    sys.B(dimension, 1, storage["B"].data());
    sys.C(1, dimension, storage["C"].data());
    sys.x(dimension, 1);

    controlValues.min = storage["control"][0];
    controlValues.max = storage["control"][1];

    horizons.prediction = storage["horizons"][0];
    horizons.control = storage["horizons"][1];

    opt.F(horizons.prediction, dimension);
    opt.fi(horizons.prediction, horizons.control);
    opt.Rw(horizons.control, horizons.control, "eye");
    opt.Rs(horizons.prediction, 1);
    opt.H(horizons.control, horizons.control);
    opt.W(horizons.control, 1);
    opt.scalarRs = storage["set"][0];
    opt.scalarRw = 1;

    CalculateOptimizationMatrices();

    eigenvalues.max = Utils::Misc::PowerMethod(opt.H, 20);
    eigenvalues.min = Utils::Misc::PowerMethod(opt.H.Inverse(), 20);
    eigenvalues.step = 1 / eigenvalues.max;
    eigenvalues.fastConvergence = (std::sqrt(eigenvalues.max) - std::sqrt(eigenvalues.min)) /
                                  (std::sqrt(eigenvalues.max) + std::sqrt(eigenvalues.min));
}


void Control::MPC::CalculateProjectedGradientStep(CVector& v, CVector& w) {
    CVector gradient(v.GetRows(), 1), v_old = v;
    gradient = opt.H * v + opt.W;
    w = v - gradient * eigenvalues.step;
    for (uint32_t i = 0; i < v.GetRows(); i++) {
        if (w[i][0] < controlValues.min) {
            v[i][0] = controlValues.min;
        }
        else if (w[i][0] > controlValues.max) {
            v[i][0] = controlValues.max;
        } else {
            v[i][0] = w[i][0];
        }
    }
    w = v + (v - v_old) * eigenvalues.fastConvergence;
}


void Control::MPC::CalculateOptimizationMatrices() {
    opt.Rw *= opt.scalarRw;
    opt.Rs.SetValue(opt.scalarRs);
    CVector productMatrix(sys.C.GetColumns());
    productMatrix = sys.C * (sys.A ^ (horizons.prediction - horizons.control));
    for (uint32_t i = horizons.prediction; i != 0; i--) {
        for (uint32_t j = horizons.control; j != 0; j--) {
            if (i == horizons.prediction) {
                if (j < horizons.control) {
                    productMatrix *= sys.A;
                }
                opt.fi[i-1][j-1] = (productMatrix * sys.B)[0][0];
            } else if (i < j && j < horizons.control) {
                opt.fi[i-1][j-1] = opt.fi[i][j];
            }
        }
    }
    opt.H = opt.fi.T() * opt.fi + opt.Rw;
    productMatrix = sys.C * sys.A;
    for (uint32_t i = 0; i < horizons.prediction; i++) {
        opt.F.SetRow(i, productMatrix);
        productMatrix *= sys.A;
    }
}
