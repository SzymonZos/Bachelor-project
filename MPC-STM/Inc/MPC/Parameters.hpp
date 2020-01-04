#ifndef MPC_STM_PARAMETERS_HPP
#define MPC_STM_PARAMETERS_HPP

#include "utils/Matrix.hpp"
namespace Control {

    typedef struct {
        CMatrix A;
        CVector B;
        CVector C;
        CVector x;
    } SystemMatrices;

    typedef struct {
        CMatrix H;
        CMatrix W;
        CMatrix fi;
        CMatrix F;
        CMatrix Rw;
        CVector Rs;
        double scalarRw;
        double scalarRs;
    } OptimizationMatrices;

    typedef struct {
        double min;
        double max;
    } ControlValues;

    typedef struct {
        uint32_t prediction;
        uint32_t control;
    } Horizons;

    typedef struct {
        double min;
        double max;
        double step;
        double fastConvergence;
    } Eigenvalues;
}
#endif //MPC_STM_PARAMETERS_HPP
