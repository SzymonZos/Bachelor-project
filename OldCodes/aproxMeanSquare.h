#pragma once
#include <iostream>
#include <cstdio>
#include <cmath>
#include <fstream>
#include "Matrix.h"

using namespace std;

class aproxMeanSquare
{
	friend class CMatrix;
private:
	double a; //lower limit of the integral
	double b; //upper limit of the integral
	double m; //amount of sub-intervals in Simpson method
	uint32_t n; //n-1 is degree of the generalised polynomial which is used to approximate given function
	uint32_t size; //degree of the orthogonal polynomial
	CVector c; //result; vector that consists of coeffients of the generalised polynomial
	CVector vectorOrth; //vector that consists of coefficients of the orthogonal polynomial

	uint32_t i; //present number of the row
	uint32_t j; //present number of the column

	double amplitude;

	double f(double x); //declared function
	double g(double x, uint32_t flag); //integrand function in the matrix A
	double gf(double x, uint32_t flag); //integrand function in the vector b
	double gAMS(double x, uint32_t flag);

	double gAMS(double x, double b1, double b0, CMatrix B);
	double intsim(double b1, double b0, CMatrix B);

	double czebyszew(double x);
	double legendre(double x);
	double hermitte(double x);
	double laguerre(double x);

	double intsim(uint32_t fl, uint32_t flag); //Simpson method, flag decides which functions are used
	double determinant(uint32_t d, CMatrix matrix, CVector constantTerm); //calculates determinant of matrix 3x3 or 2x2

public:
	aproxMeanSquare();
	aproxMeanSquare(double a, double b, uint32_t size);
	aproxMeanSquare(double a, double b, double m, uint32_t size);
	aproxMeanSquare(double a, double b, uint32_t n, double amplitude);
	~aproxMeanSquare();

	void linearFunction(double b1, double b0, double *x, double *y);
	void generateNoise(double *y);
	double measureAMS(double &result, uint32_t flag);
	double measureAMS(double &result, double b1, double b0, CMatrix B);
	double measureUniform(double &result, uint32_t flag);
	double measureUniform(double &result, double b1, double b0, CMatrix B);

	CVector resultGeneralised(CMatrix &matrix, CVector &constantTerm, uint32_t flag); //calculates coefficients in matrix A and b; solves this set of equations; generalised polynomial
	CVector resultOrthogonal(CMatrix &matrix, CVector &constantTerm, uint32_t flag); //orthogonal polynomial

	void save(CVector vector);
	void erase();
};

