#include "aproxMeanSquare.h"
#include "Matrix.h"

//uint32_t flag; //1: generalised; 2, 3, 4, 5: orthogonal; czebyszew, legendre, hermitte, laguerre

aproxMeanSquare::aproxMeanSquare() : c(CVector (3)), vectorOrth(CVector (3))
{
	this->a = 0;
	this->b = 2;
	this->m = 50000;
	this->n = 3;
	this->i = 0;
	this->j = 0;
	this->amplitude = 0;
}

aproxMeanSquare::aproxMeanSquare(double a, double b, uint32_t size) : c(CVector(3)), vectorOrth(CVector(size))
{
	this->a = a;
	this->b = b;
	this->size = size;
	this->m = 50000;
	this->n = 3;
	this->i = 0;
	this->j = 0;
	this->amplitude = 0;
}

aproxMeanSquare::aproxMeanSquare(double a, double b, double m, uint32_t size) : c(CVector(3)), vectorOrth(CVector(size))
{
	this->a = a;
	this->b = b;
	this->size = size;
	this->m = m;
	this->n = 3;
	this->i = 0;
	this->j = 0;
	this->amplitude = 0;
}

aproxMeanSquare::aproxMeanSquare(double a, double b, uint32_t n, double amplitude) : c(CVector(0)), vectorOrth(CVector(0))
{
	this->a = a;
	this->b = b;
	this->size = 0;
	this->m = 100;
	this->n = n;
	this->i = 0;
	this->j = 0;
	this->amplitude = amplitude;
}

aproxMeanSquare::~aproxMeanSquare()
{
}

double aproxMeanSquare::f(double x)
{
	return pow(x, 1.0 / 3.0);
}

double aproxMeanSquare::g(double x, uint32_t flag)
{
	double fi = 0;
	double fj = 0;
	switch (flag)
	{
	case 1:
	{
		fi = pow(x, i);
		fj = pow(x, j);
		return fi*fj;
	}
	case 2:
	{
		fi = czebyszew(x);
		return fi * fi / sqrt(1-pow(x,2));
	}
	case 3:
	{	
		fi = legendre(x);
		return fi * fi;
	}
	case 4:
	{
		fi = hermitte(x);
		return fi * fi * exp(-pow(x, 2));
	}
	case 5:
	{
		fi = laguerre(x);
		return fi * fi * exp(-pow(x, 2));
	}
	default: return -1;
	}
}

double aproxMeanSquare::gf(double x, uint32_t flag)
{

	double fF = f(x);
	double fi = 1;
	switch (flag)
	{
	case 1:
	{
		fi = pow(x, i);
		return fi * fF;
	}
	case 2:
	{
		fi = czebyszew(x);
		return fi * fF / sqrt(1 - pow(x, 2));
	}
	case 3:
	{
		fi = legendre(x);
		return fi * fF;
	}
	case 4:
	{
		fi = hermitte(x);
		return fi * fF * exp(-pow(x, 2));
	}
	case 5:
	{
		fi = laguerre(x);
		return fi * fF * exp(-pow(x, 2));
	}
	default: return -1;
	}
}

double aproxMeanSquare::gAMS(double x, uint32_t flag)
//uint32_t flag; //1: generalised; 2, 3, 4, 5: orthogonal; czebyszew, legendre, hermitte, laguerre
{
	double sumX = 0, integrand;
	for (uint32_t i = 0; i < n; i++)
	{
		if (flag == 1) sumX += c[0][i] * pow(x, i);
		else sumX += vectorOrth[0][i] * pow(x, i);
		//cout << endl << "Sumx: " << sumX << endl;
	}
	integrand = pow(sumX - f(x), 2);
	switch (flag)
	{
	case 1:
	{
		return integrand;
	}
	case 2:
	{
		return integrand / sqrt(1 - pow(x, 2));
	}
	case 3:
	{
		return integrand;
	}
	case 4:
	{
		return integrand * exp(-pow(x, 2));
	}
	case 5:
	{
		return integrand * exp(-pow(x, 2));
	}
	default: return -1;
	}
}

double aproxMeanSquare::gAMS(double x, double b1, double b0, CMatrix B)
{
	double integrand;
	integrand = pow(b1*x + b0 - B[1][0] * x - B[0][0], 2);
	return integrand;
}

double aproxMeanSquare::intsim(double b1, double b0, CMatrix B)
{
	double h = (b - a) / (2 * m);
	double sum = 0;
	sum = gAMS(a, b1, b0, B) + gAMS(b, b1, b0, B);
		for (int k = 1; k < 2 * m; k++)
		{
			if (k % 2 == 1) sum += 4 * gAMS(a + k*h, b1, b0, B);
			if (k % 2 == 0) sum += 2 * gAMS(a + k*h, b1, b0, B);
		}
		
	return sum*h / 3;
}

double aproxMeanSquare::czebyszew(double x)
{
	double T0 = 1;
	double T1 = x;
	double Tnext = 2 * x * T1 - T0;
	if (i >= 2) {
		for (uint32_t j = 1; j < i; j++)
			{
				Tnext = 2 * x * T1 - T0;
				T0 = T1;
				T1 = Tnext;
			}
		return Tnext;
		}
	else if (i == 1) return T1;
	return T0;
}

double aproxMeanSquare::legendre(double x)
{
	double T0 = 1;
	double T1 = x;
	double Tnext = ((2 * (size-1) + 1) * x * T1 - (size - 1) * T0) / ((size - 1) + 1);
	if (i >= 2) {
		for (uint32_t j = 1; j < i; j++)
		{
			Tnext = ((2 * (size - 1) + 1) * x * T1 - (size - 1) * T0) / ((size - 1) + 1);
			T0 = T1;
			T1 = Tnext;
		}
		return Tnext;
	}
	else if (i == 1) return T1;
	return T0;
}

double aproxMeanSquare::hermitte(double x)
{
	double T0 = 1;
	double T1 = 2 * x;
	double Tnext = 2 * x * T1 - 2 * (size - 1) * T0;
	if (i >= 2) {
		for (uint32_t j = 1; j < i; j++)
		{
			Tnext = 2 * x * T1 - 2 * (size - 1) * T0;
			T0 = T1;
			T1 = Tnext;
		}
		return Tnext;
	}
	else if (i == 1) return T1;
	return T0;
}

double aproxMeanSquare::laguerre(double x)
{
	double T0 = 1;
	double T1 = 1 - x; 
	double Tnext = ((1 + 2 * (size - 1) - x) * T1 - (size - 1) * T0) / (n+1);
	if (i >= 2) {
		for (uint32_t j = 1; j < i; j++)
		{
			Tnext = ((1 + 2 * (size - 1) - x) * T1 - (size - 1) * T0) / (n + 1);
			T0 = T1;
			T1 = Tnext;
		}
		return Tnext;
	}
	else if (i == 1) return T1;
	return T0;
}

void aproxMeanSquare::linearFunction(double b1, double b0, double *x, double *y)
{
	for (uint32_t i = 0; i < n; i++)
	{
		y[i] = b1*x[i] + b0;
	}
}

void aproxMeanSquare::generateNoise(double * y)
{
	for (uint32_t i = 0; i < n; i++)
	{
		y[i] += ((double)rand() / RAND_MAX) * amplitude;
	}
}

double aproxMeanSquare::measureAMS(double &result, uint32_t flag)
{
	result = intsim(3, flag);
	result = pow(result, 0.5);
	return result;
}

double aproxMeanSquare::measureAMS(double &result, double b1, double b0, CMatrix B)
{
	result = intsim(b1, b0, B);
	result = pow(result, 0.5);
	return result;
}

double aproxMeanSquare::measureUniform(double &result, uint32_t flag)
{
	double sumX = 0;
	double h = (b - a) / (2 * m), umax = 0, u;

	for (double x = a; x < b; x += h)
	{
		sumX = 0;
		for (uint32_t i = 0; i < n; i++)
		{
			if (flag == 1) sumX += c[0][i] * pow(x, i);
			else sumX += vectorOrth[0][i] * pow(x, i);
		}
		u = fabs(f(x) - sumX);
		//sumU += u;
		//cout << u << endl;
		if (u > umax) umax = u;
	}
	result = umax;
	return result;
}

double aproxMeanSquare::measureUniform(double & result, double b1, double b0, CMatrix B)
{
	double sumX = 0, umax = 0, u;

	for (double x = a; x < b; x++)
	{
		u = fabs(b1*x + b0 - B[1][0] * x - B[0][0]);
		if (u > umax) umax = u;
	}
	result = umax;
	return result;
}

double aproxMeanSquare::intsim(uint32_t fl, uint32_t flag)
{
	double h = (b - a) / (2 * m);
	double sum = 0;
	switch (fl)
	{
	case 1:
	{
		sum = g(a, flag) + g(b, flag);
		for (int k = 1; k < 2 * m; k++)
		{
			if (k % 2 == 1) sum += 4 * g(a + k*h, flag);
			if (k % 2 == 0) sum += 2 * g(a + k*h, flag);
		}
		break;
	}
	case 2:
	{
		sum = gf(a, flag) + gf(b, flag);
		for (int k = 1; k < 2 * m; k++)
		{
			if (k % 2 == 1) sum += 4 * gf(a + k*h, flag);
			if (k % 2 == 0) sum += 2 * gf(a + k*h, flag);
		}
		break;
	}
	case 3:
	{
		sum = gAMS(a, flag) + gAMS(b, flag);
		for (int k = 1; k < 2 * m; k++)
		{
			if (k % 2 == 1) sum += 4 * gAMS(a + k*h, flag);
			if (k % 2 == 0) sum += 2 * gAMS(a + k*h, flag);
		}
		break;
	}
	}
	return sum*h / 3;
}

double aproxMeanSquare::determinant(uint32_t d, CMatrix matrix, CVector constantTerm)
{
	CMatrix temp(matrix);
	if (d != 0)
	{
		constantTerm.SetMatrixColumn(d - 1, temp);
	}

	if (n == 3)
		return temp[0][0] * temp[1][1] * temp[2][2] + temp[0][1] * temp[1][2] * temp[2][0] + temp[0][2] * temp[1][0] * temp[2][1]
		- temp[0][2] * temp[1][1] * temp[2][0] - temp[0][0] * temp[1][2] * temp[2][1] - temp[0][1] * temp[1][0] * temp[2][2];
	if (n == 2)
		return temp[0][0] * temp[1][1] - temp[1][0] * temp[0][1];
	return 1;
}

CVector aproxMeanSquare::resultGeneralised(CMatrix &matrix, CVector &constantTerm, uint32_t flag)
{
	uint32_t d = 0;

	for (i = 0; i < n; i++)
	{
		constantTerm[0][i] = intsim(2, flag);
		for (j = 0; j < n; j++)
		{
			matrix[i][j] = intsim(1, flag);
		}
	}

	double W = determinant(d, matrix, constantTerm);

	for (d = 1; d < n + 1; d++)
	{
		c[0][d - 1] = determinant(d, matrix, constantTerm) / W;
	}
	return c;
}

CVector aproxMeanSquare::resultOrthogonal(CMatrix & matrix, CVector & constantTerm, uint32_t flag)
{
	for (i = 0; i < size; i++)
	{
		constantTerm[0][i] = intsim(2, flag);
		matrix[i][i] = intsim(1, flag);
		vectorOrth[0][i] = constantTerm[0][i] / matrix[i][i];
	}
	return vectorOrth;
}

void aproxMeanSquare::save(CVector vector)
{
	ofstream file;
	file.open("C:\\Users\\Ganofir\\Documents\\Visual Studio 2017\\Projects\\aproksymacja2\\aproksymacja\\aproksymacja\\exe1_result.txt", ios::out | ios::app);
	file << vector << endl;
	file.close();
	//system("D:\\Programoz\\MATLAB\\bin\\matlab.exe -nodisplay -nosplash -nodesktop -r run('C:\\Users\\Ganofir\\Documents\\Visual_ Studio_ 2017\\Projects\\aproksymacja2\\aproksymacja\\aproksymacjamatlabplot.m')");
}

void aproxMeanSquare::erase()
{
	remove("C:\\Users\\Ganofir\\Documents\\Visual Studio 2017\\Projects\\aproksymacja2\\aproksymacja\\aproksymacja\\exe1_result.txt");
}
