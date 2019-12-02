#include "Matrix.h"
#include "aproxMeanSquare.h"
#include <time.h>

using namespace std;

void exe1()

{
	double a = 0, b = 3, result;
	uint32_t flag = 1, size = 3;
	aproxMeanSquare ap = aproxMeanSquare(a, b, size);
	CMatrix matrix = CMatrix(3, 3); CVector vector = CVector(3); 
	CVector c = ap.resultGeneralised(matrix, vector, flag);
	ap.measureAMS(result, flag);
	cout << "Miara sredniokwadratowa: " << result << endl;
	ap.measureUniform(result, flag);
	cout << "Miara jednostajna: " << result << endl << endl;
	ap.erase();
	ap.save(c);
	cout << matrix << endl;
	cout << vector << endl;
	cout << c << "*********************" << endl << endl;

	size = 3;
	CMatrix matrix2 = CMatrix(size, size);
	aproxMeanSquare bp = aproxMeanSquare(a, b, size); CVector vector2 = CVector(size);
	for (flag =3; flag < 6; flag++)
	{
		CVector d = bp.resultOrthogonal(matrix2, vector2, flag);
		bp.save(d);
		bp.measureAMS(result, flag);
		cout << "Miara sredniokwadratowa: " << result << endl;
		bp.measureUniform(result, flag);
		cout << "Miara jednostajna: " << result << endl << endl;
		cout << matrix2 << endl;
		cout << vector2 << endl;
		cout << d << "////////////////////////////" << endl << endl;
		matrix2.zero();
		vector2.zero();
	}
	system("pause");
}

void exe2()
{
	uint32_t n = 6, m = 2, flag = 1;
	double U1[6][2] = { {1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5} };
	double Y1[6][1] = { 6, 4, 3.8, 2, 1.5, 1 };
	CMatrix U(n, m, *U1), product(m, m), product1(m, flag), b(m, flag);
	CVector Y(n, *Y1);
	CVector Yt = Y.Tv();
	CMatrix Ut = U.T();
	product = Ut*U;
	product.inverse();
	product1 = Ut*Yt;
	b = product*product1;
	cout << endl << "Szukane wspolczynniki:" << endl << b;
	b.save();
	system("pause");
}

void exe3()
{
	srand(time(NULL));
	double amplitude = 1, b1 = 2, b0 = 1, a=0, b, result = 0;
	double U1[6][2] = { { 1, 0 },{ 1, 1 },{ 1, 2 },{ 1, 3 },{ 1, 4 },{ 1, 5 } };
	uint32_t n = 6, m = 2, flag = 1;
	double *x = new double[n];
	for (uint32_t i = 0; i < n; i++)  x[i] = (double)i;
	double *y = new double[n];

	b = (double)a + n - 1;

	aproxMeanSquare ap = aproxMeanSquare(a, b, n, amplitude);
	ap.linearFunction(b1, b0, x, y);
	for (uint32_t i = 0; i < n; i++) cout << y[i] << " ";
	ap.generateNoise(y);
	cout << endl;
	for (uint32_t i = 0; i < n; i++) cout << y[i] << " ";
	
	CMatrix U(n, m, *U1), product(m, m), product1(m, flag), B(m, flag);
	CVector Y(n, y);
	CVector Yt = Y.Tv();
	CMatrix Ut = U.T();
	product = Ut*U;
	product.inverse();
	product1 = Ut*Yt;
	B = product*product1;
	cout << endl << "Szukane wspolczynniki:" << endl << B;
	ap.measureAMS(result, b1, b0, B);
	cout << "Miara sredniokwadratowa: " << result << endl;
	ap.measureUniform(result, b1, b0, B);
	cout << "Miara jednostajna: " << result << endl;
	B.save();
	system("pause");
}

int main()
{
	exe1();
	exe2();
	exe3();
	return 0;
}