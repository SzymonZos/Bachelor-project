#include "Matrix.h"

void CMatrix::Add(const CMatrix &m) throw (invalid_argument)
{
	if (rows != m.rows || columns != m.columns) throw invalid_argument("true");

	for (uint32_t i = 0; i < rows; i++)
		for (uint32_t j = 0; j < columns; j++)
			matrix[i][j] += m.matrix[i][j];
}

void CMatrix::Sub(const CMatrix &m) throw (invalid_argument)
{
	if (rows != m.rows || columns != m.columns) throw invalid_argument("true");

	for (uint32_t i = 0; i < rows; i++)
		for (uint32_t j = 0; j < columns; j++)
			matrix[i][j] -= m.matrix[i][j];
}

CMatrix CMatrix::Mul(const CMatrix &m) const throw (invalid_argument) //mnoze prawostronnie: macierz w private*macierz podana
{
	if (this->columns != m.rows) throw invalid_argument("true");

	CMatrix ProductMatrix(this->rows, m.columns);
	for (uint32_t i = 0; i < ProductMatrix.rows; i++)
		for (uint32_t j = 0; j < ProductMatrix.columns; j++)
			for (uint32_t k = 0; k < this->columns; k++)
				ProductMatrix.matrix[i][j] += (this->matrix[i][k] * m.matrix[k][j]);
	return ProductMatrix;
}

void CMatrix::Mul(const double &scalar) throw(invalid_argument)
{
	for (uint32_t i = 0; i < rows; i++)
		for (uint32_t j = 0; j < columns; j++)
			matrix[i][j] *= scalar;
}

void CMatrix::Div(const double &scalar) throw(invalid_argument)
{
	if (scalar == 0) throw invalid_argument("true");

	for (uint32_t i = 0; i < rows; i++)
		for (uint32_t j = 0; j < columns; j++)
			matrix[i][j] /= scalar;
}

CMatrix::CMatrix(uint32_t rows, uint32_t columns) //!< Konstruktor pustej macierzy
{
	this->rows = rows; //!< Zmiennej rows z klasy nadaj� warto�� rows.
	this->columns = columns; //!< A zmiennej columns columns.
	matrix = new double *[rows]; //!< Tworz� wektor.
	for (uint32_t i = 0; i<rows; i++)
	{
		matrix[i] = new double[columns]; //!< Wska�nik wska�nika, tablica 2D, czyli macierz.
	}
	for (uint32_t i = 0; i<rows; i++)
	{
		for (uint32_t j = 0; j<columns; j++)
			matrix[i][j] = 0; //!< Zeruj� elementy tablicy.
	}

}

CMatrix::CMatrix(uint32_t rows, uint32_t columns, double **mat) //!< Konstruktor nadaj�cy macierzy w klasie warto�� r�n� od zera.
{
	this->rows = rows;
	this->columns = columns;
	matrix = new double *[rows];
	for (uint32_t i = 0; i<rows; i++)
	{
		matrix[i] = new double[columns];
	}
	for (uint32_t i = 0; i<rows; i++)
	{
		for (uint32_t j = 0; j < columns; j++)
			matrix[i][j] = mat[i][j]; //!< Do macierzy z klasy zostaje przypisana macierz mat.
	}
}

CMatrix::CMatrix(uint32_t rows, uint32_t columns, double *mat) //!< Zapisanie wektora jako macierz.
{
	this->rows = rows;
	this->columns = columns;
	matrix = new double *[rows];
	for (uint32_t i = 0; i<rows; i++)
	{
		matrix[i] = new double[columns];
	}
	for (uint32_t i = 0; i<rows; i++)
	{
		for (uint32_t j = 0; j < columns; j++)
			matrix[i][j] = mat[i*columns+j]; //!< Przyjmuj�, �e wektor jest "poziomy" => ma jeden wers i n kolumn. i==0 pierwsze wyrazy mat, i==1 od miejsca zako�czenia.
	}
}

CMatrix::CMatrix(const CMatrix & M) //!< Konstruktor kopiuj�cy
{
	this->rows = M.rows;
	this->columns = M.columns;
	matrix = new double *[rows];
	for (uint32_t i = 0; i<rows; i++)
	{
		matrix[i] = new double[columns];
	}
	for (uint32_t i = 0; i<rows; i++)
	{
		for (uint32_t j = 0; j < columns; j++)
			matrix[i][j] = M.matrix[i][j];
	}
}

CMatrix::~CMatrix() //!< Destruktor, usuwam osobno wiersze i kolumny.
{
	for (uint32_t i = 0; i < rows; i++)
	{
		delete[]matrix[i];
	}
	delete[]matrix;
}

CMatrix &CMatrix:: operator = (const CMatrix &m) //!< Operator = b�dzie przypisywa� macierz do macierzy.
{
	for (uint32_t i = 0; i < rows; i++)
		for (uint32_t j = 0; j < columns; j++)
			matrix[i][j] = m.matrix[i][j];
	return *this; //!< Referencja.
}

void CMatrix::inverse()
{
	CMatrix temp(rows, columns, matrix);
	double detA = temp[0][0] * temp[1][1] - temp[1][0] * temp[0][1];

	temp[0][0] = matrix[1][1];
	temp[0][1] = -matrix[0][1];
	temp[1][0] = -matrix[1][0];
	temp[1][1] = matrix[0][0];

	for (uint32_t i = 0; i < 2; i++) {
		for (uint32_t j = 0; j < 2; j++) {
			matrix[i][j] = (1 / detA)*temp[i][j];
		}
	}
}

void CMatrix::zero()
{
	for (uint32_t i = 0; i < rows; i++)
		for (uint32_t j = 0; j < columns; j++)
			matrix[i][j] = 0;
}

void CMatrix::GetSize(uint32_t &rows, uint32_t &columns) const //!< Przekazuje za pomoc� referencji informacje o liczbie kolumn i wierszy macierzy z klasy.
{
	rows = this->rows;
	columns = this->columns;
}

double &CMatrix::GetElement(uint32_t row, uint32_t column) throw(invalid_argument) //!< Szuka elementu o podanym numerze kolumny i wiersza.
{
	if (row < 0 || column < 0 || row >= rows || column >= columns) throw invalid_argument("Error"); //!< Je�li podana liczba jest ujemna albo wychodzi poza zakres tablicy.
	return matrix[row][column];
}


double *CMatrix::operator[](uint32_t row) throw(invalid_argument) //!< Operator [] b�dzie zwraca� konkretny wiersz macierzy.
{
	if (row < 0 || row >= rows) throw invalid_argument("Error");
	
	return matrix[row];
}

CMatrix CMatrix::operator + (const CMatrix &m) const //!< Operator + b�dzie odpowiedzialny za dodawanie do siebie macierzy.
{
	CMatrix mat(*this); //!< Stworzenie macierzy do kt�rej b�dzie dodana macierz m.
	mat.Add(m); //!< Dodanie do stworzonej macierzy macierzy m.
	return mat; //!< Zwr�cenie otrzymanego wyniku.
}

CMatrix &CMatrix::operator += (const CMatrix &m) //!< Dodanie macierzy do istniej�cej ju� macierzy.
{
	this->Add(m);
	return *this;
}

CMatrix CMatrix::operator - (const CMatrix &m) const //!< Analogicznie jak +
{
	CMatrix mat(*this);
	mat.Sub(m);
	return mat;
}

CMatrix &CMatrix::operator -= (const CMatrix &m) //!< Analogicznie jak -
{
	this->Sub(m);
	return *this;
}

CMatrix CMatrix::operator * (const CMatrix &m) const //!< Mno�enie macierzy.
{
	CMatrix mat(*this);
	mat = this->Mul(m); //!< R�nica w por�wnaniu do dodawania i odejmowania polega na tym, �e metoda mno��ca macierze zwraca macierz (dodawanie i odejmowanie to void).
	return mat;
}

CMatrix &CMatrix::operator *= (const CMatrix &m)
{
	*this = this->Mul(m);
	return *this;
}
CMatrix CMatrix::operator * (const double &scalar) const //!< Analogia do +.
{
	CMatrix mat(*this);
	mat.Mul(scalar);
	return mat;
}
CMatrix &CMatrix::operator *= (const double &scalar) //!< Analogia do +=.
{
	this->Mul(scalar);
	return *this;
}
CMatrix CMatrix::operator / (const double &scalar) const //!< Analogia do +.
{
	CMatrix mat(*this);
	mat.Div(scalar);
	return mat;
}
CMatrix &CMatrix::operator /= (const double &scalar) //!< Analogia do +=.
{
	this->Div(scalar);
	return *this;
}
CMatrix CMatrix::operator -() const //!< Mno�enie przez skalar o warto�ci -1, mo�e tez by� dzielenie.
{
	CMatrix mat(*this);
	mat.Mul(-1);
	return mat;
}

CMatrix CMatrix::T() const //!< Transpozycja macierzy. 
{
	CMatrix matrix_t(columns, rows);
	for (uint32_t i = 0; i < rows; i++)
		for (uint32_t j = 0; j < columns; j++)
			matrix_t.matrix[j][i]=matrix[i][j];
	return matrix_t;
}

CVector::CVector(uint32_t columns) : CMatrix(1, columns) {} //!< Wektor "poziomy" o 1 wierszu.

CVector::CVector(uint32_t columns, uint32_t flag) : CMatrix (columns, 1){}

CVector::CVector(uint32_t columns, double *mat) : CMatrix(1, columns, mat) {} //!< Wektor poziomy o jednym wierszu.

void CVector::SetMatrixColumn(uint32_t column, CMatrix &m)
{
	uint32_t rows, columns;
	m.GetSize(rows, columns);

	for (uint32_t i = 0; i < rows; i++)
	{
		for (uint32_t j = 0; j < columns; j++)
			{
				if (column == j)
					m[i][j] = matrix[0][i];
			}
	}

}

CVector CVector::Tv() const
{
	uint32_t flag = 0;
	CVector vector_t(columns, flag);
		for (uint32_t i = 0; i < columns; i++)
			vector_t.matrix[i][0] = matrix[0][i];
	return vector_t;
}

CVector &CVector::operator = (const CMatrix &m) //!< Jako, �e warto�� macierzy w klasie przechowywana jest w polu private, musz� uzyska� o niej informacje za pomoc� metod publicznych.
{
	uint32_t rows, columns;
	m.GetSize(rows, columns);
	CMatrix *mat = new CMatrix(1, columns);
	for (uint32_t i = 0; i < columns; i++)
		matrix[0][i] = mat->GetElement(0, i);
	delete mat;
	return *this;
}

ostream& operator<< (ostream &stream, const CMatrix &m)
{
	uint32_t rows, columns;
	m.GetSize(rows, columns);
	for (uint32_t i = 0; i < rows; i++) {
		for (uint32_t j = 0; j < columns; j++) {
			stream << m.matrix[i][j] << " ";
		}
		stream << endl;
	}
	return stream;
}

void CMatrix::save()
{
	CMatrix matrix(rows, columns, matrix);
	ofstream file;
	file.open("C:\\Users\\Ganofir\\Documents\\Visual Studio 2017\\Projects\\aproksymacja2\\aproksymacja\\aproksymacja\\exe2_result.txt", ios::out | ios::app);
	file << matrix << endl;
	file.close();
}

