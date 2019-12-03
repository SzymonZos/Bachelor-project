#include "Matrix.h"

void CMatrix::Add(const CMatrix &m)
{
	if (rows != m.rows || columns != m.columns) {
	    std::cout << "rows: " << rows << " m.rows: " << m.rows << " columns: " << columns << " m.columns: " << m.columns << std::endl;
	    throw invalid_argument("Add: Mismatch of matrices' dimensions");
	}

	for (uint32_t i = 0; i < rows; i++)
		for (uint32_t j = 0; j < columns; j++)
			matrix[i][j] += m.matrix[i][j];
}

void CMatrix::Sub(const CMatrix &m)
{
	if (rows != m.rows || columns != m.columns) {
	    throw invalid_argument("Sub: Mismatch of matrices' dimensions");
	}

	for (uint32_t i = 0; i < rows; i++)
		for (uint32_t j = 0; j < columns; j++)
			matrix[i][j] -= m.matrix[i][j];
}

CMatrix CMatrix::Mul(const CMatrix &m) const
{
	if (this->columns != m.rows) {
	    throw invalid_argument("Mul: Mismatch of matrices' dimensions");
	}

	CMatrix ProductMatrix(this->rows, m.columns);
	for (uint32_t i = 0; i < ProductMatrix.rows; i++)
		for (uint32_t j = 0; j < ProductMatrix.columns; j++)
			for (uint32_t k = 0; k < this->columns; k++)
				ProductMatrix.matrix[i][j] += (this->matrix[i][k] * m.matrix[k][j]);
	return ProductMatrix;
}

void CMatrix::Mul(const double &scalar)
{
	for (uint32_t i = 0; i < rows; i++)
		for (uint32_t j = 0; j < columns; j++)
			matrix[i][j] *= scalar;
}

void CMatrix::Div(const double &scalar)
{
	if (scalar == 0) throw invalid_argument("Div: Cannot divide by 0");

	for (uint32_t i = 0; i < rows; i++)
		for (uint32_t j = 0; j < columns; j++)
			matrix[i][j] /= scalar;
}

CMatrix::CMatrix(uint32_t rows, uint32_t columns) : rows(rows), columns(columns) //!< Konstruktor pustej macierzy
{
	matrix = new double *[rows]; //!< Tworzê wektor.
	for (uint32_t i = 0; i<rows; i++) {
		matrix[i] = new double[columns]; //!< WskaŸnik wskaŸnika, tablica 2D, czyli macierz.
	}
	for (uint32_t i = 0; i < rows; i++) {
		for (uint32_t j = 0; j<columns; j++) {
            matrix[i][j] = 0; //!< Zerujê elementy tablicy.
        }
	}
}

CMatrix::CMatrix(uint32_t rows, uint32_t columns, double **mat) : rows(rows), columns(columns) //!< Konstruktor nadaj¹cy macierzy w klasie wartoœæ ró¿n¹ od zera.
{
	matrix = new double *[rows];
	for (uint32_t i = 0; i<rows; i++) {
		matrix[i] = new double[columns];
	}
	for (uint32_t i = 0; i<rows; i++) {
		for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = mat[i][j]; //!< Do macierzy z klasy zostaje przypisana macierz mat.
        }
	}
}

CMatrix::CMatrix(uint32_t rows, uint32_t columns, double *mat) : rows(rows), columns(columns) //!< Zapisanie wektora jako macierz.
{
	matrix = new double *[rows];
	for (uint32_t i = 0; i < rows; i++) {
		matrix[i] = new double[columns];
	}
	for (uint32_t i = 0; i < rows; i++) {
		for (uint32_t j = 0; j < columns; j++) {
			matrix[i][j] = mat[i*columns+j]; //!< Przyjmujê, ¿e wektor jest "poziomy" => ma jeden wers i n kolumn. i==0 pierwsze wyrazy mat, i==1 od miejsca zakoñczenia.
		}
	}
}

CMatrix::CMatrix(const CMatrix & M) : rows(M.rows), columns(M.columns) //!< Konstruktor kopiuj¹cy
{
	matrix = new double *[rows];
	for (uint32_t i = 0; i<rows; i++) {
		matrix[i] = new double[columns];
	}
	for (uint32_t i = 0; i<rows; i++) {
		for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = M.matrix[i][j];
        }
	}
}

CMatrix::~CMatrix() //!< Destruktor, usuwam osobno wiersze i kolumny.
{
	for (uint32_t i = 0; i < rows; i++) {
		delete[] matrix[i];
	}
	delete[] matrix;
}

CMatrix &CMatrix:: operator = (const CMatrix &m) //!< Operator = bêdzie przypisywaæ macierz do macierzy.
{
	for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = m.matrix[i][j];
        }
    }
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
	for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = 0;
        }
    }
}

void CMatrix::GetSize(uint32_t &rows, uint32_t &columns) const //!< Przekazuje za pomoc¹ referencji informacje o liczbie kolumn i wierszy macierzy z klasy.
{
	rows = this->rows;
	columns = this->columns;
}

double &CMatrix::GetElement(uint32_t row, uint32_t column) const
{
	if (row < 0 || column < 0 || row >= rows || column >= columns) {
        throw invalid_argument("GetElement: Requested index is out of range");
    }
	return matrix[row][column];
}


double *CMatrix::operator[](uint32_t row)
{
	if (row < 0 || row >= rows) {
	    throw invalid_argument("[]: Requested index is out of range");
	}
	return matrix[row];
}

CMatrix CMatrix::operator + (const CMatrix &m) const //!< Operator + bêdzie odpowiedzialny za dodawanie do siebie macierzy.
{
	CMatrix mat(*this); //!< Stworzenie macierzy do której bêdzie dodana macierz m.
	mat.Add(m); //!< Dodanie do stworzonej macierzy macierzy m.
	return mat; //!< Zwrócenie otrzymanego wyniku.
}

CMatrix &CMatrix::operator += (const CMatrix &m) //!< Dodanie macierzy do istniej¹cej ju¿ macierzy.
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

CMatrix CMatrix::operator * (const CMatrix &m) const //!< Mno¿enie macierzy.
{
	CMatrix mat(*this);
	mat = this->Mul(m); //!< Ró¿nica w porównaniu do dodawania i odejmowania polega na tym, ¿e metoda mno¿¹ca macierze zwraca macierz (dodawanie i odejmowanie to void).
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
CMatrix CMatrix::operator -() const //!< Mno¿enie przez skalar o wartoœci -1, mo¿e tez byæ dzielenie.
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

CVector::CVector(uint32_t rows, uint32_t flag) : CMatrix (rows, 1){}

CVector::CVector(uint32_t columns, double *mat) : CMatrix(1, columns, mat) {} //!< Wektor poziomy o jednym wierszu.

CVector::CVector(uint32_t rows, uint32_t flag, double *mat) : CMatrix(rows, 1, mat) {}

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

CVector &CVector::operator = (const CMatrix &m) //!< Jako, ¿e wartoœæ macierzy w klasie przechowywana jest w polu private, muszê uzyskaæ o niej informacje za pomoc¹ metod publicznych.
{
	uint32_t rows, columns;
	m.GetSize(rows, columns);
	for (uint32_t i = 0; i < columns; i++)
		matrix[0][i] = m.GetElement(0, i);
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
//	CMatrix matrix(rows, columns, matrix);
	ofstream file;
	file.open("C:\\Users\\Ganofir\\Documents\\Visual Studio 2017\\Projects\\aproksymacja2\\aproksymacja\\aproksymacja\\exe2_result.txt", ios::out | ios::app);
	file << matrix << endl;
	file.close();
}

