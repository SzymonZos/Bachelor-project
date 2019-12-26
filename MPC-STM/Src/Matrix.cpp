#include "Matrix.h"


void CMatrix::Add(const CMatrix& m) {
    if (rows != m.rows || columns != m.columns) {
        throw std::invalid_argument("Add: Mismatch of matrices' dimensions");
    }
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] += m.matrix[i][j];
        }
    }
}


void CMatrix::Sub(const CMatrix& m) {
    if (rows != m.rows || columns != m.columns) {
        throw std::invalid_argument("Sub: Mismatch of matrices' dimensions");
    }
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] -= m.matrix[i][j];
        }
    }
}


CMatrix CMatrix::Mul(const CMatrix& m) const {
    if (columns != m.rows) {
        throw std::invalid_argument("Mul: Mismatch of matrices' dimensions");
    }
    CMatrix productMatrix(rows, m.columns);
    for (uint32_t i = 0; i < productMatrix.rows; i++) {
        for (uint32_t j = 0; j < productMatrix.columns; j++) {
            for (uint32_t k = 0; k < columns; k++) {
                productMatrix.matrix[i][j] += (matrix[i][k] * m.matrix[k][j]);
            }
        }
    }
    return productMatrix;
}


void CMatrix::Mul(const double& scalar) {
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] *= scalar;
        }
    }
}


void CMatrix::Div(const double& scalar) {
    if (scalar == 0) {
        throw std::invalid_argument("Div: Cannot divide by 0");
    }
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] /= scalar;
        }
    }
}


CMatrix::CMatrix(uint32_t rows, uint32_t columns) : rows(rows), columns(columns) {
    matrix = new double*[rows];
    for (uint32_t i = 0; i < rows; i++) {
        matrix[i] = new double[columns];
    }
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = 0;
        }
    }
}


CMatrix::CMatrix(uint32_t rows, uint32_t columns, double** mat) : rows(rows), columns(columns) {
    matrix = new double*[rows];
    for (uint32_t i = 0; i < rows; i++) {
        matrix[i] = new double[columns];
    }
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = mat[i][j];
        }
    }
}


CMatrix::CMatrix(uint32_t rows, uint32_t columns, const double* mat) : rows(rows), columns(columns) {
    matrix = new double*[rows];
    for (uint32_t i = 0; i < rows; i++) {
        matrix[i] = new double[columns];
    }
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = mat[i * columns + j];
        }
    }
}


CMatrix::CMatrix(const CMatrix& M) : rows(M.rows), columns(M.columns) {
    matrix = new double*[rows];
    for (uint32_t i = 0; i < rows; i++) {
        matrix[i] = new double[columns];
    }
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = M.matrix[i][j];
        }
    }
}


CMatrix::CMatrix(uint32_t rows, uint32_t columns, std::initializer_list<double> list) : rows(rows), columns(columns) {
    auto iter = list.begin();
    matrix = new double*[rows];
    for (uint32_t i = 0; i < rows; i++) {
        matrix[i] = new double[columns];
    }
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = *iter++;
        }
    }
}


CMatrix::~CMatrix() {
    for (uint32_t i = 0; i < rows; i++) {
        delete[] matrix[i];
    }
    delete[] matrix;
}


CMatrix& CMatrix:: operator= (const CMatrix& m) {
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = m.matrix[i][j];
        }
    }
    return *this;
}


void CMatrix::GetSize(uint32_t& rows, uint32_t& columns) const {
    rows = this->rows;
    columns = this->columns;
}


double& CMatrix::GetElement(uint32_t row, uint32_t column) const {
    if (row < 0 || column < 0 || row >= rows || column >= columns) {
        throw std::invalid_argument("GetElement: Requested index is out of range");
    }
    return matrix[row][column];
}


double* CMatrix::operator[](uint32_t row) const {
    if (row < 0 || row >= rows) {
        throw std::invalid_argument("[]: Requested index is out of range");
    }
    return matrix[row];
}


CMatrix CMatrix::operator+ (const CMatrix& m) const {
    CMatrix mat(*this);
    mat.Add(m);
    return mat;
}


CMatrix& CMatrix::operator+= (const CMatrix& m) {
    this->Add(m);
    return *this;
}


CMatrix CMatrix::operator- (const CMatrix& m) const {
    CMatrix mat(*this);
    mat.Sub(m);
    return mat;
}


CMatrix& CMatrix::operator-= (const CMatrix& m) {
    this->Sub(m);
    return *this;
}


CMatrix CMatrix::operator* (const CMatrix& m) const {
    return this->Mul(m);
}


CMatrix& CMatrix::operator*= (const CMatrix& m) {
    *this = this->Mul(m);
    return *this;
}


CMatrix CMatrix::operator* (const double& scalar) const {
    CMatrix mat(*this);
    mat.Mul(scalar);
    return mat;
}


CMatrix& CMatrix::operator*= (const double& scalar) {
    this->Mul(scalar);
    return *this;
}


CMatrix CMatrix::operator/ (const double& scalar) const {
    CMatrix mat(*this);
    mat.Div(scalar);
    return mat;
}


CMatrix& CMatrix::operator/= (const double& scalar) {
    this->Div(scalar);
    return *this;
}


CMatrix CMatrix::operator-() const {
    CMatrix mat(*this);
    mat.Mul(-1);
    return mat;
}


std::ostream& operator<< (std::ostream& stream, const CMatrix& m) {
    uint32_t rows, columns;
    m.GetSize(rows, columns);
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            stream << m.matrix[i][j] << " ";
        }
        stream << std::endl;
    }
    return stream;
}


CMatrix CMatrix::operator()() {
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = 0;
        }
    }
    return *this;
}


CMatrix CMatrix::operator()(uint32_t rows, uint32_t columns, const double* mat) {
    this->~CMatrix();
    this->rows = rows;
    this->columns = columns;
    matrix = new double*[rows];

    for (uint32_t i = 0; i < rows; i++) {
        matrix[i] = new double[columns];
    }
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = mat[i * columns + j];
        }
    }
    return *this;
}


CMatrix CMatrix::operator()(uint32_t rows, uint32_t columns, std::initializer_list<double> list) {
    this->~CMatrix();
    this->rows = rows;
    this->columns = columns;
    auto iter = list.begin();
    matrix = new double*[rows];

    for (uint32_t i = 0; i < rows; i++) {
        matrix[i] = new double[columns];
    }
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix[i][j] = *iter++;
        }
    }
    return *this;
}


CMatrix CMatrix::T() const {
    CMatrix matrix_t(columns, rows);
    for (uint32_t i = 0; i < rows; i++) {
        for (uint32_t j = 0; j < columns; j++) {
            matrix_t.matrix[j][i] = matrix[i][j];
        }
    }
    return matrix_t;
}


CVector::CVector(uint32_t columns) : CMatrix(1, columns) {}


CVector::CVector(uint32_t rows, uint32_t column) : CMatrix(rows, 1) {}


CVector::CVector(uint32_t columns, double* mat) : CMatrix(1, columns, mat) {}


CVector::CVector(uint32_t rows, uint32_t column, double* mat) : CMatrix(rows, 1, mat) {}


CVector& CVector::operator= (const CMatrix& m) {
    uint32_t rows, columns;
    m.GetSize(rows, columns);
    if (rows == 1) {
        for (uint32_t i = 0; i < columns; i++) {
            matrix[0][i] = m.GetElement(0, i);
        }
    }
    else {
        for (uint32_t i = 0; i < rows; i++) {
            matrix[i][0] = m.GetElement(i, 0);
        }
    }
    return *this;
}
